#include "cdc_task.h"
#include "aimassist_task.h"
#include "judgement_info.h"

#define CDC_TX_BUF_SIZE 256
#define CDC_RX_BUF_SIZE 256

#define Upload_CMD_1_SIZE 22//21
#define Upload_CMD_2_SIZE 45
#define Upload_CMD_3_MAX_SIZE 200
#define Upload_CMD_4_SIZE 4

uint8_t cdc_send_mode = CDC_CAMERA_IMU_CALIBRATION;
uint32_t CDC_Receive_Stamp = 0;

void CDC_Init(void) {
    CDC_Receive_Stamp = INS_GetTimeline();
    AimAssist.YawPositon_lpf_coef = YAW_POSITION_LPF;
    AimAssist.PitchPosition_lpf_coef = PITCH_POSITON_LPF;
}

void CDC_Task(void) {
    if (INS_GetTimeline() - CDC_Receive_Stamp > 1000) {
        AimAssist.miniPC_Online = 0;
        AimAssist.Status = TgtLost;
        ShootEvaluation.Allow_Shoot = 0;
        ShootEvaluation.Shoot_Freq = 0;
        AimAssist.auxistatus=TgtLost;
    }
    if (robot_state.robot_id >= RED_HERO && robot_state.robot_id <= RED_SENTRY) {
        AimAssist.Team = RED_TEAM;
    } else if (robot_state.robot_id >= BLUE_HERO && robot_state.robot_id <= BLUE_SENTRY) {
        AimAssist.Team = BLUE_TEAM;
    }
}

// CRC16 --- new
typedef enum {
    SOF_1 = 0xA5,
    SOF_2 = 0x5A,
    EOF_1 = 0xFF
} KeyFrame;

uint16_t
CRC16_Check(const uint8_t *data, uint8_t len) {
    uint16_t crc_16 = 0xFFFF;
    for (uint8_t i = 0; i < len; ++i) {
        crc_16 ^= data[i];
        uint8_t state;
        for (uint8_t j = 0; j < 8; ++j) {
            state = crc_16 & 0x01;
            crc_16 >>= 1;
            if (state)
                crc_16 ^= 0xA001;
        }
    }
    return crc_16;
}

void addPacket2Buff(uint8_t cmd, const uint8_t *data, uint16_t length, uint8_t *send_buffer, uint16_t *offset) {
    uint16_t head = *offset;
    send_buffer[(*offset)++] = SOF_1;
    send_buffer[(*offset)++] = SOF_2;
    send_buffer[(*offset)++] = length;
    send_buffer[(*offset)++] = cmd;

    for (uint16_t i = 0; i < length; ++i) {
        send_buffer[(*offset)++] = data[i]; // data
    }
    uint16_t crc16 = CRC16_Check(send_buffer + head, length + 4);
    send_buffer[(*offset)++] = crc16 >> 8;
    send_buffer[(*offset)++] = crc16 & EOF_1;
    send_buffer[(*offset)++] = EOF_1;
}

USBD_StatusTypeDef
sendData_CDC(uint8_t cmd, const uint8_t *data, uint16_t length, uint8_t (*send_func)(uint8_t *Buf, uint16_t Len)) {
    static uint8_t send_buf[CDC_TX_BUF_SIZE];
    uint8_t cnt = 0;

    send_buf[cnt++] = SOF_1;
    send_buf[cnt++] = SOF_2;
    send_buf[cnt++] = length;
    send_buf[cnt++] = cmd;

    for (uint16_t i = 0; i < length; ++i) {
        send_buf[cnt++] = data[i]; // data
    }
    uint16_t crc16 = CRC16_Check(send_buf, length + 4);
    send_buf[cnt++] = crc16 >> 8;
    send_buf[cnt++] = crc16 & EOF_1;
    send_buf[cnt++] = EOF_1;

    return send_func(send_buf, cnt);
}

void analysisData(uint8_t cmd, const uint8_t *data_ptr, uint8_t length) {
    if (cmd == 1) {
        // TODO: 使用单片机相关报错处理
        if ((uint8_t) sizeof(DownloadCMD_1) != length) return;

        AimAssist.miniPC_Online = 1;
        CDC_Receive_Stamp = INS_GetTimeline();

        static DownloadCMD_1 CDCReceiveF_1 = {0};
        memcpy(&CDCReceiveF_1, data_ptr, length);


        AimAssist.Status = CDCReceiveF_1.target_state;
        AimAssist.Target_label = CDCReceiveF_1.target_label;
        AimAssist.YawPosition = CDCReceiveF_1.yaw * 180 / PI;
        AimAssist.PitchPosition = CDCReceiveF_1.pitch * 180 / PI;
        AimAssist.AutoAimDebugShow = CDCReceiveF_1.autoaim_debug_show;
        ShootEvaluation.pitch_allow_range = CDCReceiveF_1.pitch_allow_range;
        ShootEvaluation.yaw_allow_range = CDCReceiveF_1.yaw_allow_range;
        if (Gimbal.Mode == AimAssist_Mode) {
            ShootEvaluation.Allow_Shoot = CDCReceiveF_1.allow_shoot;
            ShootEvaluation.Shoot_Freq = CDCReceiveF_1.shoot_frequency;
        } else {
            ShootEvaluation.Allow_Shoot = 0;
            ShootEvaluation.Shoot_Freq = 0;
        }
        if (CDCReceiveF_1.target_state != 0)
            AimAssist.Trackingcount++;
    }

}

typedef enum {
    RS_SOF_1,
    RS_SOF_2,
    RS_DATA_LENGTH,
    RS_CMD,
    RS_DATA,
    RS_HIGH_EIGHT_CRC16,
    RS_LOW_EIGHT_CRC16,
    RS_EOF_1
} ReceiveState;

int8_t
receiveData_CDC(uint8_t byte_data, void (*analysisData)(uint8_t cmd, const uint8_t *data_ptr, uint8_t length)) {
    static ReceiveState state = RS_SOF_1;
    static uint8_t receive_buf[CDC_RX_BUF_SIZE] = {0};
    static uint8_t cnt = 0;
    static uint8_t length = 0;
    static uint8_t cmd = 0;
    static uint8_t *data_ptr = NULL;
    static uint16_t crc16 = 0;
    // 数据解析-状态机
    switch (state) {
        case RS_SOF_1:
            if (SOF_1 == byte_data) {
                cnt = 0;
                receive_buf[cnt++] = byte_data;
                state = RS_SOF_2;
            }
            break;
        case RS_SOF_2:
            if (SOF_2 == byte_data) {
                receive_buf[cnt++] = byte_data;
                state = RS_DATA_LENGTH;
            } else if (SOF_1 == byte_data) {
                state = RS_SOF_2;
            } else {
                state = RS_SOF_1;
            }
            break;
        case RS_DATA_LENGTH:
            receive_buf[cnt++] = byte_data;
            length = byte_data;
            state = RS_CMD;
            break;
        case RS_CMD:
            receive_buf[cnt++] = byte_data;
            cmd = byte_data;
            data_ptr = &receive_buf[cnt]; // Data First Byte Pointer
            state = RS_DATA;

            if (0 == length)
                state = RS_HIGH_EIGHT_CRC16;
            break;
        case RS_DATA:
            receive_buf[cnt++] = byte_data;
            if (data_ptr + length == &receive_buf[cnt])
                state = RS_HIGH_EIGHT_CRC16;
            break;
        case RS_HIGH_EIGHT_CRC16:
            crc16 = byte_data;
            state = RS_LOW_EIGHT_CRC16;
            break;
        case RS_LOW_EIGHT_CRC16:
            crc16 <<= 8;
            crc16 += byte_data;
            if (crc16 == CRC16_Check(receive_buf, cnt)) {
                state = RS_EOF_1;
            } else if (SOF_1 == byte_data) {
                state = RS_SOF_2;
            } else {
                state = RS_SOF_1;
            }
            break;
        case RS_EOF_1:
            if (EOF_1 == byte_data) {
                analysisData(cmd, data_ptr, length);
                data_ptr = NULL;
                length = 0;
                state = RS_SOF_1;
                return USBD_OK;
            } else if (SOF_1 == byte_data) {
                state = RS_SOF_2;
            } else {
                state = RS_SOF_1;
            }
            break;
        default:
            state = RS_SOF_1;
            break; // Protection
    }
    return USBD_FAIL;
}

uint8_t
CDC_Send2miniPC_Data(float INSTimeCost, const float q[], uint8_t autoaim_mode, uint8_t my_team,uint8_t auxiaim_mode) {
    static uint8_t cdc_send_buf[CDC_TX_BUF_SIZE] = {0}; // CDC 单次发送缓冲区
    uint16_t offset = 0;

    autoaim_mode=AimAssist.auxistatus;
    static_assert(sizeof(UploadCMD_1) == Upload_CMD_1_SIZE, "UploadCMD_1 cannot be modified");
    static UploadCMD_1 CDCSendF_1 = {0};
    CDCSendF_1.INSTimeCost = INSTimeCost * 1000.0f; // Unit: milliseconds
    CDCSendF_1.q_w = q[0];
    CDCSendF_1.q_x = q[1];
    CDCSendF_1.q_y = q[2];
    CDCSendF_1.q_z = q[3];
    CDCSendF_1.autoaim_mode = autoaim_mode;
    CDCSendF_1.auxiaim_mode = autoaim_mode;
    addPacket2Buff(1, (uint8_t *) &CDCSendF_1, sizeof(CDCSendF_1), cdc_send_buf, &offset);

    static uint16_t send_cnt = 0;
    if (++send_cnt % 100 == 0) {
        static_assert(sizeof(UploadCMD_3) <= Upload_CMD_3_MAX_SIZE,
                      "The data frame size must be smaller than 200 bytes.");
        static UploadCMD_3 CDCSendF_3 = {0};
        CDCSendF_3.my_team_color = my_team;

        addPacket2Buff(3, (uint8_t *) &CDCSendF_3, sizeof(CDCSendF_3), cdc_send_buf, &offset);
    }

    return CDC_Transmit_FS(cdc_send_buf, offset) == CDC_OK;
}

uint8_t
CDC_Send2miniPC_CI_Cali_Data(float INSTimeCost,
                             const float q[],
                             uint8_t autoaim_mode,
                             uint8_t my_team,
                             const float accel_data[],
                             const float gyro_data[],
                             const float cov_meas_data[],
                             uint8_t is_enable_send_cov) {
    static uint8_t cdc_send_buf[CDC_TX_BUF_SIZE] = {0}; // CDC 单次发送缓冲区
    uint16_t offset = 0;

    static_assert(sizeof(UploadCMD_2) == Upload_CMD_2_SIZE, "UploadCMD_2 cannot be modified");
    static UploadCMD_2 CDCSendF_2 = {0};
    CDCSendF_2.INSTimeCost = INSTimeCost * 1000.0f; // Unit: milliseconds
    CDCSendF_2.q_w = q[0];
    CDCSendF_2.q_x = q[1];
    CDCSendF_2.q_y = q[2];
    CDCSendF_2.q_z = q[3];
    for (uint8_t i = 0; i < 3; ++i) {
        CDCSendF_2.accel[i] = accel_data[i];
        CDCSendF_2.gyro[i] = gyro_data[i];
    }
    CDCSendF_2.autoaim_mode = autoaim_mode;

    addPacket2Buff(2, (uint8_t *) &CDCSendF_2, sizeof(CDCSendF_2), cdc_send_buf, &offset);

    static uint16_t send_cnt = 0;
    if (++send_cnt % 100 == 0) {
        static_assert(sizeof(UploadCMD_3) <= Upload_CMD_3_MAX_SIZE,
                      "The data frame size must be smaller than 200 bytes.");
        static UploadCMD_3 CDCSendF_3 = {0};
        CDCSendF_3.my_team_color = my_team;

        addPacket2Buff(3, (uint8_t *) &CDCSendF_3, sizeof(CDCSendF_3), cdc_send_buf, &offset);
    }

    return CDC_Transmit_FS(cdc_send_buf, offset) == CDC_OK;
}

uint8_t CDC_Send2miniPC_Shoot_feedback_Data(uint8_t bullet_id, uint8_t is_shoot_success, uint8_t is_reference_recv,
                                            float gimbal_pitch, float bullet_speed, float shoot_delay) {
    static UploadCMD_5 Shoot_feedback = {0};
    Shoot_feedback.bullet_id = bullet_id;
    Shoot_feedback.is_shoot_success = is_shoot_success;
    Shoot_feedback.is_reference_recv = is_reference_recv;
    Shoot_feedback.gimbal_pitch = gimbal_pitch * 0.0174533f;
    Shoot_feedback.bullet_speed = bullet_speed;
    Shoot_feedback.shoot_delay = shoot_delay;

    return sendData_CDC(5, (uint8_t *) &Shoot_feedback, sizeof(Shoot_feedback), CDC_Transmit_FS);
}