#ifndef __CDC_TASK_H
#define __CDC_TASK_H

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "protocol.h"
#include <assert.h>

#define CDC_OK 1
#define CDC_FAIL 0

enum CDC_SendMode {
    CDC_NORMAL_IMU,
    CDC_CAMERA_IMU_CALIBRATION, /*camera-imu*/
};

extern uint8_t cdc_send_mode;

void CDC_Init(void);

void CDC_Task(void);

uint16_t
CRC16_Check(const uint8_t *data, uint8_t len);

USBD_StatusTypeDef
sendData_CDC(uint8_t cmd, const uint8_t *data, uint16_t length, uint8_t (*send_func)(uint8_t *Buf, uint16_t Len));

void analysisData(uint8_t cmd, const uint8_t *data_ptr, uint8_t length);

int8_t
receiveData_CDC(uint8_t byte_data, void (*analysisData)(uint8_t cmd, const uint8_t *data_ptr, uint8_t length));

uint8_t
CDC_Send2miniPC_Data(float INSTimeCost, const float q[], uint8_t autoaim_mode, uint8_t my_team,uint8_t auxiaim_mode);

uint8_t
CDC_Send2miniPC_CI_Cali_Data(float INSTimeCost,
                             const float q[],
                             uint8_t autoaim_mode,
                             uint8_t my_team,
                             const float accel_data[],
                             const float gyro_data[],
                             const float cov_meas_data[],
                             const uint8_t is_enable_send_cov
                            );

uint8_t
CDC_Send2miniPC_Shoot_Data(float bullet_speed);

uint8_t CDC_Send2miniPC_Shoot_feedback_Data(uint8_t bullet_id, uint8_t is_shoot_success, uint8_t is_reference_recv,
                                            float gimbal_pitch,
                                            float bullet_speed, float shoot_delay);

#endif