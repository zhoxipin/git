/**
 ******************************************************************************
 * @file    Motor.c
 * @author  Hongxi Wong
 * @version V1.2.2
 * @date    2021/4/13
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "motor.h"
#include "dm4310_drv.h"

float Motor_Torque_Calculate(Motor_t *motor, float torque, float target_torque) {
    // 前馈控制
    Feedforward_Calculate(&motor->FFC_Torque, target_torque);
    // 反馈控制
    PID_Calculate(&motor->PID_Torque, torque, target_torque);

    if (motor->TorqueCtrl_User_Func_f != NULL)
        motor->TorqueCtrl_User_Func_f(motor);

    if (motor->Direction != NEGATIVE)
        motor->Output = motor->FFC_Torque.Output + motor->PID_Torque.Output + motor->Ke * motor->Velocity_RPM;
    else
        motor->Output = motor->FFC_Torque.Output + motor->PID_Torque.Output - motor->Ke * motor->Velocity_RPM;
    // 输出限幅
    motor->Output = float_constrain(motor->Output, -motor->Max_Out, motor->Max_Out);

    return motor->Output;
}

float Motor_Speed_Calculate(Motor_t *motor, float velocity, float target_speed) {
    // 前馈控制
    Feedforward_Calculate(&motor->FFC_Velocity, target_speed);
    // 反馈控制
    PID_Calculate(&motor->PID_Velocity, velocity, target_speed);
    // 线性扰动观测器
    LDOB_Calculate(&motor->LDOB, velocity, motor->Output);

    if (motor->SpeedCtrl_User_Func_f != NULL)
        motor->SpeedCtrl_User_Func_f(motor);

    // 扰动补偿
    motor->Output = motor->FFC_Velocity.Output + motor->PID_Velocity.Output - motor->LDOB.Disturbance;
    // 输出限幅
    motor->Output = float_constrain(motor->Output, -motor->Max_Out, motor->Max_Out);

    return motor->Output;
}

float Motor_Angle_Calculate(Motor_t *motor, float angle, float velocity, float target_angle) {
    // 外环前馈控制
    Feedforward_Calculate(&motor->FFC_Angle, target_angle);
    // 外环反馈控制
    PID_Calculate(&motor->PID_Angle, angle, target_angle);

    if (motor->AngleCtrl_User_Func_f != NULL)
        motor->AngleCtrl_User_Func_f(motor);

    // 内环
    Motor_Speed_Calculate(motor, velocity, motor->FFC_Angle.Output + motor->PID_Angle.Output);

    return motor->Output;
}

/**
 * @Func	    void get_moto_info(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
 * @Brief      process data received from CAN
 * @Param	    Motor_t *ptr  CAN_HandleTypeDef *_hcan
 * @Retval	    None
 * @Date       2019/11/5
 **/
void get_moto_info(Motor_t *ptr, uint8_t *aData) {
    // 详见C620电调手册
    // 获取电机机械角度与转速(单位RPM)
    if (ptr->Direction != NEGATIVE) {
        ptr->RawAngle = (uint16_t) (aData[0] << 8 | aData[1]);
        ptr->Velocity_RPM = (int16_t) (aData[2] << 8 | aData[3]);
    } else {
        ptr->RawAngle = 8191 - (uint16_t) (aData[0] << 8 | aData[1]);
        ptr->Velocity_RPM = -(int16_t) (aData[2] << 8 | aData[3]);
    }
    // 获取电机实际电流与温度
    ptr->Real_Current = (int16_t) (aData[4] << 8 | aData[5]);
    ptr->Temperature = aData[6];
    ptr->Angle = loop_float_constrain(ptr->RawAngle - ptr->zero_offset, -32768, 32768);
    // 将数值制电机角度转变为度数制电机角度
    if (ptr->RawAngle - ptr->last_angle > 4096)
        ptr->round_cnt--;
    else if (ptr->RawAngle - ptr->last_angle < -4096)
        ptr->round_cnt++;
    // 获取电机经过期望零位校准后的角度值(数值制)
    ptr->Angle = loop_float_constrain(ptr->RawAngle - ptr->zero_offset, -4095, 4096);
    // 将数值制电机角度转变为度数制电机角度
    ptr->AngleInDegree = ptr->Angle * 0.0439507f;
    // 获取电机当前总角度值
    ptr->total_angle = ptr->round_cnt * 8192 + ptr->RawAngle - ptr->offset_angle;
    // 为新时间周期做准备，更新机械角度
    ptr->last_angle = ptr->RawAngle; // update last_angle
}

void get_moto_info_9015(Motor_t *ptr, uint8_t *aData) {
    if((!((aData[1]==0)&&(aData[2]==0)&&(aData[3]==0)&&(aData[6]==0)&&(aData[7]==0)))&&(aData[0]==0xA1)) {
        // 详见上海瓴控科技有限公司9025CAN通讯手册(9015同9025)
        // 获取电机机械角度与转速(单位RPM)
        // 注：实测读取的转速数据与手册上写的不符，故乘以一特定倍率0.159154943，由实测得到
        if (ptr->Direction != NEGATIVE) {
            ptr->RawAngle = (uint16_t) (aData[7] << 8 | aData[6]);
            ptr->Velocity_RPM = (int16_t) ((aData[5] << 8) | aData[4]) * 0.159154943f;
        } else {
            ptr->RawAngle = 65535 - (uint16_t) (aData[7] << 8 | aData[6]);
            ptr->Velocity_RPM = -(int16_t) ((aData[5] << 8) | aData[4]) * 0.159154943f;
        }
        // 获取电机实际电流与温度
        ptr->Real_Current = (int16_t) (aData[3] << 8 | aData[2]);
        ptr->Temperature = aData[1];
        // 根据与上一时间周期的机械角度差值判断圈数
        if (ptr->RawAngle - ptr->last_angle > 32767)
            ptr->round_cnt--;
        else if (ptr->RawAngle - ptr->last_angle < -32767)
            ptr->round_cnt++;
        // 获取电机经过期望零位校准后的角度值(数值制)
        ptr->Angle = loop_float_constrain(ptr->RawAngle - ptr->zero_offset, -32768, 32768);
        // 将数值制电机角度转变为度数制电机角度
        ptr->AngleInDegree = ptr->Angle * 0.005493164f;
        // 获取电机当前总角度值
        ptr->total_angle = ptr->round_cnt * 65536 + ptr->RawAngle - ptr->offset_angle;
        // 为新时间周期做准备，更新机械角度
        ptr->last_angle = ptr->RawAngle;
    }
}

/*this function should be called after system+can init */
void get_moto_offset(Motor_t *ptr, uint8_t *aData) {
    // 将当前机械角度值作为期望调零值
    ptr->RawAngle = (uint16_t) (aData[0] << 8 | aData[1]);
    ptr->offset_angle = ptr->RawAngle;
    ptr->last_angle = ptr->RawAngle;
}

void get_moto_offset_DM(DMMotor_t *ptr, uint8_t *aData) {
    // 将当前机械角度值作为期望调零值
    if (ptr->Direction != NEGATIVE)
    {
        ptr->para.p_int = (aData[1]<<8)|aData[2];  //电机位置获取(角度)
        ptr->para.v_int = (aData[3]<<4)|(aData[4]>>4);  //电机速度获取
        ptr->para.t_int = ((aData[4]&0xF)<<8)|aData[5];  //电机扭矩获取
    }
    else
    {
        ptr->para.p_int = 65535-(aData[1]<<8)|aData[2];  //电机位置获取(角度)
        ptr->para.v_int = -(aData[3]<<4)|(aData[4]>>4);  //电机速度获取
        ptr->para.t_int = -((aData[4]&0xF)<<8)|aData[5];  //电机扭矩获取
    }
    ptr->offset_angle = ptr->para.p_int;
    ptr->para.last_p_int = ptr->para.p_int;
}

void get_moto_info_DM4310(DMMotor_t *ptr, uint8_t *aData) {
    ptr->para.id = (aData[0]) & 0x0F;  //电机ID
    ptr->para.state = (aData[0]) >> 4;  //电机状态

    if (ptr->Direction != NEGATIVE)
    {
        ptr->para.p_int = ((aData[1]<<8)|aData[2]);  //电机位置获取(角度)
        ptr->para.v_int = (aData[3]<<4)|(aData[4]>>4);  //电机速度获取
        ptr->para.t_int = ((aData[4]&0xF)<<8)|aData[5];  //电机扭矩获取
    }
    else
    {
        ptr->para.p_int = 65535-((aData[1]<<8)|aData[2]);  //电机位置获取(角度)
        ptr->para.v_int = -((aData[3]<<4)|(aData[4]>>4));  //电机速度获取
        ptr->para.t_int = -(((aData[4]&0xF)<<8)|aData[5]);  //电机扭矩获取
    }
    ptr->para.pos = uint_to_float(ptr->para.p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
    ptr->para.vel = uint_to_float(ptr->para.v_int, V_MIN, V_MAX, 12); // (-45.0,45.0)
    ptr->para.tor = uint_to_float(ptr->para.t_int, T_MIN, T_MAX, 12);  // (-18.0,18.0)
    ptr->para.Tmos = (float) (aData[6]);  //驱动MOS管温度
    ptr->para.Tcoil = (float) (aData[7]);  //电机内部线圈温度

    ptr->Angle = loop_float_constrain(ptr->para.p_int - ptr->offset_angle, -32768, 32768) *4*PI/32768;
//    ptr->AngleInDegree = ptr->Angle * (180 / PI);  //电机经过零位校准后的角度(度数制)

    if (ptr->para.p_int - ptr->para.last_p_int > 32768)
        ptr->EncoderRoundCnt--;
    if (ptr->para.p_int - ptr->para.last_p_int < -32768)
        ptr->EncoderRoundCnt++;
    ptr->EncoderTotalAngle =  65536 * ptr->EncoderRoundCnt + ptr->para.p_int - ptr->offset_angle;
    ptr->AngleInDegree = ptr->EncoderTotalAngle * 4.0 / 32768.0 * 180.0;
    ptr->para.last_p_int = ptr->para.p_int;
}


/**
  * @Func	    void Send_Motor_Current(CAN_HandleTypeDef* hcan,
                                    int16_t c1, int16_t c2, int16_t c3, int16_t c4)
  * @Brief
  * @Param	    cx x=1,2,3,4
  * @Retval	    None
  * @Date       2019/11/5
 **/
HAL_StatusTypeDef Send_Motor_Current_1_4(CAN_HandleTypeDef *_hcan,
                                         int16_t c1, int16_t c2, int16_t c3, int16_t c4) {
    static CAN_TxHeaderTypeDef TX_MSG;
    static uint8_t CAN_Send_Data[8];
    uint32_t send_mail_box;

    // 设置发送数据包的ID和其他属性 除ID为特定内容外均为默认（通用）情况
    TX_MSG.StdId = CAN_Transmit_1_4_ID;
    TX_MSG.IDE = CAN_ID_STD;
    TX_MSG.RTR = CAN_RTR_DATA;
    TX_MSG.DLC = 0x08;

    // 根据C620电调协议设置电机电流值
    CAN_Send_Data[0] = (c1 >> 8);
    CAN_Send_Data[1] = c1;
    CAN_Send_Data[2] = (c2 >> 8);
    CAN_Send_Data[3] = c2;
    CAN_Send_Data[4] = (c3 >> 8);
    CAN_Send_Data[5] = c3;
    CAN_Send_Data[6] = (c4 >> 8);
    CAN_Send_Data[7] = c4;

    while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING))) {
    }
    while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) //如果三个邮箱都阻塞了就等一会儿，直到其中某个邮箱空闲
    {
        // HAL_CAN_MspDeInit(&hcan1);
        // HAL_CAN_MspDeInit(&hcan2);
        // MX_CAN1_Init();
        // MX_CAN2_Init();
        // CAN_Device_Init();
    }
    /* Check Tx Mailbox 1 status */
    if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U) {
        send_mail_box = CAN_TX_MAILBOX0;
    }
        /* Check Tx Mailbox 1 status */
    else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U) {
        send_mail_box = CAN_TX_MAILBOX1;
    }

        /* Check Tx Mailbox 2 status */
    else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U) {
        send_mail_box = CAN_TX_MAILBOX2;
    }
    return HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

HAL_StatusTypeDef Send_Motor_Current_5_8(CAN_HandleTypeDef *_hcan,
                                         int16_t c1, int16_t c2, int16_t c3, int16_t c4) {
    static CAN_TxHeaderTypeDef TX_MSG;
    static uint8_t CAN_Send_Data[8];
    uint32_t send_mail_box;

    TX_MSG.StdId = CAN_Transmit_5_8_ID;
    TX_MSG.IDE = CAN_ID_STD;
    TX_MSG.RTR = CAN_RTR_DATA;
    TX_MSG.DLC = 0x08;
    CAN_Send_Data[0] = (c1 >> 8);
    CAN_Send_Data[1] = c1;
    CAN_Send_Data[2] = (c2 >> 8);
    CAN_Send_Data[3] = c2;
    CAN_Send_Data[4] = (c3 >> 8);
    CAN_Send_Data[5] = c3;
    CAN_Send_Data[6] = (c4 >> 8);
    CAN_Send_Data[7] = c4;

    while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING))) {
    }
    while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) //如果三个邮箱都阻塞了就等一会儿，直到其中某个邮箱空闲
    {
        // HAL_CAN_MspDeInit(&hcan1);
        // HAL_CAN_MspDeInit(&hcan2);
        // MX_CAN1_Init();
        // MX_CAN2_Init();
        // CAN_Device_Init();
    }
    /* Check Tx Mailbox 1 status */
    if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U) {
        send_mail_box = CAN_TX_MAILBOX0;
    }
        /* Check Tx Mailbox 1 status */
    else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U) {
        send_mail_box = CAN_TX_MAILBOX1;
    }

        /* Check Tx Mailbox 2 status */
    else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U) {
        send_mail_box = CAN_TX_MAILBOX2;
    }
    return HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

HAL_StatusTypeDef Send_Motor_Current_DMYaw(CAN_HandleTypeDef *_hcan,
                                            float _pos,
                                            float _vel,
                                            float _KP,
                                            float _KD,
                                            float _torq) {
    static CAN_TxHeaderTypeDef TX_MSG;
    static uint8_t CAN_Send_Data[8];
    uint32_t send_mail_box;

    TX_MSG.StdId = CAN_Transmit_Yaw_ID;
    TX_MSG.IDE = CAN_ID_STD;
    TX_MSG.RTR = CAN_RTR_DATA;
    TX_MSG.DLC = 0x08;

    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
    kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
    kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(_torq, -35, 35, 12);
    //tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

    // 根据DM4310通讯协议设置电机电流值
    //mit_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set, motor->ctrl.kp_set, motor->ctrl.kd_set, motor->ctrl.tor_set)

    CAN_Send_Data[0] = (pos_tmp >> 8);
    CAN_Send_Data[1] = pos_tmp;
    CAN_Send_Data[2] = (vel_tmp >> 4);
    CAN_Send_Data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    CAN_Send_Data[5] = kp_tmp;
    CAN_Send_Data[4] = (kd_tmp >> 4);
    CAN_Send_Data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    CAN_Send_Data[7] = tor_tmp;

    while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING))) {
    }
    while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) //如果三个邮箱都阻塞了就等一会儿，直到其中某个邮箱空闲
    {
        // HAL_CAN_MspDeInit(&hcan1);
        // HAL_CAN_MspDeInit(&hcan2);
        // MX_CAN1_Init();
        // MX_CAN2_Init();
        // CAN_Device_Init();
    }
    /* Check Tx Mailbox 1 status */
    if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U) {
        send_mail_box = CAN_TX_MAILBOX0;
    }
        /* Check Tx Mailbox 1 status */
    else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U) {
        send_mail_box = CAN_TX_MAILBOX1;
    }

        /* Check Tx Mailbox 2 status */
    else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U) {
        send_mail_box = CAN_TX_MAILBOX2;
    }
    return HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

HAL_StatusTypeDef Send_Motor_Current_DMTrigger(CAN_HandleTypeDef *_hcan,
                                             float _pos,
                                             float _vel,
                                             float _KP,
                                             float _KD,
                                             float _torq) {
    static CAN_TxHeaderTypeDef TX_MSG;
    static uint8_t CAN_Send_Data[8];
    uint32_t send_mail_box;

    TX_MSG.StdId = CAN_Transmit_Trigger_ID;
    TX_MSG.IDE = CAN_ID_STD;
    TX_MSG.RTR = CAN_RTR_DATA;
    TX_MSG.DLC = 0x08;

    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
    kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
    kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(_torq, -20, 20, 12);
    //tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

    // 根据DM4310通讯协议设置电机电流值
    //mit_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set, motor->ctrl.kp_set, motor->ctrl.kd_set, motor->ctrl.tor_set)

    CAN_Send_Data[0] = (pos_tmp >> 8);
    CAN_Send_Data[1] = pos_tmp;
    CAN_Send_Data[2] = (vel_tmp >> 4);
    CAN_Send_Data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    CAN_Send_Data[5] = kp_tmp;
    CAN_Send_Data[4] = (kd_tmp >> 4);
    CAN_Send_Data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    CAN_Send_Data[7] = tor_tmp;

    while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING))) {
    }
    while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) //如果三个邮箱都阻塞了就等一会儿，直到其中某个邮箱空闲
    {
        // HAL_CAN_MspDeInit(&hcan1);
        // HAL_CAN_MspDeInit(&hcan2);
        // MX_CAN1_Init();
        // MX_CAN2_Init();
        // CAN_Device_Init();
    }
    /* Check Tx Mailbox 1 status */
    if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U) {
        send_mail_box = CAN_TX_MAILBOX0;
    }
        /* Check Tx Mailbox 1 status */
    else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U) {
        send_mail_box = CAN_TX_MAILBOX1;
    }

        /* Check Tx Mailbox 2 status */
    else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U) {
        send_mail_box = CAN_TX_MAILBOX2;
    }
    return HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}
//HAL_StatusTypeDef Send_DMMotor_Order(CAN_HandleTypeDef *_hcan, DMMotor *motor, DMMotor_Mode_e cmd)
//{
//    static CAN_TxHeaderTypeDef TX_MSG;
//    static uint8_t CAN_Send_Data[8];
//    uint32_t send_mail_box;
//
//    TX_MSG.StdId = CAN_Transmit_DM4310_ID;
//    TX_MSG.IDE = CAN_ID_STD;
//    TX_MSG.RTR = CAN_RTR_DATA;
//    TX_MSG.DLC = 0x08;
//
//    memset(CAN_Send_Data, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
//    CAN_Send_Data[7] = (uint8_t)cmd; // 最后一位是命令id
//
//    CAN_Transmit(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
//}
void SetMotorRef(Motor_t *motor, float angle) {
    motor->PID_Angle.Ref = angle;
}
