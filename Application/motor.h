/**
 ******************************************************************************
 * @file    Motor.h
 * @author  Hongxi Wong
 * @version V1.2.1
 * @date    2021/4/13
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef _MOTOR_H
#define _MOTOR_H

#include <stdint.h>
#include "controller.h"
#include "dm4310_drv.h"
#include "can.h"

#define ENCODERCOEF 0.0439453125f // 对于3508或6020电机的机械角度数值制(0~8191)转角度制(0°~360°)的倍率
#define RANDTOANGLE 57.295779513f //弧度转角度参数
#define DM_4PI_ENCODER 65881.135309559670469065210522618

// CAN Transmit ID
#define CAN_Transmit_1_4_ID 0x200 // C620电调ID1~4的指令ID
#define CAN_Transmit_5_8_ID 0x1ff // C620电调ID5~8的指令ID

#define CAN_Transmit_Yaw_ID 0x20//0x42 //yaw电机的MasterID
#define CAN_Transmit_Trigger_ID 0x06 //Trigger电机的MasterID

#define CAN_Transmit_DM4310_ID 0x11
// CAN Receive ID
#define CAN_Receive_1_ID 0x201
#define CAN_Receive_2_ID 0x202
#define CAN_Receive_3_ID 0x203
#define CAN_Receive_4_ID 0x204
#define CAN_Receive_5_ID 0x205
#define CAN_Receive_6_ID 0x206
#define CAN_Receive_7_ID 0x207
#define CAN_Receive_8_ID 0x208

// CAN Gimbal ID
#define CAN_GIMBAL_Info_ID 0x666
#define CAN_GIMBAL_Control_ID_1 0x233
#define CAN_GIMBAL_Control_ID_2 0x404

#define NEGATIVE 1

/*moto information receive from CAN*/
typedef __packed struct motor_t {
    int16_t Velocity_RPM; // 电机转速(单位RPM)
    float Real_Current; // 电机实际电流
    uint8_t Temperature; // 电机实际温度

    uint8_t Direction; // 电机正方向

    float Ke;

    float Angle; // 电机经过零位校准后的角度(数值制)
    float AngleInDegree; // 电机经过零位校准后的角度(度数制)
    uint16_t RawAngle; // 电机机械角度值
    uint16_t last_angle; // 上一时间周期的电机机械角度值

    int16_t offset_angle; // 自动校准的零位电机机械角度值
    int32_t round_cnt; // 圈数
    float total_angle; // 电机总机械角度值

    int16_t zero_offset; // 手动校准的零位电机机械角度值

    int32_t msg_cnt; // 时间计数

    uint16_t CAN_ID; // 电机的CAN ID

    float Output; // 电机电流输出(数值制)
    float Max_Out; // 电机最大输出

    PID_t PID_Torque; // 力矩环PID
    PID_t PID_Velocity; // 速度环PID
    PID_t PID_Angle; // 角度环PID
    PID_t PID_MechAngle;

    Feedforward_t FFC_Torque;
    Feedforward_t FFC_Velocity;
    Feedforward_t FFC_Angle;

    LDOB_t LDOB;

    Lpf_t Lpf_angle;
    Lpf_t Lpf_velocity;

    void (*TorqueCtrl_User_Func_f)(struct motor_t *motor);

    void (*SpeedCtrl_User_Func_f)(struct motor_t *motor);

    void (*AngleCtrl_User_Func_f)(struct motor_t *motor);

} Motor_t;


float Motor_Torque_Calculate(Motor_t *motor, float torque, float target_torque);

float Motor_Speed_Calculate(Motor_t *motor, float velocity, float target_speed);

float Motor_Angle_Calculate(Motor_t *motor, float angle, float velocity, float target_angle);

void get_moto_info(Motor_t *ptr, uint8_t *aData);

void get_moto_offset(Motor_t *ptr, uint8_t *aData);

void get_moto_offset_DM(DMMotor_t *ptr, uint8_t *aData);

void get_moto_info_DM4310(DMMotor_t *ptr, uint8_t *aData);

HAL_StatusTypeDef Send_Motor_Current_1_4(CAN_HandleTypeDef *_hcan, int16_t c1, int16_t c2, int16_t c3, int16_t c4);

HAL_StatusTypeDef Send_Motor_Current_5_8(CAN_HandleTypeDef *_hcan, int16_t c1, int16_t c2, int16_t c3, int16_t c4);

HAL_StatusTypeDef Send_Motor_Current_DMYaw(CAN_HandleTypeDef *_hcan,
                                           float _pos,
                                           float _vel,
                                           float _KP,
                                           float _KD,
                                           float _torq);

HAL_StatusTypeDef Send_Motor_Current_DM4310(CAN_HandleTypeDef *_hcan,
                                            float _pos,
                                            float _vel,
                                            float _KP,
                                            float _KD,
                                            float _torq);

HAL_StatusTypeDef Send_Motor_Current_DMTrigger(CAN_HandleTypeDef *_hcan,
                                               float _pos,
                                               float _vel,
                                               float _KP,
                                               float _KD,
                                               float _torq);

void SetMotorRef(Motor_t *motor, float angle);

#endif
