/**
 ******************************************************************************
 * @file    gimbal_task.h
 * @author  Wang Hongxi
 * @version V1.0.0
 * @date    2020/08/05
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _GIMBAL_TASK_H
#define _GIMBAL_TASK_H

#include "motor.h"
#include "bsp_CAN.h"
#include "controller.h"
#include "system_identification.h"

#define GIMBAL_TASK_PERIOD 1

// 云台电机Master ID
#define YAW_MOTOR_ID 0x21
#define PITCH_MOTOR_ID 0x201

//图传2006电机ID
#define VISION_MOTOR_ID 0x205

// 云台电机手动期望机械零位校准
#define YAW_MOTOR_ZERO_OFFSET 25306//23073
#define PITCH_MOTOR_ZERO_OFFSET 9960
#define VISION_PITCH_ZERO_OFFSET 4200

// 云台电机最大电流输出(数值制)
#define YAW_MOTOR_MAXOUT 18.5 // 2000
#define PITCH_MOTOR_MAXOUT 16384
#define VISION_MOTOR_MAXOUT 10000

// pitch轴俯仰角电控限位，此限位应当在机械限位内部
#define GIMBAL_MAX_DEPRESSION 3.6
#define GIMBAL_MAX_ELEVATION 44
#define PITCH_MAX_MECH_TOTALANGLE 265000
#define PITCH_MIN_MECH_TOTALANGLE 2000//2000

#define PITCH_CONVERSION_FACTOR 7415
//pitch 限位保护：当Yaw > 110 || Yaw < -80 Pitch 最大仰角改为20
#define PITCH_PROTECT_MIN 97
#define PITCH_PROTECT_MAX -97

// 遥控器及鼠标控制倍率
#define RC_STICK_YAW_RATIO 0.15//0.7
#define RC_STICK_PITCH_RATIO 0.1//0.3 // 0.4
#define RC_MOUSE_YAW_RATIO 5
#define RC_MOUSE_PITCH_RATIO 1.7
#define RC_MOUSE_VISIONPITCH_RATIO -1

//yaw轴各环PID参数
#define YAW_T_PID_MAXOUT 2000
#define YAW_T_PID_MAXINTEGRAL 2000
#define YAW_T_PID_KP 0
#define YAW_T_PID_KI 500
#define YAW_T_PID_KD 0

#define YAW_V_PID_MAXOUT 20
#define YAW_V_PID_MAXINTEGRAL 8
#define YAW_V_PID_KP 10.0f//10.0f
#define YAW_V_PID_KI 12.0f//0
#define YAW_V_PID_KD 0
#define YAW_V_PID_KP_AIMASSIST 10.0f  //1450
#define YAW_V_PID_KI_AIMASSIST 12.0f//450
#define YAW_V_PID_KD_AIMASSIST 0
#define YAW_V_PID_KP_PRECISE 9.5f
#define YAW_V_PID_KI_PRECISE 18.0f
#define YAW_V_PID_KD_PRECISE 0
#define YAW_V_PID_LPF 0 //0.000001
#define YAW_V_PID_D_LPF 0 //0.000001

#define YAW_A_PID_MAXOUT 20//320 / 60 * 2 * 3.14159 / 2
#define YAW_A_PID_MAXINTEGRAL 5
#define YAW_A_PID_KP 0.25f//0.5f
#define YAW_A_PID_KI 0
#define YAW_A_PID_KD 0.003f//0.05f
#define YAW_A_PID_KP_AIMASSIST 0.5f //0.50f
#define YAW_A_PID_KI_AIMASSIST 0
#define YAW_A_PID_KD_AIMASSIST 0.001f //0.007f
#define YAW_POSITION_LPF 0.001f
#define YAW_A_PID_LPF 0.001 //0.001
#define YAW_A_PID_D_LPF 0.001 //0.001
#define YAW_A_PID_KP_PRECISE 0.25f
#define YAW_A_PID_KI_PRECISE 0
#define YAW_A_PID_KD_PRECISE 0.002f

#define YAW_T_FFC_MAXOUT 2000
//#define YAW_T_FCC_C0 0.6
#define YAW_T_FCC_C0 0 //1
#define YAW_T_FCC_C1 0
#define YAW_T_FCC_C2 0
#define YAW_T_FCC_LPF 0

#define YAW_V_FFC_MAXOUT 2000
#define YAW_V_FCC_C0 0 //-2.5f
#define YAW_V_FCC_C1 0
#define YAW_V_FCC_C2 0
#define YAW_V_FCC_LPF 0 //0.001

#define YAW_A_FFC_MAXOUT 320 / 60 * 2 * 3.14159
#define YAW_A_FCC_C0 0//0.001
#define YAW_A_FCC_C1 0.003//0.005 //0.005
#define YAW_A_FCC_C2 0//0
#define YAW_A_FCC_LPF 0 //0.001

//丝杆参数
#define FIRST_ADJACENT_SIDE 107.91
#define SECOND_ADJACENT_SIDE 126.484
#define OPPSITE_SIDE 143.88
#define ANGLE_CORRECTION 79.07

//pitch轴各环PID参数(2024第一版车不用电流环)
#define PITCH_T_PID_MAXOUT 4000//2000
#define PITCH_T_PID_MAXINTEGRAL 2000
#define PITCH_T_PID_KP 0
#define PITCH_T_PID_KI 500
#define PITCH_T_PID_KD 0

#define PITCH_V_PID_MAXOUT 16384
#define PITCH_V_PID_MAXINTEGRAL 10000
#define PITCH_V_PID_KP -40//-50//-2200
#define PITCH_V_PID_KI -50
#define PITCH_V_PID_KD 0
#define PITCH_V_PID_KP_AIMASSIST -50.0f
#define PITCH_V_PID_KI_AIMASSIST 0
#define PITCH_V_PID_KD_AIMASSIST 0
#define PITCH_V_PID_KP_PRECISE -50.0f
#define PITCH_V_PID_KI_PRECISE 0
#define PITCH_V_PID_KD_PRECISE 0
#define PITCH_V_PID_LPF 0.0001f //0.0001f
#define PITCH_V_PID_D_LPF 0 //0.0005f

#define PITCH_A_PID_MAXOUT 16384//320 / 60 * 2 * 3.14159
#define PITCH_A_PID_MAXINTEGRAL 10000
#define PITCH_A_PID_KP 3500
#define PITCH_A_PID_KI 2500
#define PITCH_A_PID_KD 0
#define PITCH_A_PID_LPF 0.001
#define PITCH_A_PID_KP_AIMASSIST 0.15f//0.8f //1.2f
#define PITCH_A_PID_KI_AIMASSIST 0.0f
#define PITCH_A_PID_KD_AIMASSIST 0.0f //0.008f
#define PITCH_POSITON_LPF 0.001
#define PITCH_A_PID_D_LPF 0 //0.001
#define PITCH_A_PID_KP_PRECISE 0.15f
#define PITCH_A_PID_KI_PRECISE 0
#define PITCH_A_PID_KD_PRECISE 0

#define PITCH_MA_PID_MAXOUT 8600//320 / 60 * 2 * 3.14159
#define PITCH_MA_PID_MAXINTEGRAL 3000
#define PITCH_MA_PID_KP 0.15f
#define PITCH_MA_PID_KI 0
#define PITCH_MA_PID_KD 0
#define PITCH_MA_PID_LPF 0.001
#define PITCH_MA_PID_KP_AIMASSIST 0.15f//0.8f //1.2f
#define PITCH_MA_PID_KI_AIMASSIST 0.0f
#define PITCH_MA_PID_KD_AIMASSIST 0.0f //0.008f
#define PITCH_MA_PID_D_LPF 0 //0.001
#define PITCH_MA_PID_KP_PRECISE 0.15f
#define PITCH_MA_PID_KI_PRECISE 0
#define PITCH_MA_PID_KD_PRECISE 0

#define PITCH_T_FFC_MAXOUT 2000
#define PITCH_T_FCC_C0 0 //1
#define PITCH_T_FCC_C1 0
#define PITCH_T_FCC_C2 0
#define PITCH_T_FCC_LPF 0

#define PITCH_V_FFC_MAXOUT 2000
#define PITCH_V_FCC_C0 -1 //-12
#define PITCH_V_FCC_C1 -1 //-5
#define PITCH_V_FCC_C2 -1 //
#define PITCH_V_FCC_LPF 0.001 //0.001

#define PITCH_A_FFC_MAXOUT 2000
#define PITCH_A_FCC_C0 0
#define PITCH_A_FCC_C1 0.006
#define PITCH_A_FCC_C2 0
#define PITCH_A_FCC_LPF 0.0003 //0.003f

#define VISION_V_PID_MAXOUT 10000
#define VISION_V_PID_MAXINTEGRAL 2000
#define VISION_V_PID_KP 6
#define VISION_V_PID_KI 0
#define VISION_V_PID_KD 0

#define VISION_A_PID_MAXOUT 2000 //320 / 60 * 2 * 3.14159
#define VISION_A_PID_MAXINTEGRAL 1800
#define VISION_A_PID_KP 0.25
#define VISION_A_PID_KI 0
#define VISION_A_PID_KD 0


//yaw轴机械同步带减速比
#define YAW_REDUCTION_RATIO 1 / 1.0f

#define YAW_REDUCTION_CORRECTION_ANGLE 180

enum {
    Normal_Mode = 0X00,     // 0000 0000
    AimAssist_Mode = 0x01,  // 0000 0001
    Precise_Mode = 0x03,    // 0000 0011
    Gimbal_Reserve2 = 0x02, // 0000 0010
    Gimbal_Reserve3 = 0x04, // 0000 0100
    Gimbal_Reserve4 = 0x08, // 0000 1000
    Gimbal_Reserve5 = 0x10, // 0001 0000
    Gimbal_Reserve6 = 0x20, // 0010 0000
    Gimbal_Reserve7 = 0x40, // 0100 0000
    Gimbal_Reserve8 = 0x80, // 1000 0000
    Auxiliary_Mode = 0x100,
};

typedef struct {
    FirstOrderSI_t YawSI;
    FirstOrderSI_t PitchSI;

    uint8_t ResetFlag;

    float Q0;
    float Q1;
    float Q2;
    float R;
    float lambda;
} GimbalSI_t;

typedef struct _GimbalControl {
    DMMotor_t YawMotor; // 云台yaw电机
    Motor_t PitchMotor; // 云台pitch电机
    Motor_t VisionMotor; // 图传pitch2006电机

    float YawAngle; // 云台当前yaw角度，来源于姿态解算
    float PitchAngle; // 云台当前pitch角度，来源于姿态解算
    float PitchMechAngle; //pitch机械角度
    float PitchAngleOffset;//丝杆pitch零点
    float PitchZeroOffset;
    float VisionAngle; //2006倍镜总角度
    float VisionAngleOffset; //2006倍镜零点
    float EncoderYawAngle; // 云台当前yaw角度，来源于电机数据
    float EncoderPitchAngle; // 云台当前pitch角度，来源于电机数据
    float YawAngularVelocity; // 云台当前yaw角速度
    float PitchAngularVelocity; // 云台当前pitch角速度
    float RampAngle;//由姿态结算角和编码器角之差算得的坡度角

    TD_t YawRefAngularVelocityTD;
    TD_t PitchRefAngularVelocityTD;
    TD_t YawRefAngleTD;
    TD_t PitchRefAngleTD;
    TD_t PitchRefMechAngleTD;
    TD_t VisionRefAngleTD;

    float ConversionFactor;
    float YawRefAngularVelocity; // 云台yaw期望角速度
    float PitchRefAngularVelocity; // 云台pitch期望角速度
    float PitchRefMechAngularVelocity;
    float PitchRefCorrectionAngularVelocity;
    float VisionRefAngularVelocity; // 2006倍镜期望角速度
    float YawCtrlAngle; // 云台yaw控制期望角度
    float PitchCtrlAngle; // 云台pitch控制期望角度
    float PitchCtrlMechAngle; //pitch控制期望角度
    float VisionCtrlAngle; // 2006倍镜控制期望角度
    float YawRefAngle; // 云台yaw期望角度
    float PitchRefAngle; // 云台pitch期望角度
    float PitchRefMechAngle; //pitch期望机械角度
    float VisionRefAngle; // 2006倍镜期望角度
    float PitchVisionAngle; // 小云台pitch舵机角度
    float YawError;
    float PitchError;

    float rcStickYawRatio; // 遥控器yaw轴控制倍率
    float rcStickPitchRatio; // 遥控器pitch轴控制倍率
    float rcMouseYawRatio; // 鼠标yaw轴控制倍率
    float rcMousePitchRatio; // 鼠标pitch轴控制倍率

    float ChassisOmega[3];

    uint16_t ModeSwitchCount; //云台同模式持续时间

    uint8_t LaserState;
    uint8_t Mode; // 云台模式
    uint8_t ModeLast; // 上一时间周期云台模式

    float DepressionEncoderInDegree; // 云台pitch电控俯角最大限位
    float ElevationEncoderInDegree;  // 云台pitch电控仰角最大限位
    float DepressionIMU;             // 云台pitch电控俯角最大限位
    float ElevationIMU;              // 云台pitch电控仰角最大限位
    float DepressionMechAngle;
    float ElevationMechAngle;

    uint8_t isGaming;
    uint8_t SetPreciseMode;
    uint8_t zoom_state;
    uint8_t vision_flag;
} Gimbal_t;

enum {
    LaserOff = 0,
    LaserOn = 1,
};

enum {
    VelocityMode = 0,
    AngleMode = 1,
};

extern Gimbal_t Gimbal;
extern uint8_t Gimbal_Data_Buffer[8];

void Gimbal_Init(void);

void Gimbal_Control(void);

void PitchMotor_Tuning(void);

void YawMotor_Tuning(void);

#endif
