/**
 ******************************************************************************
 * @file    shoot task.h
 * @author  Wang Hongxi
 * @version V1.0.0
 * @date    2019/12/31
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _SHOOT_TASK_H
#define _SHOOT_TASK_H

#define SHOOT_TASK_PERIOD 1

#define SPEED_HERO
#define SHOOT_USE_ANGLE_LOOP
#define SHOOT_USE_RAMP
// Fric motor configuration
#define FRIC_MOTOR_USE_RM3508


#ifdef FRIC_MOTOR_USE_RM3508

#include "bsp_CAN.h"
#include "cdc_task.h"

#define FRIC_RM3508_LEFT_ID 0x201 // 一级左前摩擦轮CAN ID
#define FRIC_RM3508_MIDDLE_ID 0x202 // 一级右前摩擦轮CAN ID
#define FRIC_RM3508_RIGHT_ID 0x203 // 一级右前摩擦轮CAN ID


#define FRIC_RM3508_LEFT_LEVELTWO_ID 0x203 // 二级左前摩擦轮CAN ID
#define FRIC_RM3508_RIGHT_LEVELTWO_ID 0x204 // 二级右前摩擦轮CAN ID
#define FRIC_MOTOR_MIN_RPM 1000 // 判断摩擦轮起转的最小转速
#define FRIC_MOTOR_MAX_RPM 6500 // 摩擦轮最大转速
#define FRIC_MOTOR_MAXOUT 16384.0f // 摩擦轮电机最大输出

//#define SHOOT_USE_ANGLE_LOOP 2
#else
#define FRIC_MOTOR_USE_PWM
#include "bsp_PWM.h"
#define FRIC_MOTOR_TIM htim3
#define FRIC_MOTOR_TIM_CHANNEL_1 TIM_CHANNEL_1
#define FRIC_MOTOR_TIM_CHANNEL_2 TIM_CHANNEL_2
#define FRIC_MOTOR_SUSPEND_CCR 1000
#define FRIC_MOTOR_MIN_CCR 1100
#define FRIC_MOTOR_MAX_CCR 2000
#endif

#define TRIGGER_MOTOR_ID 0x202//0x16 // 拨弹电机CAN ID
#define TRIGGER_MOTOR_REDUCTION_RATIO 1159//65535/(360*4)//1//1157.4//436.75// 拨弹盘减速比
#define TRIGGER_GEAR 2.5185f
#define BULLETS_PER_ROUND 6// 拨弹盘弹丸数

enum {
    NoShooting = 0,
    DebugShoot,
    OneShot,
    CounterShot,
    KeepShooting,
    SpinningShot,
    SHOOTBACK,
    SHOOTNONEEEE,
    SHOOTNONEEEEE,
    SHOOTNONEEEEEE,
    SHOOTNONEEEEEEE,
    SHOOTBLOCK
};

typedef struct _shoot {
    uint8_t ShootMode; // 射击模式
    uint8_t LastShootMode; // 上一次射击模式

    int16_t BulletToShoot;

    uint8_t isLidOpen; // 激光是否开启
    uint8_t isFricOn; // 摩擦轮是否开启
    uint8_t FricBlocked;

    float TriggerSpeed; // 拨弹电机速度
    float trigger_target_angle;
    float trigger_target_angle_ramp;
    float trigger_last_target_angle;
    Motor_t TriggerMotor; // 拨弹电机
    uint8_t trigger_flag;
    float initial_speed;
    uint8_t trigger_finish;
    float heatRemain;
    float BulletSpeedLimit;
    float BulletSpeedCompensation;

    float SpeedInBulletsPerSec;
    uint32_t NumsOfOneShot; // 一次射击的弹丸数
    uint32_t ShootDelayInMs; // 射击间隔时间

    uint32_t ShootFinishTick;
    uint32_t ShootBlockTick;

    uint8_t Shoot_Motor_State; // 拨弹点击状态
    uint8_t is_Trigger_Motor_OK; // 拨弹点击是否准备好
    int16_t Bullet_Remaining_Num;
    uint8_t back_flag;
    uint8_t Shoot_Overheat_Protect;

    uint8_t is_sendcurrent_error;

    //	float Last_Trigger_Output;

    void (*Back)(struct _shoot *shoot);

    void (*Reset)(struct _shoot *shoot);

#ifdef FRIC_MOTOR_USE_RM3508
    TD_t FricTD;
    TD_t TriggerTD;
    uint8_t Speed_Flag;
    float FricSpeed; // 摩擦轮转速
    float RefSecFricSpeed; // 期望二级摩擦轮转速
    float RefFirstFricSpeed; // 期望一级摩擦轮转速
    float FricSpeed_dot;
    float FricAccel; // 摩擦轮加速度
    Motor_t FricMotor[3]; // 4个摩擦轮电机
    // FirstOrderSI_t FricMotorSI[2];
#else
    float RefFricSpeed;
    float Fricdt[2];
    float FricSpeed[2];
    float FricSpeedRPM[2];
    int32_t BulletSpeedCompensation_CCR;
    uint32_t FricDWT_Count[2];
    uint32_t Fric_EXTI_Tick[2];
    PID_t FricPID[2];

    TIM_HandleTypeDef PWM_TIM;
    uint32_t FricPWM;
    uint8_t PWM_CHANNEL_1;
    uint8_t PWM_CHANNEL_2;
#endif
} Shoot_t;

extern Shoot_t Shoot;

extern uint8_t Sixfric_Data_Buffer[8];

extern uint32_t Shoot_delay_time;
extern uint8_t bullet_id;
extern uint8_t is_shoot_success;//摩擦轮掉速判断打蛋
extern uint8_t is_reference_recv;//裁判系统更新判断打蛋
extern uint16_t fast_fric_speed_u16;
extern int16_t FirstFricAccel;

void Shoot_Init(void);

void Shoot_Control(void);

float ShootAndDelay(Motor_t *trigger, float speedInNumsPerSec, uint32_t NumsOfOneShot, uint32_t delayTimeInMs);

float RotateAngleInDegree(Motor_t *trigger, float speedRPM, float angleInDegree);

void Shoot_SetTriggerAngle(Shoot_t *shoot, float angle);

void Shoot_Back(Shoot_t *shoot);

void Shoot_Reset(Shoot_t *shoot);

#endif
