/*
******************************************************************************
* @file    shoot task.c
* @author  Wang Hongxi
* @version V1.1.0
* @date    2021/3/17
* @brief
******************************************************************************
* @attention
*
******************************************************************************
*/
#include "shoot_task.h"
#include "aimassist_task.h"
#include "math.h"
#include "usart.h"
#include "motor.h"
#include "controller.h"
#include "judgement_info.h"

Shoot_t Shoot = {0};
RAMP_T Ramp = {0};

uint32_t Shoot_DWT_Count = 0;
static float dt = 0, t = 0;

uint8_t TriggerSpeedNum = 100; // 80
uint8_t ShootDebugMode = 1;
static float amp[6] = {10, 1, 1, 0.1, 1, 1};

float min_fricspeed_dot = -20000;
static float c[3] = {0};
float c_trigger[3] = {0};
float trigger_v_ffc_maxout = 30000;
float trigger_v_ffc_lpf = 0.003;
//设置10和16两种弹速上限下的摩擦轮转速
#if HERO == ONE
uint16_t fast_first_fric_speed = 3000;//4500;//first fric
uint16_t fast_second_fric_speed = 4400;//second fric
uint16_t low_first_fric_speed = 4600;//first fric
uint16_t low_second_fric_speed = 4500;//second fric
float low_fric_speed = 6500;
uint16_t fast_fric_speed_u16 = 5100;
#else
float fast_fric_speed = 7120; // 5350，5620
float low_fric_speed = 3720;

#endif

float trigger_error_speed = -1000;
float trigger_error_handle_time = 300;
float speedInBulletsPerSec = 6;//9;

uint32_t lastmousekey = 0;

uint32_t ShootBlockTick = 0;
uint32_t ShootBlockState = 0;
uint32_t LastBlockState = 0;
float BlockkScacle = 0.9f;
extern uint8_t ErrorFlag;
static float shoot_time = 0;
static float wait_time = 0;

static uint16_t aim_count = 0;
static uint8_t aim_enable = 0;

uint32_t Shoot_start_time = 0;
uint32_t Shoot_delay_time = 0;
int16_t FirstFricAccel = 0;
uint8_t bullet_id = 0;
uint8_t is_shoot_success = 0;//摩擦轮掉速判断打蛋
uint8_t is_reference_recv = 0;//裁判系统更新判断打蛋

float bullet_speed[100];
static int bullet_speed_recordnum = 0;


uint8_t Sixfric_Data_Buffer[8];


static void Shoot_Set_Mode(void);

static void Shoot_Get_CtrlValue(void);

static void Shoot_CtrlbyKey(void);

static void Shoot_Set_Control(void);

static void Send_Shoot_Output(void);

static void Shoot_Debug(void);

static void bullet_speed_record();

static float bullet_speed_control(float initial_speed, float initial_speed_ref, float fric_speed);

static void Sixfric_Data_PackUp(void);

void Shoot_Init(void) {
    // 射击模式、拨弹电机初始化
    Shoot.ShootMode = NoShooting;
    Shoot.trigger_target_angle = Shoot.TriggerMotor.total_angle;   ///此处依然不理解这样设置的缘由
    Shoot.trigger_target_angle_ramp = Shoot.TriggerMotor.total_angle;
    Shoot.TriggerMotor.Direction = 0;
    // Shoot.heatRemain = 20;
    Shoot.BulletSpeedCompensation = 0;

#ifdef FRIC_MOTOR_USE_RM3508
#if HERO == ONE
    // 摩擦轮电机初始化
    Shoot.RefSecFricSpeed = 0;
    // OLS_Init(&Shoot.FricSpeed_OLS, 10);
    TD_Init(&Shoot.FricTD, 50000, 0.001);
    for (uint8_t i = 0; i < 3; i++)
        Shoot.FricMotor[i].CAN_ID = 0x201 + i;
    PID_Init(&Shoot.FricMotor[0].PID_Velocity, 16384, 200, 0,
             25, 0, 0, 0, 0, 0.002, 0, 1, Integral_Limit | OutputFilter | DerivativeFilter);
    c[0] = 0;
    c[1] = 0;
    c[2] = 0;
    Feedforward_Init(&Shoot.FricMotor[0].FFC_Velocity, 16384, c, 0, 4, 4);
    Shoot.FricMotor[0].Max_Out = FRIC_MOTOR_MAXOUT;
    PID_Init(&Shoot.FricMotor[1].PID_Velocity, 16384, 200, 0,
             25, 0, 0, 0, 0, 0.002, 0, 1, Integral_Limit | OutputFilter | DerivativeFilter);
    c[0] = 0;
    c[1] = 0;
    c[2] = 0;
    Feedforward_Init(&Shoot.FricMotor[1].FFC_Velocity, 16384, c, 0, 4, 4);
    Shoot.FricMotor[1].Max_Out = FRIC_MOTOR_MAXOUT;
    PID_Init(&Shoot.FricMotor[2].PID_Velocity, 16384, 200, 0,
             25, 0, 0, 0, 0, 0.002, 0, 1, Integral_Limit | OutputFilter | DerivativeFilter);
    c[0] = 0;
    c[1] = 0;
    c[2] = 0;
    Feedforward_Init(&Shoot.FricMotor[2].FFC_Velocity, 16384, c, 0, 4, 4);
    Shoot.FricMotor[2].Max_Out = FRIC_MOTOR_MAXOUT;
    // 拨弹电机初始化
    TD_Init(&Shoot.TriggerTD, 7000000, 0.001f);
    PID_Init(&Shoot.TriggerMotor.PID_Angle, 5000, 5000, 0, 0.15, 0, 0.001, 0, 0, 0.001, 0.01, 0, Integral_Limit | DerivativeFilter);
    PID_Init(&Shoot.TriggerMotor.PID_Velocity, 16384, 10000, 0,
             6, 2, 0, 0, 0, 0.0001f, 0.001f, 5, Integral_Limit | ErrorHandle | OutputFilter);
    //LowPassFilter_Init(&Shoot.TriggerMotor.Lpf_AngleVelocity,10);
    Shoot.TriggerMotor.Max_Out = 16384;
    c_trigger[0] = 0;
    c_trigger[1] = 0;
    c_trigger[2] = 0;
    Feedforward_Init(&Shoot.TriggerMotor.FFC_Velocity, trigger_v_ffc_maxout, c_trigger, trigger_v_ffc_lpf, 3, 3);


    LowPassFilter_Init(&Shoot.FricMotor[0].Lpf_velocity, 20);
    LowPassFilter_Init(&Shoot.FricMotor[1].Lpf_velocity, 20);
    LowPassFilter_Init(&Shoot.FricMotor[2].Lpf_velocity, 20);

    Shoot.Back = Shoot_Back;
    Shoot.Reset = Shoot_Reset;
#else
    // 摩擦轮电机初始化
    Shoot.RefFricSpeed = 0;
    // OLS_Init(&Shoot.FricSpeed_OLS, 10);
    TD_Init(&Shoot.FricTD, 35000, 0.001);
    for (uint8_t i = 0; i < 2; i++)
    {
        Shoot.FricMotor[i].CAN_ID = 0x201 + i;
        PID_Init(&Shoot.FricMotor[i].PID_Velocity, 16384, 200, 0,
                 20, 10, 0, 0, 0, 0.0005, 0.0005, 5, Integral_Limit | OutputFilter);

        Shoot.FricMotor[i].Max_Out = FRIC_MOTOR_MAXOUT;
    }
    // 拨弹电机初始化
    PID_Init(&Shoot.TriggerMotor.PID_Velocity, 16384, 12000, 0,
             -12, -100, 0, 500, 1000, 0.001, 0.001, 5, Integral_Limit | ErrorHandle);
    Shoot.TriggerMotor.Max_Out = 16384.0f;
    c_trigger[0] = 0;
    c_trigger[1] = 0;
    c_trigger[2] = 0;
    Feedforward_Init(&Shoot.TriggerMotor.FFC_Velocity, trigger_v_ffc_maxout, c_trigger, trigger_v_ffc_lpf, 3, 3);

#endif
#else
    // 使用pwm控制的电机初始化
    Shoot.FricPWM = 1000;
    Shoot.PWM_TIM = FRIC_MOTOR_TIM;
    Shoot.PWM_CHANNEL_1 = FRIC_MOTOR_TIM_CHANNEL_1;
    Shoot.PWM_CHANNEL_2 = FRIC_MOTOR_TIM_CHANNEL_2;
    HAL_TIM_PWM_Start(&Shoot.PWM_TIM, Shoot.PWM_CHANNEL_1);
    HAL_TIM_PWM_Start(&Shoot.PWM_TIM, Shoot.PWM_CHANNEL_2);
    TIM_Set_PWM(&Shoot.PWM_TIM, Shoot.PWM_CHANNEL_1, FRIC_MOTOR_SUSPEND_CCR);
    TIM_Set_PWM(&Shoot.PWM_TIM, Shoot.PWM_CHANNEL_2, FRIC_MOTOR_SUSPEND_CCR);
#endif
}

void Shoot_Control(void) {
    //获取dt与t
    dt = DWT_GetDeltaT(&Shoot_DWT_Count);
    t += dt;
    // 设置射击模式
    Shoot_Set_Mode();
    // 获取射击相关电机控制量
    Shoot_CtrlbyKey();
    //设置控制量
    Shoot_Get_CtrlValue();
    // 射击相关PID控制
    Shoot_Set_Control();
    // 发送射击相关电机控制电流
    Send_Shoot_Output();
    // 发给六摩擦c板数据打包
    Sixfric_Data_PackUp();
}

static void Shoot_Set_Mode(void) {
#ifdef USE_BULLETCOUNT
    static uint32_t BulletCountLast = 0;
#endif
    static uint32_t ModeSwitchTick;
    static uint8_t rcLeftSwitchValueLast = 0, rcRightSwitchValueLast = 0, rcMouseLeftLast = 0, rcMouseRightLast = 0;
    static uint32_t OneShotCount = 0;
    static uint32_t SpinningShotCount = 0;
    static uint32_t ShotCount = 0;
    static uint8_t OneShot_delayCount = 0;
    static uint8_t BackCount = 0;

    if ((remote_control.switch_left == Switch_Down && rcLeftSwitchValueLast != Switch_Down)) // 遥控器左拨杆置下开关摩擦轮
    {
        if (Shoot.RefSecFricSpeed >= FRIC_MOTOR_MIN_RPM)
            Shoot.isFricOn = 0;
        else
            Shoot.isFricOn = 1;
    }
    if (remote_control.mouse.press_left == 1 && Shoot.RefSecFricSpeed <= FRIC_MOTOR_MIN_RPM) // 鼠标左键按下时开摩擦轮
    {
        Shoot.isFricOn = 1;
    }
    if (Shoot.Shoot_Motor_State == 1) {
        // 两级摩擦轮转速都存在时，开启激光
        HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
        if (remote_control.switch_left == Switch_Up && rcLeftSwitchValueLast != Switch_Up &&
            Gimbal.Mode != AimAssist_Mode) {
            Shoot.trigger_flag = 1;
            Shoot_start_time = USER_GetTick();
        }
    } else {
        // 摩擦轮转速不存在时，关闭激光，改变射击模式为不射击
        HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);
    }



    if (Gimbal.Mode == AimAssist_Mode && Shoot.Shoot_Motor_State) {
        aim_count++;//等待裁判系统数据更新
        if (aim_count > 100) {
            aim_enable = 1;
            aim_count = 0;
        }
        if(AimAssist.Status == 2)
        {
            if (/*ShootEvaluation.Allow_Shoot && !ShootEvaluation.Last_Allow_Shoot && aim_enable && (abs(Gimbal.YawError)/180.0*PI <ShootEvaluation.yaw_allow_range)
        && (abs(Gimbal.PitchError)/180.0*PI<ShootEvaluation.pitch_allow_range) &&*/ (remote_control.mouse.press_left ||
                remote_control.switch_left == Switch_Up)) {
            Shoot.trigger_flag = 1;
            aim_enable = 0;
            Shoot_start_time = USER_GetTick();
        }
        ShootEvaluation.Last_Allow_Shoot = ShootEvaluation.Allow_Shoot;
        }
        else if(AimAssist.Status == 1)
        {
            if (aim_enable && (abs(Gimbal.YawError)/180.0*PI <ShootEvaluation.yaw_allow_range)
                && (abs(Gimbal.PitchError)/180.0*PI<ShootEvaluation.pitch_allow_range) && (remote_control.mouse.press_left ||
                                                                                           remote_control.switch_left == Switch_Up)) {
                Shoot.trigger_flag = 1;
                aim_enable = 0;
                Shoot_start_time = USER_GetTick();
            }
        }
    } else {
        aim_enable = 0;
        aim_count = 0;
    }

    // if (GlobalDebugMode == SHOOT_DEBUG)
    //     Shoot.ShootMode = DebugShoot;

    if (GlobalDebugMode == SHOOT_DEBUG)
        Shoot_Debug();
    // 更新遥控器、键盘、鼠标各控制数据
    rcLeftSwitchValueLast = remote_control.switch_left;
    rcRightSwitchValueLast = remote_control.switch_right;
    rcMouseLeftLast = remote_control.mouse.press_left;
    rcMouseRightLast = remote_control.mouse.press_right;
    lastmousekey = remote_control.mouse.press_left;
}

static void Shoot_Get_CtrlValue(void) {
    static float heatGain = 1.5;
    static float ShootFreqGain = 0;
    static uint16_t shoot_finish_cnt = 0;
    static int16_t last_bullet_num;
    static uint16_t block_count;
    static uint16_t back_count;

    if ((is_TOE_Error(RC_TOE) && is_TOE_Error(VTM_TOE) && GlobalDebugMode != SHOOT_DEBUG) || is_TOE_Error(TRIGGER_MOTOR_TOE))
    {
        Shoot.isFricOn = 0;
    }
    if (Shoot.isFricOn == 1) {
        if(!Shoot.Speed_Flag){
            Shoot.RefFirstFricSpeed = low_first_fric_speed;
            Shoot.RefSecFricSpeed = low_second_fric_speed;
        }else if(Shoot.Speed_Flag){
            Shoot.RefFirstFricSpeed = fast_first_fric_speed;
            Shoot.RefSecFricSpeed = fast_second_fric_speed;
        }
    } else {
        // 若摩擦轮开启标志为0，则设置摩擦轮期望为0
        Shoot.RefSecFricSpeed = 0;
        Shoot.RefFirstFricSpeed = 0;
    }
    if (Shoot.FricSpeed >= FRIC_MOTOR_MIN_RPM)
        Shoot.Shoot_Motor_State = 1;
    else
        Shoot.Shoot_Motor_State = 0;

    // 拨弹电机设置
    if (Shoot.isFricOn == 1 && Shoot.Shoot_Motor_State == 0) {
        block_count++;
        if (block_count > 150) {
            Shoot.FricBlocked = 1;
            block_count = 0;
        }
        if (Shoot.FricBlocked == 1 && remote_control.key_code & Key_Q) {
            Shoot.trigger_target_angle -= TRIGGER_MOTOR_REDUCTION_RATIO * 10.0f;
            RAMP_CAL_STEP(&Ramp, Shoot.TriggerMotor.total_angle, Shoot.trigger_target_angle, 2, 70000);
            Shoot.FricBlocked = 0;

        }
    } else {
        Shoot.FricBlocked = 0;
        block_count = 0;
    }

    Shoot.Back(&Shoot);//回拨检测
//    if (!robot_state.power_management_s6hooter_output)
//        Shoot.Reset(&Shoot);
    // 若下一发打出后热量将超限，改变射击模式为不射击，结束拨弹；除非操作手按住R，可无视此热量保护用英雄血量强换基地血量
    if ((robot_state.shooter_barrel_heat_limit - power_heat_data.shooter_42mm_barrel_heat <= 100) && !(remote_control.key_code & Key_R)) {
        Shoot.trigger_flag = 0;
    }
/*    if ((Shoot.Bullet_Remaining_Num == 1) && !(remote_control.key_code & Key_R)) {
        Shoot.trigger_flag = 0;
    }*/
   if (shoot_time + wait_time > DWT_GetTimeline_ms()) {
       Shoot.trigger_flag = 0;
       //return;
   }
    switch (Gimbal.Mode) {
        case Normal_Mode:
            if (Shoot.trigger_flag == 1 && !Shoot.back_flag) {
                Shoot.trigger_target_angle += TRIGGER_MOTOR_REDUCTION_RATIO * 60.0f;
                RAMP_CAL_STEP(&Ramp, Shoot.TriggerMotor.total_angle, Shoot.trigger_target_angle, 2, 70000);
                Shoot.trigger_flag = 0;
                shoot_time = DWT_GetTimeline_ms();
                wait_time = 150;
            }
            break;
        case Precise_Mode:
            if (Shoot.trigger_flag == 1 && !Shoot.back_flag) {
                Shoot.trigger_target_angle += TRIGGER_MOTOR_REDUCTION_RATIO * 60.0f;
                RAMP_CAL_STEP(&Ramp, Shoot.TriggerMotor.total_angle, Shoot.trigger_target_angle, 2, 70000);
                Shoot.trigger_flag = 0;
                shoot_time = DWT_GetTimeline_ms();
                wait_time = 150;
            }
            break;
        case AimAssist_Mode:
            if (Shoot.trigger_flag == 1 && !Shoot.back_flag && ShootEvaluation.Allow_Shoot) {
                Shoot.trigger_target_angle += TRIGGER_MOTOR_REDUCTION_RATIO * 60.0f;
                RAMP_CAL_STEP(&Ramp, Shoot.TriggerMotor.total_angle, Shoot.trigger_target_angle, 2, 70000);
                Shoot.trigger_flag = 0;
                aim_enable = 0;
                shoot_time = DWT_GetTimeline_ms();
                wait_time = 700;
            }
    }

    if (is_TOE_Error(RC_TOE) && is_TOE_Error(VTM_TOE) && GlobalDebugMode != SHOOT_DEBUG) {

        Ramp.step_val = 0;
        Ramp.inc_val = 0;
        Ramp.target_val = Shoot.TriggerMotor.total_angle;
        Ramp.present_ref = Shoot.TriggerMotor.total_angle;
        Shoot.trigger_target_angle_ramp = Shoot.TriggerMotor.total_angle;
        Shoot.trigger_target_angle = Shoot.TriggerMotor.total_angle;
        Shoot.TriggerMotor.PID_Velocity.Iout = 0;
    }
}

static void Shoot_Set_Control(void) {
    static uint8_t LastERROR_Type;
    static uint32_t triggerMotorBlockCount = 0;
    static float last_fric_speed;
    static uint8_t blockprotect_count = 0;
    static uint8_t BackCount;
    static float fric_velocity_lpf[3];
    static float FricAccel_last;



// 防止误触发，给予操作手恢复拨弹电机输出的操作空间
//    if ((remote_control.key_code & Key_Q)) {
//        Shoot.TriggerMotor.Max_Out = 16384.0f;
//        Shoot.is_Trigger_Motor_OK = 1;
//    }
// 计算三摩擦轮平均速度，计算第一级摩擦轮平均加速度
    Shoot.FricSpeed = (fabsf(Shoot.FricMotor[0].Velocity_RPM) + fabsf(Shoot.FricMotor[1].Velocity_RPM) +
                       fabsf(Shoot.FricMotor[2].Velocity_RPM)) / 3;
    Shoot.FricAccel = (Shoot.FricSpeed - last_fric_speed) / (0.01f + dt) + Shoot.FricAccel * 0.01f / (0.01f + dt);


    if (FirstFricAccel < 20000 && FricAccel_last > 20000) {
//Shoot.trigger_target_angle = Shoot.TriggerMotor.total_angle;
//bullet_control_flag = 1;
//bullet_speed_record();
        is_shoot_success = 1;
        bullet_id += 1;
        Shoot_delay_time = USER_GetTick() - Shoot_start_time;
    }

    if (is_reference_recv == 1) {
        CDC_Send2miniPC_Shoot_feedback_Data(bullet_id, is_shoot_success, is_reference_recv, Gimbal.PitchAngle,
                                            Shoot.initial_speed,
                                            Shoot_delay_time);
        is_reference_recv = 0;
        is_shoot_success = 0;
    }

    FricAccel_last = FirstFricAccel;
//if (Shoot.TriggerMotor.PID_Velocity.ERRORHandler.ERRORType != Motor_Blocked)
// 拨弹电机PID控制
#ifdef SHOOT_USE_ANGLE_LOOP  //若改回速度控制，需要删除这个宏定义
#ifdef SHOOT_USE_RAMP
    Shoot.trigger_target_angle_ramp = RAMP_REF(&Ramp);
    //Shoot.TriggerMotor.AngleVelocity_LPF = LowPassFilter( Shoot.TriggerMotor.para.vel, &Shoot.TriggerMotor.Lpf_AngleVelocity, dt);
    PID_Calculate(&Shoot.TriggerMotor.PID_Angle, Shoot.TriggerMotor.total_angle, Shoot.trigger_target_angle_ramp);
    PID_Calculate(&Shoot.TriggerMotor.PID_Velocity,
                  Shoot.TriggerMotor.Velocity_RPM,
                  Shoot.TriggerMotor.PID_Angle.Output);
    Shoot.TriggerMotor.Output = float_constrain(Shoot.TriggerMotor.PID_Velocity.Output, -16384, 16384);
#else
    TD_Calculate(&Shoot.TriggerTD, Shoot.trigger_target_angle);
    //拨弹电机角度环控制
    PID_Calculate(&Shoot.TriggerMotor.PID_Angle, Shoot.TriggerMotor.total_angle, Shoot.TriggerTD.x);
    PID_Calculate(&Shoot.TriggerMotor.PID_Velocity,
                  Shoot.TriggerMotor.Velocity_RPM,
                  Shoot.TriggerMotor.PID_Angle.Output);
    Shoot.TriggerMotor.Output = float_constrain(Shoot.TriggerMotor.PID_Velocity.Output, -16384, 16384);
#endif
#else
    Motor_Speed_Calculate(&Shoot.TriggerMotor, Shoot.TriggerMotor.Velocity_RPM, -Shoot.TriggerSpeed);
#endif
    // 摩擦轮电机PID控制
#ifdef FRIC_MOTOR_USE_RM3508
    fric_velocity_lpf[0] = LowPassFilter(Shoot.FricMotor[0].Velocity_RPM, &Shoot.FricMotor[0].Lpf_velocity, dt);
    fric_velocity_lpf[1] = LowPassFilter(Shoot.FricMotor[1].Velocity_RPM, &Shoot.FricMotor[1].Lpf_velocity, dt);
    fric_velocity_lpf[2] = LowPassFilter(Shoot.FricMotor[2].Velocity_RPM, &Shoot.FricMotor[2].Lpf_velocity, dt);
    TD_Calculate(&Shoot.FricTD, Shoot.RefSecFricSpeed);   //稳速，避免速度期望突变的错误值影响弹道
    Motor_Speed_Calculate(&Shoot.FricMotor[0], fric_velocity_lpf[0], Shoot.FricTD.x);
    Motor_Speed_Calculate(&Shoot.FricMotor[1], fric_velocity_lpf[1], -Shoot.FricTD.x);
    Motor_Speed_Calculate(&Shoot.FricMotor[2], fric_velocity_lpf[2], -Shoot.FricTD.x);
    /*Motor_Speed_Calculate(&Shoot.FricMotor[0], Shoot.FricMotor[0].Velocity_RPM, Shoot.FricTD.x);
    Motor_Speed_Calculate(&Shoot.FricMotor[1], Shoot.FricMotor[1].Velocity_RPM, -Shoot.FricTD.x);
    Motor_Speed_Calculate(&Shoot.FricMotor[2], Shoot.FricMotor[2].Velocity_RPM, -Shoot.FricTD.x);*/
#endif
    // 更新第一级摩擦轮转速
    last_fric_speed = Shoot.FricSpeed;
}

static void Send_Shoot_Output(void) {
    static uint32_t fric_motor_startCount = 0;
    //若遥控器未开启或R键E键一起按，将电机电流给0
    if ((is_TOE_Error(RC_TOE) && is_TOE_Error(VTM_TOE) && GlobalDebugMode != SHOOT_DEBUG) ||
        ((remote_control.key_code & Key_R) && (remote_control.key_code & Key_E))) {

        Shoot.is_sendcurrent_error = 1;

#ifdef FRIC_MOTOR_USE_RM3508
        Shoot.trigger_target_angle = Shoot.TriggerMotor.total_angle;
        //Send_Motor_Current_DMTrigger(&hcan1,0,0,0,0,0);
        Send_Motor_Current_1_4(&hcan1, 0, 0, 0, 0);
        Send_Motor_Current_1_4(&hcan2, 0, 0, 0, 0);
#else
        Send_Motor_Current_5_8(&hcan2, 0, 0, 0, 0);
        TIM_Set_PWM(&Shoot.PWM_TIM, Shoot.PWM_CHANNEL_1, FRIC_MOTOR_SUSPEND_CCR);
        TIM_Set_PWM(&Shoot.PWM_TIM, Shoot.PWM_CHANNEL_2, FRIC_MOTOR_SUSPEND_CCR);
#endif
    } else {
        // if (remote_control.key_code & Key_G)
        // {
        //     Shoot.isFricOn = 0;
        //     if (Shoot.TriggerMotor.Direction == NEGATIVE)
        //     {
        //         Shoot.FricMotor[0].Output = 3000;
        //         Shoot.FricMotor[1].Output = -3000;
        //     }
        //     fric_motor_startCount++;
        //     if (fric_motor_startCount > 100)
        //     {
        //         Shoot.FricMotor[0].Output = 0;
        //         Shoot.FricMotor[1].Output = 0;
        //     }
        // }
        // else
        //     fric_motor_startCount = 0;

        Shoot.is_sendcurrent_error = 0;
        // 发送摩擦轮电机与拨弹电机电流
#ifdef FRIC_MOTOR_USE_RM3508
        //Send_Motor_Current_DMTrigger(&hcan1,0,0,0,0,Shoot.TriggerMotor.Output);
        //Send_Motor_Current_1_4(&hcan1, 0, Shoot.TriggerMotor.Output, 0, 0);
        Send_Motor_Current_1_4(&hcan2, Shoot.FricMotor[0].Output, Shoot.FricMotor[1].Output, Shoot.FricMotor[2].Output,
                               0);
#else
        Send_Motor_Current_1_4(&hcan1, Shoot.TriggerMotor.Output, 0, 0, 0);
        TIM_Set_PWM(&Shoot.PWM_TIM, Shoot.PWM_CHANNEL_1, Shoot.FricPWM);
        TIM_Set_PWM(&Shoot.PWM_TIM, Shoot.PWM_CHANNEL_2, Shoot.FricPWM);
#endif
    }
    // 更新发射模式
    Shoot.LastShootMode = Shoot.ShootMode;
}

static void Shoot_Debug(void) {
    switch (ShootDebugMode) {
        case 0:
            Serial_Debug(&huart1,
                         3,
                         amp[0] * Shoot.TriggerMotor.PID_Velocity.Ref,
                         amp[1] * Shoot.TriggerMotor.PID_Velocity.Measure,
                         amp[2] * Shoot.TriggerMotor.PID_Velocity.ERRORHandler.ERRORType,
                         0,
                         0,
                         0);
            break;
        case 1:
            Serial_Debug(&huart1, 3, Shoot.FricAccel, Shoot.FricSpeed,
                         10000 * Shoot.ShootMode, 10 * Shoot.TriggerMotor.Angle,
                         10 * Shoot.TriggerMotor.Velocity_RPM, 0);
            break;
        case 2:
            Serial_Debug(&huart1, 3, Shoot.FricMotor[0].PID_Velocity.Ref, -Shoot.FricMotor[1].PID_Velocity.Ref,
                         Shoot.FricMotor[0].Velocity_RPM, -Shoot.FricMotor[0].Velocity_RPM,
                         0, 0);
            break;
        case 3:
            Serial_Debug(&huart1,
                         3,
                         Shoot.TriggerMotor.PID_Velocity.Ref,
                         Shoot.TriggerMotor.PID_Velocity.Measure,
                         Shoot.TriggerMotor.PID_Velocity.MaxOut,
                         Shoot.TriggerMotor.PID_Velocity.Output,
                         Shoot.TriggerMotor.Output,
                         0);
            break;
        case 4:
            if (Shoot.ShootMode == OneShot) {
                if (Shoot.LastShootMode != OneShot)
                    Serial_Debug(&huart1, 1, shoot_data.initial_speed * 1.0f, 0, 0, 0, 0, 0);
            }
            break;
        default:
            break;
    }
}

void Shoot_Back(Shoot_t *shoot) {
    static uint16_t Back_cnt1 = 0, Back_cnt2 = 0, Back_cnt3 = 0;
    if (shoot->TriggerMotor.Output >= 16384) {
        Back_cnt2++;
        if (Back_cnt2 > 500) {
            Back_cnt2 = 0;
            shoot->back_flag = 1;
        }
    } else {
        Back_cnt2 = 0;
    }
    if (shoot->back_flag == 1) {
        shoot->trigger_target_angle -= TRIGGER_MOTOR_REDUCTION_RATIO * 60.0f;
        RAMP_CAL_STEP(&Ramp, Shoot.TriggerMotor.total_angle, Shoot.trigger_target_angle, 2, 70000);
        shoot->back_flag = 0;
    }
}

static void Shoot_CtrlbyKey(void) {
    static uint8_t rcMouseLeftLast = 0;//用于检测上升沿
    static uint8_t Mouse_L_flag = 0;
    static uint16_t Mouse_L_cnt = 0;
    static uint8_t Mouse_L_Press = 0;
    static uint16_t LastKeyCode = 0;
    if (remote_control.key_code & Key_X && !(LastKeyCode & Key_X)) // 操作手按下X开关摩擦轮
    {
        if (Shoot.RefSecFricSpeed >= FRIC_MOTOR_MIN_RPM)
            Shoot.isFricOn = 0;
        else
            Shoot.isFricOn = 1;
    }
    if(Shoot.isFricOn == 1)
    if(remote_control.key_code & Key_R && !(LastKeyCode & Key_R))
    {
        if(Shoot.Speed_Flag){
            Shoot.Speed_Flag = 0;
        }else if(!Shoot.Speed_Flag){
            Shoot.Speed_Flag = 1;
        }
    }

    if (Shoot.Shoot_Motor_State) {
        if (!Mouse_L_flag) {
            Mouse_L_cnt++;
            if (Mouse_L_cnt > 50) {
                Mouse_L_flag = 1;
                Mouse_L_cnt = 0;
            }
        }
        if (rcMouseLeftLast == 0 && remote_control.mouse.press_left == 1) {
            Mouse_L_Press = 1;
        } else {
            Mouse_L_Press = 0;
        }
        if (Mouse_L_flag && Mouse_L_Press && Gimbal.Mode != AimAssist_Mode) {
            Shoot.trigger_flag = 1;
            Mouse_L_flag = 0;
        }
    } else {
        Mouse_L_flag = 0;
        Mouse_L_cnt = 0;
    }

    rcMouseLeftLast = remote_control.mouse.press_left;
    LastKeyCode = remote_control.key_code;
}

void Shoot_Reset(Shoot_t *shoot) {
    shoot->trigger_flag = 0;
    shoot->trigger_target_angle = shoot->TriggerMotor.total_angle;
}

static void bullet_speed_record() {
    static uint16_t record_cnt = 0;
    uint8_t speed_record_flag = 1;

    if (speed_record_flag == 1) {
        bullet_speed[bullet_speed_recordnum] = Shoot.initial_speed;
        CDC_Send2miniPC_Shoot_Data(Shoot.initial_speed);
        Serial_Debug(&huart1, 1, Shoot.initial_speed, 0, 0, 0, 0, 0);
        bullet_speed_recordnum++;
        if (bullet_speed_recordnum == 99) {
            bullet_speed_recordnum = 0;
        }
        speed_record_flag = 0;
    }

    if (speed_record_flag == 0) {
        record_cnt++;
        if (record_cnt > 500) {
            record_cnt = 0;
            speed_record_flag = 1;
        }
    }
}

static float bullet_speed_control(float initial_speed, float initial_speed_ref, float fric_speed) {
    float fric_speed_cal = 0;
    float error = 0;
    float deadband = 0.15;
    float max_out = 6000, min_out = 5700;

    error = initial_speed_ref - initial_speed;

    if (abs(error) < deadband) {
        fric_speed_cal = fric_speed;
    } else {
        fric_speed_cal = fric_speed + (error * 300);
    }

    if (fric_speed_cal > max_out) {
        fric_speed_cal = max_out;
    }
    if (fric_speed_cal < min_out) {
        fric_speed_cal = min_out;
    }

    return fric_speed_cal;
}

static void Sixfric_Data_PackUp(void) {
    Sixfric_Data_Buffer[0] = Shoot.isFricOn;
    Sixfric_Data_Buffer[1] = Shoot.is_sendcurrent_error;
    Sixfric_Data_Buffer[2] = ((uint16_t)(Shoot.RefFirstFricSpeed) >> 8) & 0xFF;
    Sixfric_Data_Buffer[3] = (uint16_t)(Shoot.RefFirstFricSpeed) & 0xFF;
    Sixfric_Data_Buffer[4] = 0;
    Sixfric_Data_Buffer[5] = 0;
    Sixfric_Data_Buffer[6] = 0;
    Sixfric_Data_Buffer[7] = 0;
}