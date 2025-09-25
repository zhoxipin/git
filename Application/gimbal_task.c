/**
 ******************************************************************************
 * @file    gimbal_task.c
 * @author  Wang Hongxi
 * @version V2.0.0
 * @date    2021/09/07
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "gimbal_task.h"
#include "bsp_CAN.h"
#include "remote_control.h"
#include "ins_task.h"
#include "QuaternionEKF.h"
#include "arm_math.h"

Gimbal_t Gimbal = {0};
GimbalSI_t GimbalSI;
uint8_t GimbalTuningEnable = 0;

uint8_t PitchLock = 0;   //如果Pitch电机的仰角+输出长时间不正常则用此标志位将输出置0
uint8_t PitchLockCount = 0;  //Pitch电机堵转计数，超500ms即算为堵转
uint8_t PitchReback = 0;   //Pitch电机堵转后恢复计时 输出给0后1s恢复
static float c[3] = {0};

uint32_t Gimbal_DWT_Count = 0;
static float dt = 0, t = 0;
uint32_t CAN_SEND_ERROR_COUNT = 0;

uint8_t debugMode = 3, ControlerMode = 0;
static float amp[6] = {10, 10, 0.001, 0.001, 0.001, 0.001};
float YawOmega = 4, PitchOmega = 4, YawAmp = 5, PitchAmp = 5, YawOffset = 0, PitchOffset = 10, YawDisturbance = 0, PitchDisturbance = 0;
float ElevationIMU_Correction = 7.0;
float DepressionIMU_Correction = 7.0;
float is_precise_OK = 0;
uint8_t gimbal_protection_flag = 0;
uint8_t vision_correction_flag = 0;
uint8_t pitch_correction_flag = 0;
uint8_t LastChassis_data = 0;
uint8_t Gimbal_Data_Buffer[8];
// static void Gimbal_Position_Reset(void);
static void GimbalSI_Init(void);

static void Vision_Init(void);

static void Vision_Correction(void);

static void Pitch_Correction(void);

static void Gimbal_Set_Mode(void);

static void Gimbal_Get_CtrlValue(void);

static void Gimbal_CtrlValue_Limit(void);

static void Gimbal_Set_Control(void);

static void Send_Gimbal_Current(void);

static void GimbalSI_Calculate(void);

static void Gimbal_Tuning(void);

static void Gimbal_Data_PackUp(void);

void Gimbal_Init(void) {
    //云台电机初始化
    memset(&Gimbal.PitchMotor, 0, sizeof(Gimbal.PitchMotor));
    //memset(&Gimbal.YawMotor, 0, sizeof(Gimbal.YawMotor));

    Vision_Init();

    // Yaw电机正方向与手动机械角度零位校准初始化
    Gimbal.PitchMotor.Direction = NEGATIVE;
    Gimbal.YawMotor.Direction = 0;
    Gimbal.YawMotor.zero_offset = YAW_MOTOR_ZERO_OFFSET;
    Gimbal.PitchZeroOffset = PITCH_MOTOR_ZERO_OFFSET;

    // 电机CAN ID初始化
    Gimbal.YawMotor.CAN_ID = YAW_MOTOR_ID;
    Gimbal.PitchMotor.CAN_ID = PITCH_MOTOR_ID;


    // pitch电机最大俯仰角初始化
    Gimbal.DepressionIMU = GIMBAL_MAX_DEPRESSION;
    Gimbal.ElevationIMU = GIMBAL_MAX_ELEVATION;
    Gimbal.DepressionMechAngle = PITCH_MIN_MECH_TOTALANGLE;
    Gimbal.ElevationMechAngle = PITCH_MAX_MECH_TOTALANGLE;

    Gimbal.ConversionFactor = PITCH_CONVERSION_FACTOR;

    // 遥控器与鼠标控制倍率初始化
    Gimbal.rcStickYawRatio = RC_STICK_YAW_RATIO;
    Gimbal.rcStickPitchRatio = RC_STICK_PITCH_RATIO;
    Gimbal.rcMouseYawRatio = RC_MOUSE_YAW_RATIO;
    Gimbal.rcMousePitchRatio = RC_MOUSE_PITCH_RATIO;

    // 云台模式初始化
    Gimbal.Mode = Normal_Mode;

    // 云台TD初始化
    TD_Init(&Gimbal.YawRefAngularVelocityTD, 5000000, 0.003);
    TD_Init(&Gimbal.PitchRefAngularVelocityTD, 5000000, 0.003);
    TD_Init(&Gimbal.YawRefAngleTD, 2000000, 0.003);
    TD_Init(&Gimbal.PitchRefAngleTD, 2000000, 0.003);
    TD_Init(&Gimbal.PitchRefMechAngleTD, 2000000, 0.003);

    // yaw电机PID初始化
    PID_Init(&Gimbal.YawMotor.PID_Torque, YAW_T_PID_MAXOUT, YAW_T_PID_MAXINTEGRAL, 0,
             0, YAW_T_PID_KI, 0, 0, 0,
             0, 0, 0,
             Integral_Limit | Trapezoid_Intergral | OutputFilter);
    c[0] = 1;
    c[1] = 0;
    c[2] = 0;
    Feedforward_Init(&Gimbal.YawMotor.FFC_Torque, 2000, c, 0, 4, 4);

    PID_Init(&Gimbal.YawMotor.PID_Velocity, YAW_V_PID_MAXOUT, YAW_V_PID_MAXINTEGRAL, 0,
             YAW_V_PID_KP, YAW_V_PID_KI, YAW_V_PID_KD, 0, 0,
             YAW_V_PID_LPF, 0, 0,
             Integral_Limit | Trapezoid_Intergral | OutputFilter);

    c[0] = YAW_V_FCC_C0;
    c[1] = YAW_V_FCC_C1;
    c[2] = YAW_V_FCC_C2;
    Feedforward_Init(&Gimbal.YawMotor.FFC_Velocity, YAW_V_FFC_MAXOUT, c, YAW_V_FCC_LPF, 8, 8);

    PID_Init(&Gimbal.YawMotor.PID_Angle, YAW_A_PID_MAXOUT, YAW_A_PID_MAXINTEGRAL, 0,
             YAW_A_PID_KP, YAW_A_PID_KI, YAW_A_PID_KD, 0, 0, YAW_A_PID_LPF, YAW_A_PID_D_LPF, 3,
             Integral_Limit | Trapezoid_Intergral | OutputFilter | DerivativeFilter);
    LowPassFilter_Init(&Gimbal.YawMotor.Lpf_AngleInDegree,10);
    c[0] = YAW_A_FCC_C0;
    c[1] = YAW_A_FCC_C1;
    c[2] = YAW_A_FCC_C2;
    Feedforward_Init(&Gimbal.YawMotor.FFC_Angle, YAW_A_FFC_MAXOUT, c, YAW_A_FCC_LPF, 5, 5);
    Gimbal.YawMotor.Max_Out = YAW_MOTOR_MAXOUT;

    // pitch电机PID初始化
    PID_Init(&Gimbal.PitchMotor.PID_Torque, 2000, 2000, 0,
             0, PITCH_T_PID_KI, 0, 0, 0, 0, 0, 0,
             Integral_Limit | Trapezoid_Intergral | OutputFilter);
    c[0] = 1;
    c[1] = 0;
    c[2] = 0;
    Feedforward_Init(&Gimbal.PitchMotor.FFC_Torque, 2000, c, 0, 4, 4);
    //Gimbal.PitchMotor.Ke = 0;

    PID_Init(&Gimbal.PitchMotor.PID_Velocity, PITCH_V_PID_MAXOUT, PITCH_V_PID_MAXINTEGRAL, 0,
             PITCH_V_PID_KP, PITCH_V_PID_KI, PITCH_V_PID_KD, 0, 0, PITCH_V_PID_LPF, 0, 0,
             Integral_Limit | Trapezoid_Intergral | OutputFilter);
    c[0] = PITCH_V_FCC_C0;
    c[1] = PITCH_V_FCC_C1;
    c[2] = PITCH_V_FCC_C2;
    Feedforward_Init(&Gimbal.PitchMotor.FFC_Velocity, PITCH_V_FFC_MAXOUT, c, PITCH_V_FCC_LPF, 8, 8);

    PID_Init(&Gimbal.PitchMotor.PID_Angle, PITCH_A_PID_MAXOUT, PITCH_A_PID_MAXINTEGRAL, 0,
             PITCH_A_PID_KP, PITCH_A_PID_KI, PITCH_A_PID_KD, 0, 0, PITCH_A_PID_LPF, PITCH_A_PID_D_LPF, 3,
             Integral_Limit | Trapezoid_Intergral | OutputFilter | DerivativeFilter);
    c[0] = PITCH_A_FCC_C0;
    c[1] = PITCH_A_FCC_C1;
    c[2] = PITCH_A_FCC_C2;
    Feedforward_Init(&Gimbal.PitchMotor.FFC_Angle, PITCH_A_FFC_MAXOUT, c, PITCH_A_FCC_LPF, 5, 5);

    PID_Init(&Gimbal.PitchMotor.PID_MechAngle, PITCH_MA_PID_MAXOUT, PITCH_MA_PID_MAXINTEGRAL, 0,
             PITCH_MA_PID_KP, PITCH_MA_PID_KI, PITCH_MA_PID_KD, 0, 0, PITCH_MA_PID_LPF, PITCH_MA_PID_D_LPF, 3,
             Integral_Limit | Trapezoid_Intergral | OutputFilter | DerivativeFilter);
    Gimbal.PitchMotor.Max_Out = PITCH_MOTOR_MAXOUT;
    
    GimbalSI_Init();
}

static void GimbalSI_Init(void) {
    GimbalSI.Q0 = 0.001;
    GimbalSI.Q1 = 0.001;
    GimbalSI.Q2 = 0.001;
    GimbalSI.R = 10000;
    GimbalSI.lambda = 0.999;

    FirstOrderSI_Init(&GimbalSI.YawSI, 0, 0, GimbalSI.Q0, GimbalSI.Q1, GimbalSI.Q2, GimbalSI.R, GimbalSI.lambda);
    FirstOrderSI_Init(&GimbalSI.PitchSI, 0, 0, GimbalSI.Q0, GimbalSI.Q1, GimbalSI.Q2, GimbalSI.R, GimbalSI.lambda);
}

void Gimbal_Control(void) {
    // 获取dt与t
    dt = DWT_GetDeltaT(&Gimbal_DWT_Count);
    t += dt;
    // 图传2006堵转标零
    Vision_Correction();
    //丝杆pitch堵转标零
    Pitch_Correction();
    // 设置云台运动模式
    Gimbal_Set_Mode();
    // 处理来自不同设备的控制量
    Gimbal_Get_CtrlValue();
    // 云台运动解算及PID计算
    Gimbal_Set_Control();
    // 发送云台电机控制电流
    Send_Gimbal_Current();

    GimbalSI_Calculate();
    // 云台部分数据打包 用于通讯
    Gimbal_Data_PackUp();
}

static void Vision_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,5000);

    Gimbal.VisionMotor.Direction = 0;
    Gimbal.VisionMotor.CAN_ID = VISION_MOTOR_ID;
    Gimbal.VisionMotor.zero_offset = 0;
    Gimbal.VisionAngleOffset = 0;
    Gimbal.PitchVisionAngle = VISION_PITCH_ZERO_OFFSET;

    TD_Init(&Gimbal.VisionRefAngleTD, 2000000, 0.003);

    //图传2006电机PID初始化
    PID_Init(&Gimbal.VisionMotor.PID_Velocity, VISION_V_PID_MAXOUT, VISION_V_PID_MAXINTEGRAL, 0,
             VISION_V_PID_KP, VISION_V_PID_KI, VISION_V_PID_KD, 0, 0, 0, 0, 0,
             Integral_Limit | Trapezoid_Intergral | OutputFilter);
    PID_Init(&Gimbal.VisionMotor.PID_Angle, VISION_A_PID_MAXOUT, VISION_A_PID_MAXINTEGRAL, 0,
             VISION_A_PID_KP, VISION_A_PID_KI, VISION_A_PID_KD, 0, 0, 0, 0, 0,
             Integral_Limit | Trapezoid_Intergral | OutputFilter | DerivativeFilter);
}

static void Vision_Correction(void)
{
    static uint8_t vision_init_count = 0;

    if(is_TOE_Error(VISION_MOTOR_TOE) && vision_correction_flag == 1)
    {
        vision_correction_flag = 0;
    }

    if(vision_correction_flag == 0 && (!(is_TOE_Error(RC_TOE)) || !(is_TOE_Error(VTM_TOE))))
    {
        Gimbal.VisionMotor.Max_Out = 2500;
        Gimbal.VisionRefAngularVelocity = -1000;
        if ((fabsf(Gimbal.VisionMotor.Output) >= 2450)&&(fabsf(Gimbal.VisionMotor.Velocity_RPM) <= 10))
        {
            vision_init_count ++;
        }
        if(vision_init_count > 80)
        {
            Gimbal.VisionAngleOffset = Gimbal.VisionMotor.total_angle;
            vision_correction_flag = 1;
            vision_init_count = 0;
            Gimbal.VisionMotor.Max_Out = VISION_MOTOR_MAXOUT;
        }
    }
}

static void Pitch_Correction(void){
    static uint8_t pitch_init_count = 0;

    if(is_TOE_Error(GIMBAL_PITCH_MOTOR_TOE) && pitch_correction_flag == 1 )
    {
        pitch_correction_flag = 0;
    }

    if(pitch_correction_flag == 0 && (!(is_TOE_Error(RC_TOE)) || !(is_TOE_Error(VTM_TOE))))
    {
        Gimbal.PitchMotor.Max_Out = 10000;
        Gimbal.PitchRefCorrectionAngularVelocity = -500;
        if ((fabsf(Gimbal.PitchMotor.Output) >= 9950)&&(fabsf(Gimbal.PitchMotor.Velocity_RPM) <= 10))
        {
            pitch_init_count ++;
        }
        if(pitch_init_count > 60)
        {
            Gimbal.PitchAngleOffset = Gimbal.PitchMotor.total_angle;
            pitch_correction_flag = 1;
            pitch_init_count = 0;
            Gimbal.PitchMotor.Max_Out = PITCH_MOTOR_MAXOUT;
        }
    }
}

static void Gimbal_Set_Mode(void) {
    static uint16_t LastKeyCode = 0;
    static uint8_t servo_flag = 0;
    ///精确模式预判断

    if (Gimbal.SetPreciseMode == 4) // 按Z切换为精确模式，或切回普通模式
    {
        Gimbal.Mode = Precise_Mode;
    }else
    {
        Gimbal.Mode = Normal_Mode;
    }

    //精确模式下按下W/A/S/D且开启电容,理解为要跑路,云台切回普通模式,底盘随动
   if (remote_control.key_code & Key_SHIFT && Gimbal.Mode == Precise_Mode &&
       ((remote_control.key_code & Key_W) || (remote_control.key_code & Key_A) || (remote_control.key_code & Key_S) ||
        (remote_control.key_code & Key_D))) {
       Gimbal.Mode = Normal_Mode; 
   }

    ///拨杆控制模式转换
    switch (remote_control.switch_right) {
        case Switch_Up:
            Gimbal.Mode = AimAssist_Mode; // 当遥控器右拨杆置上时 云台模式切换为自瞄模式
            break;
        case Switch_Middle:
            if (Gimbal.Mode != Precise_Mode)
                if (remote_control.mouse.press_right == 1)
                    Gimbal.Mode = AimAssist_Mode; // 当遥控器右拨杆置中且鼠标右键按下时且原模式不为精确模式 云台模式切换为自瞄模式
                else
                    Gimbal.Mode = Normal_Mode; // 若遥控器右拨杆置中且鼠标右键不按下且原模式不为精确模式 云台模式为普通模式
            break;
        case Switch_Down:
            Gimbal.Mode = Normal_Mode; // 当遥控器右拨杆置下时 云台模式为普通模式
            break;
    }

    Gimbal.ModeSwitchCount++; // 记录同模式持续时间
    if (Gimbal.Mode != Gimbal.ModeLast) // 当模式切换时，重置持续时间
        Gimbal.ModeSwitchCount = 0;
    LastKeyCode = remote_control.key_code; // 更新键位信息
}

static void Gimbal_Get_CtrlValue(void) {
    static float AimAssistYaw, AimAssistPitch;
    static uint16_t LastKeyCode = 0;
    static float precise_ratio = 0;
    static uint8_t vision_position_flag = 0;

    switch (Gimbal.Mode) // 不同云台模式下 选择两轴实际角度的来源 当前为相同
    {
        case AimAssist_Mode:
            Gimbal.YawAngle = INS.YawTotalAngle;
            Gimbal.PitchAngle = INS.Pitch;
            Gimbal.PitchMechAngle = Gimbal.PitchMotor.total_angle - Gimbal.PitchAngleOffset;
            Gimbal.VisionAngle = Gimbal.VisionMotor.total_angle - Gimbal.VisionAngleOffset;
            break;
        case Normal_Mode:
            Gimbal.YawAngle = INS.YawTotalAngle;
            Gimbal.PitchAngle = INS.Pitch;
            Gimbal.PitchMechAngle = Gimbal.PitchMotor.total_angle - Gimbal.PitchAngleOffset;
            Gimbal.VisionAngle = Gimbal.VisionMotor.total_angle - Gimbal.VisionAngleOffset;
            break;
        case Precise_Mode:
            Gimbal.YawAngle = Gimbal.YawMotor.AngleInDegree_LPF;
            Gimbal.PitchAngle = INS.Pitch;
            Gimbal.PitchMechAngle = Gimbal.PitchMotor.total_angle - Gimbal.PitchAngleOffset;
            Gimbal.VisionAngle = Gimbal.VisionMotor.total_angle - Gimbal.VisionAngleOffset;
            break;
    }

    // 获取两轴角速度
    Gimbal.EncoderYawAngle = Gimbal.YawMotor.AngleInDegree;
    Gimbal.EncoderPitchAngle = Gimbal.PitchMotor.AngleInDegree;
    Gimbal.YawMotor.AngleInDegree_LPF = LowPassFilter(Gimbal.YawMotor.AngleInDegree,&Gimbal.YawMotor.Lpf_AngleInDegree,dt);
    Gimbal.YawAngularVelocity = INS.Gyro[Z] * arm_cos_f32(Gimbal.PitchAngle / RADIAN_COEF) +
                                INS.Gyro[X] * arm_sin_f32(Gimbal.PitchAngle / RADIAN_COEF);
    Gimbal.PitchAngularVelocity = -INS.Gyro[Y];


    // 非精确模式下，用遥控器或鼠标信息设定两轴期望角速度
    if (Gimbal.Mode != Precise_Mode) {
        Gimbal.YawRefAngularVelocity = -remote_control.ch1 * Gimbal.rcStickYawRatio -
                                       float_constrain(remote_control.mouse.x * Gimbal.rcMouseYawRatio, -3000, 3000);
        Gimbal.PitchRefAngularVelocity = remote_control.ch2 * Gimbal.rcStickPitchRatio +
                                         (remote_control.mouse.y * Gimbal.rcMousePitchRatio);
        Gimbal.VisionRefAngle = 0;
    } else {
        // 精确模式下，用WASD信息设定两轴期望角速度
        // 若按住V，云台控制倍率切换为10倍，否则为1倍
        if (remote_control.key_code & Key_V)
            precise_ratio = 1;
        else precise_ratio = 0.5;

        Gimbal.YawRefAngularVelocity = -remote_control.ch1 * 0.05 -
                float_constrain(remote_control.mouse.x * precise_ratio, -3000, 3000);;
        Gimbal.PitchRefAngularVelocity = remote_control.ch2 * 0.01 +
                remote_control.mouse.y * precise_ratio;

        if((remote_control.key_code & Key_G) && (LastKeyCode & Key_G))
            if(!Gimbal.vision_flag){
                Gimbal.vision_flag = 1;
            }else if(Gimbal.vision_flag){
                Gimbal.vision_flag = 0;
            }
        if(!Gimbal.vision_flag){
            Gimbal.VisionRefAngle = 0;
        }else if(Gimbal.vision_flag){
            if(!Shoot.Speed_Flag){
                Gimbal.VisionRefAngle = 42000;
            }else if(Shoot.Speed_Flag){
                Gimbal.VisionRefAngle = 35000;
            }
        }

//        if(Gimbal.PitchAngle <= 36.0f)
//            Gimbal.VisionRefAngle = 0;
//        else if(Gimbal.PitchAngle > 36.0f && Gimbal.PitchAngle <= 43.0f)
//            Gimbal.VisionRefAngle = 30000;
//        if(Gimbal.PitchAngle <= 15.0f)
//            Gimbal.VisionRefAngle = 0;
//        else if(Gimbal.PitchAngle > 15.0f && Gimbal.PitchAngle <= 32.0f)
//            Gimbal.VisionRefAngle = 30000;
//        else if(Gimbal.PitchAngle > 32.0f && Gimbal.PitchAngle <= 35.0f)
//            Gimbal.VisionRefAngle = 41000;
//        else if(Gimbal.PitchAngle > 35.0f && Gimbal.PitchAngle <= 38.0f)
//            Gimbal.VisionRefAngle = 42000;
    }

    //按q开关倍镜
    if (remote_control.key_code & Key_Q && !(LastKeyCode & Key_Q)) {
        if(!Gimbal.zoom_state){
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,5000);
            Gimbal.zoom_state = 1;
        }else if(Gimbal.zoom_state){
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,1105);
            Gimbal.zoom_state = 0;
        }
    }

   if (remote_control .key_code&Key_M&&!(LastKeyCode & Key_M)) {
       Gimbal.Mode=Auxiliary_Mode;
       AimAssist.auxistatus=1;
   }else{
       Gimbal.Mode=Normal_Mode;
       AimAssist.auxistatus=0;
   }

    ///期望角度+期望角速度处理
    Gimbal.YawRefAngularVelocity = TD_Calculate(&Gimbal.YawRefAngularVelocityTD, Gimbal.YawRefAngularVelocity);
    Gimbal.PitchRefAngularVelocity = TD_Calculate(&Gimbal.PitchRefAngularVelocityTD, Gimbal.PitchRefAngularVelocity);
    Gimbal.YawRefAngle += Gimbal.YawRefAngularVelocity * dt;
    Gimbal.PitchRefAngle += Gimbal.PitchRefAngularVelocity * dt;
    Gimbal.PitchRefMechAngle = 4096.0 * (OPPSITE_SIDE - sqrtf(fabs(FIRST_ADJACENT_SIDE*FIRST_ADJACENT_SIDE + SECOND_ADJACENT_SIDE*SECOND_ADJACENT_SIDE
            - 2*FIRST_ADJACENT_SIDE*SECOND_ADJACENT_SIDE * cosf(PI/180.f*(Gimbal.PitchRefAngle - ANGLE_CORRECTION))))) - Gimbal.PitchZeroOffset;

    Gimbal.VisionCtrlAngle = TD_Calculate(&Gimbal.VisionRefAngleTD,Gimbal.VisionRefAngle);

    // 自瞄模式
    if (Gimbal.Mode == AimAssist_Mode && AimAssist.Status != TgtLost) {
        // if (TgtPosPredict.isSpinning)
        // {
        //     // 认为敌方为陀螺 进入打陀螺模式
        //     AimAssistYaw = HitSpinning.SpinningYawPosition;
        //     AimAssistPitch = HitSpinning.SpinningPitchPosition;
        //     if (TgtPosPredict.SpinningTgtValid)
        //     {
        //         if (AimAssistYaw - INS.Yaw < -180.0f)
        //             Gimbal.YawRefAngle = AimAssistYaw + INS.YawTotalAngle - INS.Yaw + 360.0f;
        //         else if (AimAssistYaw - INS.Yaw > 180.0f)
        //             Gimbal.YawRefAngle = AimAssistYaw + INS.YawTotalAngle - INS.Yaw - 360.0f;
        //         else
        //             Gimbal.YawRefAngle = AimAssistYaw + INS.YawTotalAngle - INS.Yaw;
        //         Gimbal.PitchRefAngle = AimAssistPitch;

        //         Gimbal.YawRefAngle = float_constrain(Gimbal.YawRefAngle, Gimbal.YawAngle - 10.5f, Gimbal.YawAngle + 10.5f);
        //         Gimbal.PitchRefAngle = float_constrain(Gimbal.PitchRefAngle, Gimbal.PitchAngle - 10.5f, Gimbal.PitchAngle + 10.5f);

        //         Gimbal.YawRefAngleTD.x = Gimbal.YawRefAngle;
        //         Gimbal.PitchRefAngleTD.x = Gimbal.PitchRefAngle;
        //         TgtPosPredict.SpinningTgtValid = 0;
        //     }
        // }
        // else
        {
            // 一般自瞄模式
            AimAssist.YawPosition_lpf = AimAssist.YawPosition * dt / (dt + AimAssist.YawPositon_lpf_coef) +
                                        AimAssist.YawPosition_lpf * AimAssist.YawPositon_lpf_coef /
                                        (dt + AimAssist.YawPositon_lpf_coef);
            AimAssist.PitchPosition_lpf = AimAssist.PitchPosition * dt / (dt + AimAssist.PitchPosition_lpf_coef) +
                                          AimAssist.PitchPosition_lpf * AimAssist.PitchPosition_lpf_coef /
                                          (dt + AimAssist.PitchPosition_lpf_coef);
            AimAssistYaw = AimAssist.YawPosition_lpf;
            AimAssistPitch = AimAssist.PitchPosition_lpf;

            if (AimAssist.Trackingcount > 100) {
                if (AimAssistYaw - INS.Yaw < -180.0f)
                    Gimbal.YawRefAngle = AimAssistYaw + INS.YawTotalAngle - INS.Yaw + 360.0f;
                else if (AimAssistYaw - INS.Yaw > 180.0f)
                    Gimbal.YawRefAngle = AimAssistYaw + INS.YawTotalAngle - INS.Yaw - 360.0f;
                else
                    Gimbal.YawRefAngle = AimAssistYaw + INS.YawTotalAngle - INS.Yaw;

                Gimbal.PitchRefAngle = AimAssistPitch;

                //Gimbal.YawRefAngle = float_constrain(Gimbal.YawRefAngle, Gimbal.YawAngle - 10.5f, Gimbal.YawAngle + 10.5f);
                Gimbal.PitchRefAngle = float_constrain(Gimbal.PitchRefAngle,
                                                       Gimbal.PitchAngle - 10.5f,
                                                       Gimbal.PitchAngle + 10.5f);
            }
         /*   if (Gimbal.Mode == Auxiliary_Mode && AimAssist.auxistatus != TgtLost) {


            }*/
        }
    }

    //过洞一键平头
    if(remote_control.key_code & Key_C)
    {
        Gimbal.PitchRefAngle = Gimbal.DepressionIMU;
    }
    if(remote_control.key_code & Key_V)
    {
        Gimbal.PitchRefAngle = 33.1f;
    }

    // 进入debug模式以控制云台周期性运动校准姿态解算
    if (GlobalDebugMode == GIMBAL_DEBUG)
        Gimbal_Tuning();
    // 确定最终控制角度期望
    Gimbal.YawCtrlAngle = TD_Calculate(&Gimbal.YawRefAngleTD, Gimbal.YawRefAngle);
    Gimbal.PitchCtrlAngle = TD_Calculate(&Gimbal.PitchRefAngleTD, Gimbal.PitchRefAngle);
    Gimbal.PitchCtrlMechAngle = TD_Calculate(&Gimbal.PitchRefMechAngleTD, Gimbal.PitchRefMechAngle);
   
    if(Gimbal.Mode == AimAssist_Mode)
    {
        Gimbal.PitchError = (Gimbal.PitchCtrlAngle - Gimbal.PitchAngle)/180*PI;
        Gimbal.YawError = (Gimbal.YawCtrlAngle - Gimbal.YawAngle)/180*PI;
    }
    // 若操作手按下B键，yaw轴快速转头180度
    // 注意，若为精确模式下，使用B键应无效
    if (remote_control.key_code & Key_B && !(LastKeyCode & Key_B) && (Gimbal.Mode != Precise_Mode)) {
        Gimbal.YawRefAngle += 180.0f;
        Gimbal.YawCtrlAngle += 180.0f;
        Gimbal.YawRefAngleTD.x += 180.0f;
    }

    // 如果遥控器未开启或因被敌方击杀等原因导致yaw pitch电机离线，将期望设置为当前实际角度
    // 以防止离线期间由于操作手乱动鼠标导致期望飘移，在上电一刹那代码跑飞从而云台“疯头”
    if (((is_TOE_Error(RC_TOE) && is_TOE_Error(VTM_TOE) && GlobalDebugMode != GIMBAL_DEBUG) ||
         Gimbal.Mode != Gimbal.ModeLast) || (is_TOE_Error(GIMBAL_YAW_MOTOR_TOE)) || (is_TOE_Error(
            GIMBAL_PITCH_MOTOR_TOE)) || ((remote_control.key_code & Key_E) && (remote_control.key_code & Key_R))) {
        Gimbal.YawCtrlAngle = Gimbal.YawAngle;
        Gimbal.PitchCtrlAngle = Gimbal.PitchAngle;
        Gimbal.PitchCtrlMechAngle = Gimbal.PitchMechAngle;

        Gimbal.YawRefAngle = Gimbal.YawAngle;
        Gimbal.PitchRefAngle = Gimbal.PitchAngle;
        Gimbal.PitchRefMechAngle = Gimbal.PitchMechAngle;

        Gimbal.YawRefAngleTD.x = Gimbal.YawAngle;
        Gimbal.YawRefAngleTD.dx = 0;
        Gimbal.YawRefAngularVelocityTD.x = Gimbal.YawAngularVelocity;
        Gimbal.YawRefAngularVelocityTD.dx = 0;
        Gimbal.PitchRefAngleTD.x = Gimbal.PitchAngle;
        Gimbal.PitchRefAngleTD.dx = 0;
        Gimbal.PitchRefAngularVelocityTD.x = Gimbal.PitchAngularVelocity;
        Gimbal.PitchRefAngularVelocityTD.dx = 0;
        Gimbal.PitchRefMechAngleTD.x = Gimbal.PitchMechAngle;
        Gimbal.PitchRefMechAngleTD.dx = 0;

        Gimbal.YawMotor.PID_Velocity.Iout = 0;
        Gimbal.PitchMotor.PID_Velocity.Iout = 0;

        Gimbal.YawMotor.FFC_Angle.Last_Ref = Gimbal.YawCtrlAngle;
        Gimbal.PitchMotor.FFC_Angle.Last_Ref = Gimbal.PitchCtrlAngle;


    }
    // 云台角度期望限幅函数
    Gimbal_CtrlValue_Limit();
    // 为新时间周期做准备，更新按键与云台模式
    LastKeyCode = remote_control.key_code;
    Gimbal.ModeLast = Gimbal.Mode;
}

static void Gimbal_CtrlValue_Limit(void) {
    static uint8_t LastERROR_Type;
    static float YawAngle_precise;

    Gimbal.PitchRefAngle = float_constrain(Gimbal.PitchRefAngle, Gimbal.DepressionIMU,
                                           Gimbal.ElevationIMU);
    Gimbal.PitchCtrlAngle = float_constrain(Gimbal.PitchRefAngle, Gimbal.DepressionIMU,
                                            Gimbal.ElevationIMU);
    Gimbal.PitchRefMechAngle = float_constrain(Gimbal.PitchRefMechAngle, Gimbal.DepressionMechAngle,
                                               Gimbal.ElevationMechAngle);
    Gimbal.PitchCtrlMechAngle = float_constrain(Gimbal.PitchRefMechAngle, Gimbal.DepressionMechAngle,
                                                Gimbal.ElevationMechAngle);

    // 前馈输出限幅
    if (fabs(Gimbal.PitchCtrlAngle - Gimbal.DepressionIMU) < 0.5f || fabs(Gimbal.ElevationIMU - Gimbal.PitchCtrlAngle) < 0.5f ||
        t < 1) {
        Gimbal.PitchMotor.FFC_Angle.MaxOut = 0;
        Gimbal.PitchMotor.FFC_Velocity.MaxOut = 0;
        Gimbal.PitchMotor.FFC_Torque.MaxOut = 0;
    }

    /*if (GlobalDebugMode != GIMBAL_DEBUG && GimbalTuningEnable == 0) {
        switch (Gimbal.Mode) // 根据云台模式选择PID参数
        {
            case AimAssist_Mode:
                if (Gimbal.ModeSwitchCount > 10) {
                    Gimbal.YawMotor.PID_Angle.Kp = YAW_A_PID_KP_AIMASSIST;
                    Gimbal.YawMotor.PID_Angle.Ki = YAW_A_PID_KI_AIMASSIST;
                    Gimbal.YawMotor.PID_Angle.Kd = YAW_A_PID_KD_AIMASSIST;
                    Gimbal.YawMotor.PID_Velocity.Kp = YAW_V_PID_KP_AIMASSIST;
                    Gimbal.YawMotor.PID_Velocity.Ki = YAW_V_PID_KI_AIMASSIST;
                    Gimbal.YawMotor.PID_Velocity.Kd = YAW_V_PID_KD_AIMASSIST;
                    Gimbal.YawMotor.PID_Angle.Derivative_LPF_RC = YAW_A_PID_D_LPF;
                    if (AimAssist.Mode == HIT_RUNE_MIN || AimAssist.Mode == HIT_RUNE_MAX) {
                        Gimbal.YawMotor.FFC_Velocity.c[1] = YAW_V_FCC_C1;
                        Gimbal.PitchMotor.FFC_Velocity.c[1] = PITCH_V_FCC_C1;
                        Gimbal.PitchMotor.FFC_Angle.MaxOut = PITCH_A_FFC_MAXOUT;
                    } else {
                        Gimbal.YawMotor.FFC_Velocity.c[1] = 0;
                        Gimbal.PitchMotor.FFC_Velocity.c[1] = 0;
                        Gimbal.PitchMotor.FFC_Angle.MaxOut = 0;
                    }
                    Gimbal.PitchMotor.PID_Angle.Kp = PITCH_A_PID_KP_AIMASSIST;
                    Gimbal.PitchMotor.PID_Angle.Ki = PITCH_A_PID_KI_AIMASSIST;
                    Gimbal.PitchMotor.PID_Angle.Kd = PITCH_A_PID_KD_AIMASSIST;
                    Gimbal.PitchMotor.PID_Velocity.Kp = PITCH_V_PID_KP_AIMASSIST;
                    Gimbal.PitchMotor.PID_Velocity.Ki = PITCH_V_PID_KI_AIMASSIST;
                    Gimbal.PitchMotor.PID_Velocity.Kd = PITCH_V_PID_KD_AIMASSIST;
                }
                break;
            case Normal_Mode:
                if (Gimbal.ModeSwitchCount > 10) {
                    Gimbal.YawMotor.PID_Angle.Kp = YAW_A_PID_KP;
                    Gimbal.YawMotor.PID_Angle.Ki = YAW_A_PID_KI;
                    Gimbal.YawMotor.PID_Angle.Kd = YAW_A_PID_KD;
                    Gimbal.YawMotor.PID_Velocity.Kp = YAW_V_PID_KP;
                    Gimbal.YawMotor.PID_Velocity.Ki = YAW_V_PID_KI;
                    Gimbal.YawMotor.PID_Velocity.Kd = YAW_V_PID_KD;
                    Gimbal.YawMotor.PID_Angle.Derivative_LPF_RC = YAW_A_PID_D_LPF;
                    Gimbal.YawMotor.FFC_Velocity.c[1] = YAW_V_FCC_C1;
                    Gimbal.PitchMotor.PID_Angle.Kp = PITCH_A_PID_KP;
                    Gimbal.PitchMotor.PID_Angle.Ki = PITCH_A_PID_KI;
                    Gimbal.PitchMotor.PID_Angle.Kd = PITCH_A_PID_KD;
                    Gimbal.PitchMotor.PID_Velocity.Kp = PITCH_V_PID_KP;
                    Gimbal.PitchMotor.PID_Velocity.Ki = PITCH_V_PID_KI;
                    Gimbal.PitchMotor.PID_Velocity.Kd = PITCH_V_PID_KD;
                    Gimbal.PitchMotor.FFC_Velocity.c[1] = PITCH_V_FCC_C1;
                    Gimbal.PitchMotor.FFC_Angle.MaxOut = PITCH_A_FFC_MAXOUT;
                }
                break;
            case Precise_Mode:
                if (Gimbal.ModeSwitchCount > 10) {
                    Gimbal.YawMotor.PID_Angle.Kp = YAW_A_PID_KP_PRECISE;
                    Gimbal.YawMotor.PID_Angle.Ki = YAW_A_PID_KI_PRECISE;
                    Gimbal.YawMotor.PID_Angle.Kd = YAW_A_PID_KD_PRECISE;
                    Gimbal.YawMotor.PID_Velocity.Kp = YAW_V_PID_KP_PRECISE;
                    Gimbal.YawMotor.PID_Velocity.Ki = YAW_V_PID_KI_PRECISE;
                    Gimbal.YawMotor.PID_Velocity.Kd = YAW_V_PID_KD_PRECISE;
                    Gimbal.YawMotor.PID_Angle.Derivative_LPF_RC = YAW_A_PID_D_LPF;

                    Gimbal.PitchMotor.PID_Angle.Kp = PITCH_A_PID_KP_PRECISE;
                    Gimbal.PitchMotor.PID_Angle.Ki = PITCH_A_PID_KI_PRECISE;
                    Gimbal.PitchMotor.PID_Angle.Kd = PITCH_A_PID_KD_PRECISE;
                    Gimbal.PitchMotor.PID_Velocity.Kp = PITCH_V_PID_KP_PRECISE;
                    Gimbal.PitchMotor.PID_Velocity.Ki = PITCH_V_PID_KI_PRECISE;
                    Gimbal.PitchMotor.PID_Velocity.Kd = PITCH_V_PID_KD_PRECISE;
                }
                break;
        }
    }*/
}

static void Gimbal_Set_Control(void) {
    static float compCoef = 0, gravityTorque;
    static float YawVelocityLoopInput;
    static float PitchAngleLoopInput;
    static float PitchVelocityLoopInput;
    /********************** Yaw Calculate **********************/
    // 角度环反馈控制
    PID_Calculate(&Gimbal.YawMotor.PID_Angle, Gimbal.YawAngle, Gimbal.YawCtrlAngle);
    // 角度环前馈控制
    Gimbal.YawMotor.FFC_Angle.Output = float_constrain(Gimbal.YawMotor.FFC_Angle.c[1] * Gimbal.YawRefAngleTD.dx,
                                                       -Gimbal.YawMotor.FFC_Angle.MaxOut,
                                                       Gimbal.YawMotor.FFC_Angle.MaxOut);
    YawVelocityLoopInput = float_constrain(Gimbal.YawMotor.PID_Angle.Output + Gimbal.YawMotor.FFC_Angle.Output,
                                           -Gimbal.YawMotor.PID_Angle.MaxOut, Gimbal.YawMotor.PID_Angle.MaxOut);
    if(Gimbal.Mode != AimAssist_Mode)
    {
        YawVelocityLoopInput = float_constrain(Gimbal.YawMotor.PID_Angle.Output + Gimbal.YawMotor.FFC_Angle.Output,
                                               -Gimbal.YawMotor.PID_Angle.MaxOut, Gimbal.YawMotor.PID_Angle.MaxOut);
    }else
    {
        YawVelocityLoopInput = float_constrain(Gimbal.YawMotor.PID_Angle.Output + Gimbal.YawMotor.FFC_Angle.Output + ShootEvaluation.YawSpeed,
                                               -Gimbal.YawMotor.PID_Angle.MaxOut, Gimbal.YawMotor.PID_Angle.MaxOut);
    }
    // 速度环反馈控制
    PID_Calculate(&Gimbal.YawMotor.PID_Velocity, Gimbal.YawAngularVelocity, YawVelocityLoopInput);
    // 速度环前馈控制
    // Gimbal.YawMotor.FFC_Velocity.Output = float_constrain(Gimbal.YawMotor.FFC_Velocity.c[1] * Gimbal.YawRefAngleTD.ddx +
    //                                                           Gimbal.YawMotor.FFC_Velocity.c[0] * Gimbal.YawRefAngleTD.dx,
    //                                                       -Gimbal.YawMotor.FFC_Velocity.MaxOut, Gimbal.YawMotor.FFC_Velocity.MaxOut);
    Feedforward_Calculate(&Gimbal.YawMotor.FFC_Velocity, Gimbal.YawRefAngleTD.dx);

    Gimbal.YawMotor.Output = float_constrain(Gimbal.YawMotor.PID_Velocity.Output + Gimbal.YawMotor.FFC_Velocity.Output,
                                             -Gimbal.YawMotor.Max_Out, Gimbal.YawMotor.Max_Out);

    /********************* Pitch Calculate *********************/
    gravityTorque = arm_cos_f32((INS.Pitch + 30) / RADIAN_COEF) * compCoef;
    if(pitch_correction_flag == 1)
    {
        if(Gimbal.Mode != Precise_Mode)
        {
            //角度环反馈控制
            PID_Calculate(&Gimbal.PitchMotor.PID_Angle, Gimbal.PitchAngle, Gimbal.PitchCtrlAngle);
            PitchAngleLoopInput = float_constrain(Gimbal.PitchMotor.PID_Angle.Output + Gimbal.PitchCtrlMechAngle,
                                                  Gimbal.DepressionMechAngle, Gimbal.ElevationMechAngle);

            PID_Calculate(&Gimbal.PitchMotor.PID_MechAngle, Gimbal.PitchMechAngle, PitchAngleLoopInput);
            PitchVelocityLoopInput = float_constrain(
                    Gimbal.PitchMotor.PID_MechAngle.Output,-Gimbal.PitchMotor.PID_MechAngle.MaxOut,Gimbal.PitchMotor.PID_MechAngle.MaxOut);

            // 速度环反馈控制
            PID_Calculate(&Gimbal.PitchMotor.PID_Velocity, Gimbal.PitchMotor.Velocity_RPM, PitchVelocityLoopInput);
            // 速度环前馈控制
            Feedforward_Calculate(&Gimbal.PitchMotor.FFC_Velocity, Gimbal.PitchRefAngleTD.dx);

            Gimbal.PitchMotor.Output = float_constrain(Gimbal.PitchMotor.PID_Velocity.Output /*+ Gimbal.PitchMotor.FFC_Velocity.Output*/,
                                                       -Gimbal.PitchMotor.Max_Out, Gimbal.PitchMotor.Max_Out);
        }else
        {
            PID_Calculate(&Gimbal.PitchMotor.PID_MechAngle, Gimbal.PitchMechAngle, Gimbal.PitchCtrlMechAngle);
            PitchVelocityLoopInput = float_constrain(
                    Gimbal.PitchMotor.PID_MechAngle.Output,-Gimbal.PitchMotor.PID_MechAngle.MaxOut,Gimbal.PitchMotor.PID_MechAngle.MaxOut);

            // 速度环反馈控制
            PID_Calculate(&Gimbal.PitchMotor.PID_Velocity, Gimbal.PitchMotor.Velocity_RPM, PitchVelocityLoopInput);
            // 速度环前馈控制
            Feedforward_Calculate(&Gimbal.PitchMotor.FFC_Velocity, Gimbal.PitchRefAngleTD.dx);

            Gimbal.PitchMotor.Output = float_constrain(Gimbal.PitchMotor.PID_Velocity.Output + Gimbal.PitchMotor.FFC_Velocity.Output,
                                                       -Gimbal.PitchMotor.Max_Out, Gimbal.PitchMotor.Max_Out);
        }
    }
     else if(pitch_correction_flag == 0)
     {
         PID_Calculate(&Gimbal.PitchMotor.PID_Velocity, Gimbal.PitchMotor.Velocity_RPM, Gimbal.PitchRefCorrectionAngularVelocity);
         Gimbal.PitchMotor.Output = float_constrain(Gimbal.PitchMotor.PID_Velocity.Output, -Gimbal.PitchMotor.Max_Out, Gimbal.PitchMotor.Max_Out);
     }
    /********************* Vision Calculate *********************/
    if(vision_correction_flag == 1)
    {
        PID_Calculate(&Gimbal.VisionMotor.PID_Angle, Gimbal.VisionAngle, Gimbal.VisionCtrlAngle);
        PID_Calculate(&Gimbal.VisionMotor.PID_Velocity, Gimbal.VisionMotor.Velocity_RPM, Gimbal.VisionMotor.PID_Angle.Output);
        Gimbal.VisionMotor.Output = float_constrain(Gimbal.VisionMotor.PID_Velocity.Output,-Gimbal.VisionMotor.Max_Out,Gimbal.VisionMotor.Max_Out);
    }
    else if(vision_correction_flag == 0)
    {
        PID_Calculate(&Gimbal.VisionMotor.PID_Velocity, Gimbal.VisionMotor.Velocity_RPM, Gimbal.VisionRefAngularVelocity);
        Gimbal.VisionMotor.Output = float_constrain(Gimbal.VisionMotor.PID_Velocity.Output,-Gimbal.VisionMotor.Max_Out,Gimbal.VisionMotor.Max_Out);
    }
}

static void Send_Gimbal_Current(void) {
    // 若遥控器未开启或R键E键一起按，将电机电流给0
    if ((is_TOE_Error(RC_TOE) && is_TOE_Error(VTM_TOE) && GlobalDebugMode != GIMBAL_DEBUG) ||
        ((remote_control.key_code & Key_R) && (remote_control.key_code & Key_E)))
    {
        Send_Motor_Current_5_8(&hcan2, 0, 0, 0, 0);
        Send_Motor_Current_1_4(&hcan1, 0, 0, 0, 0);
        if ((Send_Motor_Current_1_4(&hcan1, 0, 0, 0, 0) == HAL_OK)
            && (Send_Motor_Current_DMYaw(&hcan1, 0, 0, 0, 0, 0) == HAL_OK))
            HAL_IWDG_Refresh(&hiwdg); // 喂狗 非常重要！
        else
            CAN_SEND_ERROR_COUNT++;
    }
        //Pitch堵转保护
    else if (PitchLock == 1 && PitchReback <= 1000) {
        Send_Motor_Current_1_4(&hcan1, 0, 0, 0, 0);
        Send_Motor_Current_DMYaw(&hcan1,0, 0, 0, 0, 0);
        PitchReback++;
    } else {
        // 发送云台电机电流
        PitchReback = 0; //Pitch恢复计数清零
        PitchLockCount = 0;  //Pitch堵转计数清零
        if ((Send_Motor_Current_1_4(&hcan1, Gimbal.PitchMotor.Output, Shoot.TriggerMotor.Output, 0, 0) == HAL_OK)
            && (Send_Motor_Current_DMYaw(&hcan1, 0, 0, 0, 0, Gimbal.YawMotor.Output) == HAL_OK)
            && (Send_Motor_Current_5_8(&hcan2, Gimbal.VisionMotor.Output, 0, 0, 0)) == HAL_OK)
        {
            HAL_IWDG_Refresh(&hiwdg); // 喂狗 非常重要！
        }

        else
            CAN_SEND_ERROR_COUNT++;
    }
}

static void GimbalSI_Calculate(void) {
    if (GimbalSI.ResetFlag) {
        GimbalSI.ResetFlag = 0;

        for (uint8_t i = 0; i < 3; i++) {
            GimbalSI.YawSI.SI_EKF.xhat_data[i] = 0;
            GimbalSI.PitchSI.SI_EKF.xhat_data[i] = 0;
        }

        GimbalSI.YawSI.SI_EKF.P_data[0] = 10000;
        GimbalSI.YawSI.SI_EKF.P_data[4] = 10000000;
        GimbalSI.YawSI.SI_EKF.P_data[8] = 10000000;
        GimbalSI.PitchSI.SI_EKF.P_data[0] = 10000;
        GimbalSI.PitchSI.SI_EKF.P_data[4] = 10000000;
        GimbalSI.PitchSI.SI_EKF.P_data[8] = 10000000;
    }
    FirstOrderSI_EKF_Tuning(&GimbalSI.YawSI, GimbalSI.Q0, GimbalSI.Q1, GimbalSI.Q2, GimbalSI.R, GimbalSI.lambda);
    FirstOrderSI_EKF_Tuning(&GimbalSI.PitchSI, GimbalSI.Q0, GimbalSI.Q1, GimbalSI.Q2, GimbalSI.R, GimbalSI.lambda);

    FirstOrderSI_Update(&GimbalSI.YawSI, Gimbal.YawMotor.PID_Torque.Ref, Gimbal.YawAngularVelocity, dt);
    FirstOrderSI_Update(&GimbalSI.PitchSI, Gimbal.PitchMotor.PID_Torque.Ref, Gimbal.PitchAngularVelocity, dt);
    // FirstOrderSI_Update(&GimbalSI.YawSI, Gimbal.YawMotor.Real_Current, Gimbal.YawAngularVelocity, dt);
    // FirstOrderSI_Update(&GimbalSI.PitchSI, Gimbal.PitchMotor.Real_Current, Gimbal.PitchAngularVelocity, dt);
}

static void Gimbal_Tuning(void)   //云台调参：使用通过此函数输入激励信号，后通过系统辨识算法算出系统闭环传递函数，获取相应PID参数
{
    static uint32_t Tuning_Cnt = 0;
    if (is_TOE_Error(RC_TOE) && is_TOE_Error(VTM_TOE)) {
        Gimbal.YawRefAngle = YawAmp* sinf(YawOmega* t) +YawOffset;
//        if(Tuning_Cnt++ > 2000){
//            Gimbal.YawRefAngle = YawAmp * (-1);
//            Tuning_Cnt = 0;
//        }
    } else {
        YawAmp = 0;
        YawOmega = 0;
        YawOffset = Gimbal.YawAngle;
        PitchAmp = 0;
        PitchOmega = 0;
        PitchOffset = Gimbal.PitchAngle;
    }
    if (debugMode == 0)
        Serial_Debug(&huart1,
                     2,
                     amp[0] * Gimbal.PitchMotor.PID_Torque.Ref / 100,
                     amp[1] * Gimbal.PitchMotor.PID_Torque.Measure / 100,
                     amp[2] * Gimbal.PitchMotor.PID_Torque.Output * 100,
                     amp[3] * Gimbal.PitchMotor.FFC_Torque.Output * 100,
                     amp[4] * Gimbal.PitchMotor.PID_Torque.Pout * 100,
                     amp[5] * Gimbal.PitchMotor.PID_Torque.Iout * 100);
    if (debugMode == 1)
        Serial_Debug(&huart1,
                     1,
                     amp[0] * Gimbal.PitchMotor.PID_Velocity.Ref,
                     amp[1] * Gimbal.PitchMotor.PID_Velocity.Measure,
                     amp[2] * Gimbal.PitchMotor.PID_Velocity.Output,
                     amp[3] * Gimbal.PitchMotor.FFC_Velocity.Output,
                     amp[4] * Gimbal.PitchMotor.LDOB.Output,
                     amp[5] * PitchDisturbance);
    if (debugMode == 2)
        Serial_Debug(&huart1, 1, amp[0] * Gimbal.PitchMotor.PID_Angle.Ref, amp[1] * Gimbal.PitchMotor.PID_Angle.Measure,
                     amp[2] * Gimbal.PitchMotor.PID_Angle.Output * 10, amp[3] * Gimbal.PitchMotor.FFC_Angle.Output * 10,
                     amp[4] * Gimbal.PitchMotor.PID_Angle.Pout * 10, amp[5] * Gimbal.PitchMotor.PID_Angle.Iout * 10);
    if (debugMode == 3)
        Serial_Debug(&huart1,
                     2,
                     amp[0] * Gimbal.YawMotor.PID_Torque.Ref / 100,
                     amp[1] * Gimbal.YawMotor.PID_Torque.Measure / 100,
                     amp[2] * Gimbal.YawMotor.PID_Torque.Output * 100,
                     amp[3] * Gimbal.YawMotor.FFC_Torque.Output * 100,
                     amp[4] * Gimbal.YawMotor.PID_Torque.Pout * 100,
                     amp[5] * Gimbal.YawMotor.PID_Torque.Iout * 100);
    if (debugMode == 4)
        Serial_Debug(&huart1,
                     1,
                     amp[0] * Gimbal.YawMotor.PID_Velocity.Ref,
                     amp[1] * Gimbal.YawMotor.PID_Velocity.Measure,
                     amp[2] * Gimbal.YawMotor.PID_Velocity.Output,
                     amp[3] * Gimbal.YawMotor.FFC_Velocity.Output,
                     amp[4] * Gimbal.YawMotor.LDOB.Output,
                     amp[5] * YawDisturbance);
    if (debugMode == 5)
        Serial_Debug(&huart1, 1, amp[0] * Gimbal.YawMotor.PID_Angle.Ref, amp[1] * Gimbal.YawMotor.PID_Angle.Measure,
                     amp[2] * Gimbal.YawMotor.PID_Angle.Output * 10, amp[3] * Gimbal.YawMotor.FFC_Angle.Output * 10,
                     amp[4] * Gimbal.YawMotor.PID_Angle.Pout * 10, amp[5] * Gimbal.YawMotor.PID_Angle.Iout * 10);
    if (debugMode == 6)
        Serial_Debug(&huart1, 1, TgtPosPredict.YawPosition, TgtPosPredict.PitchPosition,
                     Gimbal.YawCtrlAngle, Gimbal.PitchCtrlAngle,
                     0, 0);
    if (debugMode == 7)
        Serial_Debug(&huart1,
                     2,
                     Gimbal.YawAngularVelocity * 60 / 2 / 3.1415926f,
                     -Gimbal.YawAngularVelocity * 60 / 2 / 3.1415926f,
                     Gimbal.YawMotor.para.vel,
                     0,
                     0,
                     0);
}

static void Gimbal_Data_PackUp(void) {
    float gimbal_pitch_f = Gimbal.PitchAngle * 1000;
    uint16_t gimbal_pitch_u16 = (int16_t)(gimbal_pitch_f);
    Gimbal_Data_Buffer[0] = AimAssist.Status;
    Gimbal_Data_Buffer[1] = AimAssist.miniPC_Online;
    Gimbal_Data_Buffer[2] = Gimbal.Mode;
    Gimbal_Data_Buffer[3] = Shoot.Shoot_Motor_State;
    Gimbal_Data_Buffer[4] =  (gimbal_pitch_u16 >> 8) & 0xFF;
    Gimbal_Data_Buffer[5] =  gimbal_pitch_u16 & 0xFF;
    Gimbal_Data_Buffer[6] = Shoot.Speed_Flag;
    Gimbal_Data_Buffer[7] = 0;
}