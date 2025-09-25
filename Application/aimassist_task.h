/**
 ******************************************************************************
 * @file	aimassist_task.c
 * @author  Wang Hongxi
 * @version v2.5.3
 * @date    2022/4/2
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef _AIMASSIST_H
#define _AIMASSIST_H

#include "ins_task.h"
#include "gimbal_task.h"
#include "kalman_filter.h"
#include "remote_control.h"

#define ENABLE_CALIBRATION 0

#define AUTOAIM_TASK_PERIOD 1
#define CALIBRATION_RX_BUF_NUM 28
#define AimAssist_RX_BUF_NUM 18
#define AimAssist_TX_BUF_NUM 18
#define AIM_VERSION AIM_2024
#define AIM_2024 0
#define AIM_PAST 1

#define PITCH_MAX_DEG 15
#define YAW_MAX_DEG 20

#define X 0
#define Y 1
#define Z 2

typedef struct
{
    uint8_t CF_SOF;
    uint8_t flg;
    int16_t x; // mm
    uint16_t y;
    int16_t z;
    float rVecx; // mm
    float rVecy;
    float rVecz;
    uint32_t time_stamp_ms;
    uint8_t CF_EOF;
} CalibrationFrame_t;

/**
 * @Brief: TX2����ս��֡�ṹ��
 */
typedef struct _controlFrame
{
    uint8_t CF_SOF;
    uint8_t flg;
    int16_t x; // mm
    uint16_t y;
    int16_t z;
    uint32_t time_stamp_ms;
    short reserve1;   // �������ȱ�
    uint8_t reserve2; // װ�װ�����
    uint8_t CF_EOF;
} ControlFrame;

typedef struct _controlFrameFull
{
    uint8_t CF_SOF;         //
    uint8_t flg;            // װ�װ�ID
    int16_t px;             // Ŀ�����ϵ���� x
    uint16_t py;            // Ŀ�����ϵ���� y
    int16_t pz;             // Ŀ�����ϵ���� z
    int16_t rx;             // Ŀ�����ϵ������ x
    int16_t ry;             // Ŀ�����ϵ������ y
    int16_t rz;             // Ŀ�����ϵ������ z
    uint16_t time_stamp_ms; // ms*10
    uint8_t type;           // ��/Сװ��
    uint8_t CF_EOF;         // 15
} ControlFrameFull;

/**
 * @Brief: ս���ش�����֡�ṹ��
 */
typedef struct _feedBackFrame
{
    uint8_t FDBF_SOF;
    uint8_t myteam; //  EE DD
    short pitch;    //  10k*rad
    short yaw;
    uint16_t bullet_speed;  // m/s
    uint16_t time_stamp_ms; // ms
    short outpost_alive;
    short status;
    short reserve2;
    uint8_t mode;
    uint8_t FDBF_EOF;
} FeedBackFrame;

enum ObjectType
{
    UNKNOWN_ARMOR = 0,
    SMALL_ARMOR = 1,
    BIG_ARMOR = 2,
    RUNE_WING = 3,
    RUNE_CENTRE = 4
};

enum miniPC_Mode
{
    SUSPEND = 0,
    AUTO_AIM,
    HIT_RUNE_MIN,
    HIT_RUNE_MAX,
    NTP_CALIBRATE
};

enum ColorChannels
{
    INVALID_COLOR = 0,
    BLUE = 1,
    RED = 2
};

enum
{
    NONE_TEAM   = 0,
    RED_TEAM    = 0x1F,
    BLUE_TEAM   = 0xF1,
};

enum NTP_MSG
{
    PC2STM32_T1 = 1,
    STM2PC_T3,
    PC2STM32_T4,
    NTP_SUCCESS,
    NTP_FAIL,
    NTP_START
};

enum TargetStatus
{
    TargetLost = 0,
    TargetValid,
};

enum TgtPosPredictStatus
{
    TgtLost = 0,
    TgtTracking,
    TgtSwitch,
    TgtConjecture,
};

typedef struct
{
    float T1;
    float T2;
    float T3;
    float T4;
    float deltaT;
    float TransmitDelay;
} NTP_t;

typedef struct
{
    uint8_t Status;
    uint8_t Mode;
    uint8_t LastMode;
    uint8_t miniPC_Online;
    uint8_t PerspectiveEnable;
    ControlFrame CtrlFrame;
    ControlFrameFull CtrlFrameFull;
    FeedBackFrame FdbFrame;
    uint32_t FrameTimeStamp;
    uint8_t dataValid;
    uint8_t FdbFlag;
    uint8_t Team;
    uint8_t Target_label;
    uint8_t auxistatus;

    float TargetBodyFrame[3];
    float TargetEarthFrame[3];
    uint8_t TargetID;

    float YawPosition;
    float PitchPosition;
    float Distance;
    float YawPosition_lpf;
    float PitchPosition_lpf;
    float YawPositon_lpf_coef;
    float PitchPosition_lpf_coef;
    uint32_t Trackingcount;
    uint16_t AutoAimDebugShow; // 自瞄调试信息
    float TimeConsuming;
    float Frame_dt;
    float FrameDelayToINS;
    float DelayTime;
    uint32_t UpdatePeriodCount;

    NTP_t NTP;

    UART_HandleTypeDef *AA_USART;
} AimAssist_t;

enum
{
    Clockwise,
    Counterclockwise
};

typedef struct
{
    uint8_t Direction;
    uint32_t deltaT;
    float ArmorDistance;
    float CrossProductZ;
    float Velocity;
    float VelocityX;
    float VelocityY;
} SpinningVelocityPriori_t;

enum
{
    NormalSpeedSpinning,
    HighSpeedSpinning
};

typedef struct
{
    SpinningVelocityPriori_t VelocityPriori;

    uint8_t UseSpinningTgtPredict;
    uint8_t UseSpinningAccelPredict;
    uint8_t UseCenterPredict;
    uint8_t UseVelocityPriori;
    uint8_t SpinningCountThreshold;
    uint8_t SpinningCount;
    uint8_t SpinningSpeedStatus;
    float SpinningThresholdScale;
    float VelocityThreshold;

    float MixCoef;

    uint32_t PresentTime;
    float PresentPosition[3];
    float DebugPresentYaw;
    uint32_t PreviousTime;
    float PreviousPosition[3];
    uint32_t LastPresentTime;
    float LastPresentPosition[3];
    uint32_t LastPreviousTime;
    float LastPreviousPosition[3];
    float DebugLastPreviousYaw;
    float DebugLastPreviousPosition[3];

    float TranslationalVelocity;

    float SpinningYawPosition;
    float SpinningPitchPosition;
    float TempSpinningYawPosition;

    float SpinningCenter[2];
    float SpinningEarthFrame[3];
    float PreTargetSpinningEarthFrame[3];
    float HorizontalDistance;

    float Velocity_Chassis, Accel_Chassis;

    float SpinningPitchLPF;
} HitSpinning_t;

typedef struct
{
    uint8_t Status;
    uint8_t LastStatus;
    uint8_t isSpinning;
    uint8_t SpinningTgtValid;
    float TargetLostTime;
    float TrackingTimeStamp;
    uint32_t LostCount;
    uint32_t TrackingCount;

    uint32_t TgtSwitchPeriod_ms;
    uint32_t TgtSwitchTick_ms;

    uint8_t ArmorType;

    mat Ccb, CcbT;
    float Ccb_data[9], CcbT_data[9];
    mat Cbn, CbnT;
    float Cbn_data[9], CbnT_data[9];
    mat Rc;
    float Rc_data[9];
    mat tempMat;
    float tempMat_data[9];
    mat H, HT, M1, M2;
    float H_data[12], HT_data[12], M1_data[12], M2_data[4];
    mat ResErr, ResErrT;
    float ResErr_data[2], ResErrT_data[2];
    KalmanFilter_t TgtMotionEst;

    float BulletVelocity;

    float Distance;
    float HorizontalDistance;
    float HorizontalDistance_dot;
    float HorizontalDistance_ddot;
    float HorizontalDistanceLPF;
    float HorizontalDistance_dotLPF;
    float TgtHeight;
    float TgtHeight_dot;
    float TgtHeight_ddot;

    float Position[3];
    float Velocity[3];
    float xOyVelocity;
    float Accel[3];
    float AccLPF;

    float ForwardTime;

    float TargetBodyFrame[3];
    float TargetEarthFrame[3];
    float PreTargetEarthFrame[3];

    float YawPosition;
    float PitchPosition;
    float YawVelocity;
    float PitchVelocity;
    float YawAccel;
    float PitchAccel;
} TgtPosPredict_t;

typedef struct
{
    float CamX;
    float CamY;
    float CamZ;
    float axis_offset;

    float CameraYaw;
    float CameraPitch;
    float CameraRoll;

    float BulletYaw;
    float BulletPitch;
    float BulletYaw30;
    float BulletPitch30;
    float BulletYawRune;
    float BulletPitchRune;
} OffsetCorrection_t;

typedef struct
{
    uint8_t Allow_Shoot;
    uint8_t Last_Allow_Shoot;
    uint8_t Shoot_Freq;
    uint8_t UseAccel;
    uint8_t SpinningKeepShooting;
    uint32_t TargetValidTime;
    float MaxForwardTime;
    float SafeForwardTime;
    float MaxFreqGain;
    float PreTargetEarthFrame[3];
    float YawSpeed;
    float YawAngle;
    float YawError;
    float BulletShootDelay;
    float yaw_allow_range;       // 开火允许的Yaw范围 rad
    float pitch_allow_range;     // 开火允许的Pitch范围 rad

} ShootEvaluation_t;

typedef struct
{
    float Position[3];
    float Velocity[3];
    uint32_t TimeStamp;
    uint8_t TgtStatus;
} TgtPosFrame_t;

#define Tgt_FRAME_LEN 500
typedef struct
{
    TgtPosFrame_t TgtFrame[Tgt_FRAME_LEN];
    uint16_t LatestNum;
} TgtPosBuf_t;

extern AimAssist_t AimAssist;
extern TgtPosPredict_t TgtPosPredict;
extern ShootEvaluation_t ShootEvaluation;
extern HitSpinning_t HitSpinning;
extern TgtPosBuf_t TgtPosBuf;

void AimAssist_Init(UART_HandleTypeDef *huart);
void AimAssist_Task(void);
uint8_t isAbleToShoot(float *shoot_freq_gain);
float ShootFreqGain(void);

#endif
