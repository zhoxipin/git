#include "aimassist_task.h"
#include "QuaternionEKF.h"
#include "judgement_info.h"
#include "bsp_usart_idle.h"
#include "user_lib.h"
#include "vt03.h"

AimAssist_t AimAssist = {0};
TgtPosPredict_t TgtPosPredict = {0};
ShootEvaluation_t ShootEvaluation = {0};
HitSpinning_t HitSpinning;
TgtPosBuf_t TgtPosBuf = {0};
OffsetCorrection_t OffsetCorrection;

uint8_t UseAccelPredict = FALSE;
uint8_t UseTwicePredict = TRUE;
uint8_t UseCovariance = TRUE;
uint8_t UseSpinningTgtPredict = FALSE;
int8_t Mode = 0;
float bulletModelK = 0.026f + 0.0779f * 0 + 0.055f * 0;
float SpinningYawRatio = 0.3;

/**************************** Aim Assist KF Data ******************************/
float TgtMotionEst_F[36] = {
        1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1};
float TgtMotionEst_P[36];
float TgtMotionEst_Pinit[36] = {
        1000, 0.1, 0.1, 0.1, 0.1, 0.1,
        0.1, 100000, 0.1, 0.1, 0.1, 0.1,
        0.1, 0.1, 1000, 0.1, 0.1, 0.1,
        0.1, 0.1, 0.1, 100000, 0.1, 0.1,
        0.1, 0.1, 0.1, 0.1, 1000, 0.1,
        0.1, 0.1, 0.1, 0.1, 0.1, 100000};
float TgtMotionEst_Sigma[3] = {10000, 10000, 1000};
float TgtMotionEst_Q[36] = {
        0.1, 0, 0, 0, 0, 0,
        0, 50, 0, 0, 0, 0,
        0, 0, 0.1, 0, 0, 0,
        0, 0, 0, 50, 0, 0,
        0, 0, 0, 0, 0.1, 0,
        0, 0, 0, 0, 0, 50};
float sigmaSqY = 5000;
float sigmaSqTheta = 1.25e-6;
float sigmaSqPhi = 3.5e-6;
float ChiSquare;
float xyPositonError;
float xhat_data_obsv[6];

/*************************** Aim Assist Data Buf ******************************/
static uint8_t AimAssist_Rx_Buf[AimAssist_RX_BUF_NUM];
static uint8_t AimAssist_Tx_Buf[AimAssist_TX_BUF_NUM];
ControlFrame CtrlFrameTemp;
ControlFrameFull CtrlFrameTempFull;
static uint8_t Calibration_Rx_Buf[CALIBRATION_RX_BUF_NUM];
static CalibrationFrame_t CalibrationFrame;

/************************** Aim Assist Time Stamp *****************************/
uint32_t AimAssist_DWT_Count = 0;
static float dt = 0, t = 0;
uint32_t AimAssist_DWT_Cost = 0;
static float AimAssistCostTime, AimAssisTempTimeStamp;

static void HitSpinning_Init(void);

static void ShootEvaluation_Init(void);

static void TgtMotionEst_Init(void);

static void TgtMotionEst_Update(float dt);

static void Set_AimAssistMode(void);

static float Ballistic_Compensation(float x, float y, float y_dot, float v, float *forwardTime);

static float Ballistic_Model(float x, float v, float pitch);

static void Check_Target_Status(float dt);

static uint8_t is_Target_Spinning(void);

static void HitSpinningVelocityPriori(void);

static void AimAssist_Get_Target(void);

uint8_t AimAssist_Shoot_Evaluation(void);

static void AimAssist_Send_FdbFrame(void);

static void GetTargetPosition(ControlFrame CtrlFrameTemp, uint8_t *buff);

static void GetTargetPositionFull(ControlFrameFull CtrlFrameTempFull, uint8_t *buff);

static void CalibrationCallback(CalibrationFrame_t caliFrameTemp);

static void CameraOffsetCorrection(float *inputPos, float *outputPos);

static void TgtMotionEst_Tuning(KalmanFilter_t *kf);

static void TgtMotionEst_Set_R(KalmanFilter_t *kf);

static void TgtMotionEst_ChiSquare_Test(KalmanFilter_t *kf);

static void TgtMotionEst_Reset(float *new_position);

static void AimAssist_Debug(void);

/******************************** Task Func ***********************************/
void AimAssist_Init(UART_HandleTypeDef *huart) {
    AimAssist.AA_USART = huart;

    AimAssist.Mode = AUTO_AIM;
    AimAssist.miniPC_Online = 0;
    // ң����������Ϣ��ʼ�� �����
#if ENABLE_CALIBRATION
    USART_IDLE_Init(huart, Calibration_Rx_Buf, CALIBRATION_RX_BUF_NUM);
#else
    USART_IDLE_Init(huart, AimAssist_Rx_Buf, AimAssist_RX_BUF_NUM);
#endif
    HitSpinning_Init();
    ShootEvaluation_Init();

    TgtPosPredict.ForwardTime = 0.002f;

    TgtPosPredict.BulletVelocity = 15.6f;

    TgtPosPredict.AccLPF = 0.025;
    TgtPosPredict.HorizontalDistanceLPF = 0.005;
    TgtPosPredict.HorizontalDistance_dotLPF = 0.1;

    OffsetCorrection.CamX = -16;
    OffsetCorrection.CamY = -24;
    OffsetCorrection.CamZ = 370;
    OffsetCorrection.axis_offset = 40;
    OffsetCorrection.CameraYaw = 0;
    OffsetCorrection.CameraPitch = 0;
    OffsetCorrection.CameraRoll = 0.5;
    OffsetCorrection.BulletYaw = -0.54;
    OffsetCorrection.BulletPitch = 1.0;
    // ����ʱ��������
    AimAssist.FrameDelayToINS = 0.003f;

    TgtMotionEst_Init();
    Matrix_Init(&TgtPosPredict.Ccb, 3, 3, (float *) TgtPosPredict.Ccb_data);
    Matrix_Init(&TgtPosPredict.CcbT, 3, 3, (float *) TgtPosPredict.CcbT_data);
    Matrix_Init(&TgtPosPredict.Cbn, 3, 3, (float *) TgtPosPredict.Cbn_data);
    Matrix_Init(&TgtPosPredict.CbnT, 3, 3, (float *) TgtPosPredict.CbnT_data);
    Matrix_Init(&TgtPosPredict.Rc, 3, 3, (float *) TgtPosPredict.Rc_data);
    Matrix_Init(&TgtPosPredict.tempMat, 3, 3, (float *) TgtPosPredict.tempMat_data);
    Matrix_Init(&TgtPosPredict.H, 2, 6, (float *) TgtPosPredict.H_data);
    Matrix_Init(&TgtPosPredict.HT, 6, 2, (float *) TgtPosPredict.HT_data);
    Matrix_Init(&TgtPosPredict.M1, 6, 2, (float *) TgtPosPredict.M1_data);
    Matrix_Init(&TgtPosPredict.M2, 2, 2, (float *) TgtPosPredict.M2_data);
    Matrix_Init(&TgtPosPredict.ResErr, 2, 1, (float *) TgtPosPredict.ResErr_data);
    Matrix_Init(&TgtPosPredict.ResErrT, 1, 2, (float *) TgtPosPredict.ResErrT_data);
}

static void HitSpinning_Init(void) {
    HitSpinning.MixCoef = 0.1;
    HitSpinning.SpinningThresholdScale = 3;
    HitSpinning.SpinningPitchLPF = 0.5;
    HitSpinning.UseSpinningTgtPredict = 0;
    HitSpinning.UseSpinningAccelPredict = 0;
    HitSpinning.UseCenterPredict = 0;
    HitSpinning.VelocityThreshold = 400;
    HitSpinning.UseVelocityPriori = 1;
    HitSpinning.SpinningCountThreshold = 2;
}

static void ShootEvaluation_Init(void) {
    ShootEvaluation.SpinningKeepShooting = 0;
    ShootEvaluation.UseAccel = 0;
    ShootEvaluation.MaxForwardTime = 0.35;
    ShootEvaluation.SafeForwardTime = 0.15;
    ShootEvaluation.MaxFreqGain = 10;
    ShootEvaluation.BulletShootDelay = 0.05f;
}

void AimAssist_Task(void) {
    dt = DWT_GetDeltaT(&AimAssist_DWT_Count);
    t += dt;
    AimAssisTempTimeStamp = DWT_GetDeltaT(&AimAssist_DWT_Cost);

    Set_AimAssistMode();

//    TgtMotionEst_Update(dt);
//
//    Check_Target_Status(dt);

    AimAssist_Get_Target();

    if (AimAssist.Mode == AUTO_AIM)
        is_Target_Spinning();

    AimAssist_Shoot_Evaluation();

    AimAssist_Send_FdbFrame();
    if (GlobalDebugMode == AIMASSIST_DEBUG) {
        AimAssist_Debug();
    }

    AimAssisTempTimeStamp = DWT_GetDeltaT(&AimAssist_DWT_Cost);
    AimAssistCostTime = AimAssisTempTimeStamp;
    AimAssist.LastMode = AimAssist.Mode;
    TgtPosPredict.LastStatus = TgtPosPredict.Status;
}

void InsertTgtFrame(TgtPosBuf_t *tgtBuf, float *position, float *velocity, uint32_t time_stamp, float status) {
    if (tgtBuf->LatestNum == Tgt_FRAME_LEN - 1)
        tgtBuf->LatestNum = 0;
    else
        tgtBuf->LatestNum++;

    tgtBuf->TgtFrame[tgtBuf->LatestNum].TimeStamp = time_stamp;
    for (uint8_t i = 0; i < 3; i++) {
        tgtBuf->TgtFrame[tgtBuf->LatestNum].Position[i] = position[i];
        tgtBuf->TgtFrame[tgtBuf->LatestNum].Velocity[i] = velocity[i];
    }
    tgtBuf->TgtFrame[tgtBuf->LatestNum].TgtStatus = status;
}

uint16_t FindLastTrackingFrame(TgtPosBuf_t *tgtBuf) {
    uint16_t num;

    num = tgtBuf->LatestNum;

    for (uint16_t i = 0; i < Tgt_FRAME_LEN; i++) {
        if (num == 0)
            num = Tgt_FRAME_LEN - 1;
        else
            num--;
        if (tgtBuf->TgtFrame[num].TgtStatus == TgtTracking)
            break;
    }

    return num;
}

static void AimAssist_Get_Target(void) {
    static float lastBulletSpeed = 0;
    static uint32_t count = 0;

    // ���ݲ���ϵͳ�������µ���
    // if (!is_TOE_Error(JUDGE_TOE) && shoot_data.bullet_speed > 5 && fabsf(shoot_data.bullet_speed - lastBulletSpeed) > 0.09f)
    // {
    //     if (fabsf(shoot_data.bullet_speed - TgtPosPredict.BulletVelocity) > 2.0f)
    //     {
    //         if (fabsf(shoot_data.bullet_speed - TgtPosPredict.BulletVelocity) < 40.0f)
    //             TgtPosPredict.BulletVelocity = shoot_data.bullet_speed;
    //     }
    //     else
    //         TgtPosPredict.BulletVelocity = TgtPosPredict.BulletVelocity * 0.85f + shoot_data.bullet_speed * 0.15f;
    //     lastBulletSpeed = shoot_data.bullet_speed;
    // }
    // else if (!is_TOE_Error(JUDGE_TOE) && shoot_data.bullet_speed < 5 && Shoot.BulletSpeedLimit > 5)
    //     TgtPosPredict.BulletVelocity = Shoot.BulletSpeedLimit * 0.94f;
    // TgtPosPredict.BulletVelocity = float_constrain(TgtPosPredict.BulletVelocity, 10, 35);

    if (TgtPosPredict.Status == TgtTracking || TgtPosPredict.Status == TgtConjecture) {
        // ���������ӵ�����ʱ��
        TgtPosPredict.PitchPosition = Ballistic_Compensation(TgtPosPredict.HorizontalDistance / 1000.0f,
                                                             TgtPosPredict.TgtHeight / 1000.0f,
                                                             TgtPosPredict.TgtHeight_dot / 1000.0f,
                                                             TgtPosPredict.BulletVelocity, &TgtPosPredict.ForwardTime);
        if (!isnormal(TgtPosPredict.PitchPosition))
            TgtPosPredict.PitchPosition = 0;
        if (!isnormal(TgtPosPredict.ForwardTime))
            TgtPosPredict.ForwardTime = 0;

        if (UseTwicePredict && AimAssist.Mode == AUTO_AIM) {
            for (uint8_t i = 0; i < 3; i++) {
                TgtPosPredict.PreTargetEarthFrame[i] =
                        TgtPosPredict.Position[i] + TgtPosPredict.Velocity[i] * TgtPosPredict.ForwardTime +
                        TgtPosPredict.Accel[i] * TgtPosPredict.ForwardTime * TgtPosPredict.ForwardTime * 0.5f *
                        UseAccelPredict;
            }
            // ���μ��㵯������
            TgtPosPredict.PitchPosition = Ballistic_Compensation(
                    sqrtf(TgtPosPredict.PreTargetEarthFrame[X] * TgtPosPredict.PreTargetEarthFrame[X] +
                          TgtPosPredict.PreTargetEarthFrame[Y] * TgtPosPredict.PreTargetEarthFrame[Y]) / 1000.0f,
                    TgtPosPredict.TgtHeight / 1000.0f,
                    0,
                    TgtPosPredict.BulletVelocity,
                    &TgtPosPredict.ForwardTime);
            if (!isnormal(TgtPosPredict.PitchPosition))
                TgtPosPredict.PitchPosition = 0;
            if (!isnormal(TgtPosPredict.ForwardTime))
                TgtPosPredict.ForwardTime = 0;
        }

        TgtPosPredict.ForwardTime += AimAssist.FrameDelayToINS;
        TgtPosPredict.ForwardTime = float_constrain(TgtPosPredict.ForwardTime, 0.005f, 0.75f);
        // �����ⲻ����Ԥ��
        if (TgtPosPredict.HorizontalDistance > 8000)
            TgtPosPredict.ForwardTime = 0;
        if (AimAssist.Mode == HIT_RUNE_MIN || AimAssist.Mode == HIT_RUNE_MAX)
            TgtPosPredict.ForwardTime = 0;
        for (uint8_t i = 0; i < 3; i++) {
            TgtPosPredict.PreTargetEarthFrame[i] =
                    TgtPosPredict.Position[i] + TgtPosPredict.Velocity[i] * TgtPosPredict.ForwardTime +
                    TgtPosPredict.Accel[i] * TgtPosPredict.ForwardTime * TgtPosPredict.ForwardTime * 0.5f *
                    UseAccelPredict;
        }
    } else {
        for (uint8_t i = 0; i < 3; i++)
            TgtPosPredict.PreTargetEarthFrame[i] = TgtPosPredict.Position[i];
    }
    TgtPosPredict.YawPosition = atan2f(-TgtPosPredict.PreTargetEarthFrame[X],
                                       TgtPosPredict.PreTargetEarthFrame[Y]) * RADIAN_COEF;
    if (!isnormal(TgtPosPredict.YawPosition))
        TgtPosPredict.YawPosition = 0;

    TgtPosPredict.YawPosition += OffsetCorrection.BulletYaw;
    if (TgtPosPredict.Status != TgtLost)
        TgtPosPredict.PitchPosition += OffsetCorrection.BulletPitch;

    TgtPosPredict.PitchPosition = float_constrain(TgtPosPredict.PitchPosition,
                                                  INS.Pitch - PITCH_MAX_DEG,
                                                  INS.Pitch + PITCH_MAX_DEG);

    // ��������
    if (TgtPosPredict.YawPosition > 180.0f)
        TgtPosPredict.YawPosition -= -360.0f;
    if (TgtPosPredict.YawPosition < -180.0f)
        TgtPosPredict.YawPosition += 360.0f;

    /*float MaxYaw = INS.Yaw + YAW_MAX_DEG;
    float MinYaw = INS.Yaw - YAW_MAX_DEG;

    if (TgtPosPredict.YawPosition > INS.Yaw && MaxYaw < 180)
        TgtPosPredict.YawPosition = float_constrain(TgtPosPredict.YawPosition, -180, MaxYaw);
    if (TgtPosPredict.YawPosition < INS.Yaw && MinYaw > -180)
        TgtPosPredict.YawPosition = float_constrain(TgtPosPredict.YawPosition, MinYaw, 180);*/

    // ��¼����
    if (TgtPosPredict.Status != TgtLost && count % 2 == 0) {
        InsertTgtFrame(&TgtPosBuf,
                       TgtPosPredict.TargetEarthFrame,
                       TgtPosPredict.Velocity,
                       INS_GetTimeline(),
                       TgtPosPredict.Status);
    }
    count++;
}

static void Check_Target_Status(float dt) {
    static uint32_t count = 0;
    if (INS_GetTimeline() - AimAssist.FrameTimeStamp > 1000) {
        AimAssist.Status = TargetLost;
        AimAssist.miniPC_Online = 0;
        TgtPosPredict.Status = TgtLost;
        TgtPosPredict.TrackingCount = 0;

        // �������˲�����λ
        TgtMotionEst_Reset(NULL);
#if ENABLE_CALIBRATION
        USART_IDLE_Init(AimAssist.AA_USART, Calibration_Rx_Buf, CALIBRATION_RX_BUF_NUM);
#else
        USART_IDLE_Init(AimAssist.AA_USART, AimAssist_Rx_Buf, AimAssist_RX_BUF_NUM);
#endif
    }

    if (TgtPosPredict.Status == TgtTracking) {
        if (TgtPosPredict.LastStatus == TgtLost) {
            TgtMotionEst_Reset(AimAssist.TargetEarthFrame);
            TgtPosPredict.TargetEarthFrame[X] = AimAssist.TargetEarthFrame[X];
            TgtPosPredict.TargetEarthFrame[Y] = AimAssist.TargetEarthFrame[Y];
            TgtPosPredict.TargetEarthFrame[Z] = AimAssist.TargetEarthFrame[Z];
        }
        // Ŀ�����
        TgtPosPredict.TrackingCount++;
    }
    if (TgtPosPredict.Status == TgtSwitch) {
        TgtPosPredict.TrackingCount = 0;

        if (TgtPosPredict.LastStatus != TgtSwitch) {
            TgtPosPredict.TgtSwitchPeriod_ms = INS_GetTimeline() - TgtPosPredict.TgtSwitchTick_ms;
            TgtPosPredict.TgtSwitchTick_ms = INS_GetTimeline();
        }

        // Ŀ���л� ��λ�˶���Ϣ
        // �������˲�����λ
        TgtMotionEst_Reset(AimAssist.TargetEarthFrame);

        TgtPosPredict.TargetEarthFrame[X] = AimAssist.TargetEarthFrame[X];
        TgtPosPredict.TargetEarthFrame[Y] = AimAssist.TargetEarthFrame[Y];
        TgtPosPredict.TargetEarthFrame[Z] = AimAssist.TargetEarthFrame[Z];
    }

    count++;
}

static uint8_t is_Target_Spinning(void) {
    static uint32_t TgtSwitchTimeStamp;
    static float PreviousYaw, PresentYaw;
    static float SpinningSwitchPeriodThreshold = 0.4;
    static float YawDifference;

    SpinningSwitchPeriodThreshold =
            HitSpinning.SpinningThresholdScale * TgtPosPredict.Distance / TgtPosPredict.BulletVelocity;

    if (TgtPosPredict.Status != TgtSwitch) {
        if (INS_GetTimeline() - TgtSwitchTimeStamp > SpinningSwitchPeriodThreshold * 1.5f ||
            TgtPosPredict.Status == TgtLost) {
            HitSpinning.SpinningCount = 0;
            TgtPosPredict.isSpinning = 0;
            return 0;
        }
    }

    if (TgtPosPredict.isSpinning == 1) {
        // HitSpinning.Velocity_Chassis = 0;
        // HitSpinning.Accel_Chassis = 0;
        HitSpinning.SpinningEarthFrame[X] += (HitSpinning.Velocity_Chassis * dt +
                                              HitSpinning.UseSpinningAccelPredict * HitSpinning.Accel_Chassis * dt *
                                              dt * 0.5f) *
                                             HitSpinning.UseSpinningTgtPredict;

        if (HitSpinning.SpinningCount == HitSpinning.SpinningCountThreshold) {
            HitSpinning.HorizontalDistance = TgtPosPredict.HorizontalDistance;
            HitSpinning.SpinningEarthFrame[Z] = TgtPosPredict.Position[Z];
        } else {
            HitSpinning.HorizontalDistance =
                    TgtPosPredict.HorizontalDistance * dt / (HitSpinning.SpinningPitchLPF + dt) +
                    HitSpinning.HorizontalDistance * HitSpinning.SpinningPitchLPF / (HitSpinning.SpinningPitchLPF + dt);
            if (HitSpinning.SpinningSpeedStatus == NormalSpeedSpinning)
                HitSpinning.SpinningEarthFrame[Z] = TgtPosPredict.Position[Z] * dt / (0.01f + dt) +
                                                    HitSpinning.SpinningEarthFrame[Z] * 0.01f / (0.01f + dt);
            else
                HitSpinning.SpinningEarthFrame[Z] =
                        TgtPosPredict.Position[Z] * dt / (HitSpinning.SpinningPitchLPF + dt) +
                        HitSpinning.SpinningEarthFrame[Z] * HitSpinning.SpinningPitchLPF /
                        (HitSpinning.SpinningPitchLPF + dt);
        }

        // arm_sqrt_f32(HitSpinning.SpinningEarthFrame[X] * HitSpinning.SpinningEarthFrame[X] +
        //                  HitSpinning.SpinningEarthFrame[Y] * HitSpinning.SpinningEarthFrame[Y],
        //              &HitSpinning.HorizontalDistance);

        HitSpinning.SpinningPitchPosition = Ballistic_Compensation(HitSpinning.HorizontalDistance / 1000.0f,
                                                                   HitSpinning.SpinningEarthFrame[Z] / 1000.0f,
                                                                   0.0f,
                                                                   TgtPosPredict.BulletVelocity,
                                                                   &TgtPosPredict.ForwardTime);

        TgtPosPredict.ForwardTime += AimAssist.FrameDelayToINS;
        TgtPosPredict.ForwardTime = float_constrain(TgtPosPredict.ForwardTime, 0.02f, 0.75f);

        HitSpinning.PreTargetSpinningEarthFrame[X] = HitSpinning.SpinningEarthFrame[X];
        HitSpinning.PreTargetSpinningEarthFrame[Y] = HitSpinning.SpinningEarthFrame[Y];
        HitSpinning.PreTargetSpinningEarthFrame[Z] = HitSpinning.SpinningEarthFrame[Z];

        HitSpinning.SpinningYawPosition = -atan2f(HitSpinning.PreTargetSpinningEarthFrame[X],
                                                  HitSpinning.PreTargetSpinningEarthFrame[Y]) * RADIAN_COEF;
        if (HitSpinning.SpinningSpeedStatus == HighSpeedSpinning && HitSpinning.UseCenterPredict)
            HitSpinning.SpinningYawPosition +=
                    HitSpinning.TranslationalVelocity / HitSpinning.HorizontalDistance * TgtPosPredict.ForwardTime;

        HitSpinning.SpinningYawPosition += OffsetCorrection.BulletYaw;
        HitSpinning.SpinningPitchPosition += OffsetCorrection.BulletPitch;

        if (HitSpinning.SpinningYawPosition > 180.0f)
            HitSpinning.SpinningYawPosition = -360.0f + HitSpinning.SpinningYawPosition;
        if (HitSpinning.SpinningYawPosition < -180.0f)
            HitSpinning.SpinningYawPosition = 360.0f + HitSpinning.SpinningYawPosition;
    }

    if (TgtPosPredict.Status == TgtSwitch && TgtPosPredict.LastStatus != TgtSwitch &&
        TgtPosPredict.LastStatus != TgtLost) {
        uint32_t FrameNum = FindLastTrackingFrame(&TgtPosBuf);

        HitSpinning.PresentTime = INS_GetTimeline();
        PresentYaw = AimAssist.YawPosition;
        HitSpinning.PresentPosition[X] = TgtPosPredict.TargetEarthFrame[X];
        HitSpinning.PresentPosition[Y] = TgtPosPredict.TargetEarthFrame[Y];
        HitSpinning.PresentPosition[Z] = TgtPosPredict.TargetEarthFrame[Z];

        HitSpinning.PreviousTime = TgtPosBuf.TgtFrame[FrameNum].TimeStamp;
        HitSpinning.PreviousPosition[X] = TgtPosBuf.TgtFrame[FrameNum].Position[X];
        HitSpinning.PreviousPosition[Y] = TgtPosBuf.TgtFrame[FrameNum].Position[Y];
        HitSpinning.PreviousPosition[Z] = TgtPosBuf.TgtFrame[FrameNum].Position[Z];
        PreviousYaw = -atan2f(HitSpinning.PreviousPosition[X], HitSpinning.PreviousPosition[Y]) * RADIAN_COEF;

        if (fabsf(HitSpinning.PresentPosition[Z] - HitSpinning.PreviousPosition[Z]) > 400) {
            HitSpinning.SpinningCount = 0;
            TgtPosPredict.isSpinning = 0;
            return 0;
        }

        if (PresentYaw - PreviousYaw > 180)
            YawDifference = PreviousYaw + 360 - PresentYaw;
        else if (PresentYaw - PreviousYaw < -180)
            YawDifference = PresentYaw + 360 - PreviousYaw;
        else
            YawDifference = fabsf(PresentYaw - PreviousYaw);

        if (YawDifference / RADIAN_COEF * TgtPosPredict.HorizontalDistance > 1000) {
            HitSpinning.SpinningCount = 0;
            TgtPosPredict.isSpinning = 0;
            return 0;
        }

        float xOyVelocity;
        arm_sqrt_f32(TgtPosBuf.TgtFrame[FrameNum].Velocity[X] * TgtPosBuf.TgtFrame[FrameNum].Velocity[X] +
                     TgtPosBuf.TgtFrame[FrameNum].Velocity[Y] * TgtPosBuf.TgtFrame[FrameNum].Velocity[Y],
                     &xOyVelocity);

        if (INS_GetTimeline() - TgtSwitchTimeStamp < SpinningSwitchPeriodThreshold *
                                                     (1 + TgtPosPredict.isSpinning * 0.25f) &&
            xOyVelocity > HitSpinning.VelocityThreshold)
            HitSpinning.SpinningCount++;
        else {
            if (TgtPosPredict.isSpinning == 1)
                HitSpinning.SpinningCount = HitSpinning.SpinningCountThreshold - 1;
        }

        if (HitSpinning.SpinningCount >= HitSpinning.SpinningCountThreshold) {
            TgtPosPredict.isSpinning = 1;
            TgtPosPredict.SpinningTgtValid = 1;
            if (TgtPosPredict.ForwardTime + ShootEvaluation.BulletShootDelay >
                TgtPosPredict.TgtSwitchPeriod_ms * 0.001f * (1 - HitSpinning.MixCoef) * 0.95f)
                HitSpinning.SpinningSpeedStatus = HighSpeedSpinning;
            else
                HitSpinning.SpinningSpeedStatus = NormalSpeedSpinning;
        } else {
            TgtPosPredict.isSpinning = 0;
        }

        float LastSpinningYaw = -atan2f(HitSpinning.SpinningCenter[X], HitSpinning.SpinningCenter[Y]);

        HitSpinning.SpinningCenter[X] =
                HitSpinning.PresentPosition[X] * 0.5f + HitSpinning.LastPreviousPosition[X] * 0.5f;
        HitSpinning.SpinningCenter[Y] =
                HitSpinning.PresentPosition[Y] * 0.5f + HitSpinning.LastPreviousPosition[Y] * 0.5f;
        if (HitSpinning.SpinningSpeedStatus == NormalSpeedSpinning) {
            HitSpinning.MixCoef = float_constrain(HitSpinning.MixCoef, 0, 1);
            HitSpinning.SpinningEarthFrame[X] = HitSpinning.PresentPosition[X] * HitSpinning.MixCoef +
                                                HitSpinning.LastPreviousPosition[X] * (1 - HitSpinning.MixCoef);
            HitSpinning.SpinningEarthFrame[Y] = HitSpinning.PresentPosition[Y] * HitSpinning.MixCoef +
                                                HitSpinning.LastPreviousPosition[Y] * (1 - HitSpinning.MixCoef);
        } else {
            HitSpinning.SpinningEarthFrame[X] = HitSpinning.SpinningCenter[X];
            HitSpinning.SpinningEarthFrame[Y] = HitSpinning.SpinningCenter[Y];
        }

        arm_sqrt_f32(HitSpinning.SpinningCenter[X] * HitSpinning.SpinningCenter[X] +
                     HitSpinning.SpinningCenter[Y] * HitSpinning.SpinningCenter[Y],
                     &HitSpinning.HorizontalDistance);
        float SpinningYaw = -atan2f(HitSpinning.SpinningCenter[X], HitSpinning.SpinningCenter[Y]);
        HitSpinning.TranslationalVelocity = (SpinningYaw - LastSpinningYaw) / (INS_GetTimeline() - TgtSwitchTimeStamp) *
                                            HitSpinning.HorizontalDistance;

        HitSpinning.DebugPresentYaw = -atan2f(HitSpinning.PresentPosition[X],
                                              HitSpinning.PresentPosition[Y]) * RADIAN_COEF;
        HitSpinning.DebugLastPreviousYaw = -atan2f(HitSpinning.LastPreviousPosition[X],
                                                   HitSpinning.LastPreviousPosition[Y]) * RADIAN_COEF;
        HitSpinning.DebugLastPreviousPosition[X] = HitSpinning.LastPreviousPosition[X];
        HitSpinning.DebugLastPreviousPosition[Y] = HitSpinning.LastPreviousPosition[Y];
        HitSpinning.DebugLastPreviousPosition[Z] = HitSpinning.LastPreviousPosition[Z];

        if (TgtPosPredict.isSpinning)
            HitSpinningVelocityPriori();

        TgtSwitchTimeStamp = INS_GetTimeline();
        HitSpinning.LastPresentTime = HitSpinning.PresentTime;
        HitSpinning.LastPresentPosition[X] = HitSpinning.PresentPosition[X];
        HitSpinning.LastPresentPosition[Y] = HitSpinning.PresentPosition[Y];
        HitSpinning.LastPresentPosition[Z] = HitSpinning.PresentPosition[Z];
        HitSpinning.LastPreviousTime = HitSpinning.PreviousTime;
        HitSpinning.LastPreviousPosition[X] = HitSpinning.PreviousPosition[X];
        HitSpinning.LastPreviousPosition[Y] = HitSpinning.PreviousPosition[Y];
        HitSpinning.LastPreviousPosition[Z] = HitSpinning.PreviousPosition[Z];
    }

    return TgtPosPredict.isSpinning;
}

static void HitSpinningVelocityPriori(void) {
    HitSpinning.VelocityPriori.deltaT = (HitSpinning.PreviousTime - HitSpinning.LastPresentTime);

    arm_sqrt_f32((HitSpinning.PreviousPosition[X] - HitSpinning.LastPresentPosition[X]) *
                 (HitSpinning.PreviousPosition[X] - HitSpinning.LastPresentPosition[X]) +
                 (HitSpinning.PreviousPosition[Y] - HitSpinning.LastPresentPosition[Y]) *
                 (HitSpinning.PreviousPosition[Y] - HitSpinning.LastPresentPosition[Y]),
                 &HitSpinning.VelocityPriori.ArmorDistance);

    HitSpinning.VelocityPriori.Velocity =
            HitSpinning.VelocityPriori.ArmorDistance / HitSpinning.VelocityPriori.deltaT * 1000.0f;
    // cross([x1 y1 0],[x2 y2 0]) = [0 0 x1*y2 - x2*y1]
    // 1: previous  2: lastPresent
    HitSpinning.VelocityPriori.CrossProductZ = HitSpinning.PreviousPosition[X] * HitSpinning.LastPresentPosition[Y] -
                                               HitSpinning.LastPresentPosition[X] * HitSpinning.PreviousPosition[Y];
    if (HitSpinning.VelocityPriori.CrossProductZ > 0)
        HitSpinning.VelocityPriori.Direction = Counterclockwise;
    else
        HitSpinning.VelocityPriori.Direction = Clockwise;

    float theta = atan2f(HitSpinning.PresentPosition[Y], HitSpinning.PresentPosition[X]);
    HitSpinning.VelocityPriori.VelocityX = 0.95f * HitSpinning.VelocityPriori.Velocity * arm_sin_f32(theta);
    HitSpinning.VelocityPriori.VelocityY = -0.95f * HitSpinning.VelocityPriori.Velocity * arm_cos_f32(theta);
    if (HitSpinning.VelocityPriori.Direction == Clockwise) {
        HitSpinning.VelocityPriori.VelocityX *= -1;
        HitSpinning.VelocityPriori.VelocityY *= -1;
    }
}

static void Set_AimAssistMode(void) {
    static float vPressTimeStamp, vReleaseTimeStamp;
    static float wPressTimeStamp, wReleaseTimeStamp;
    static uint16_t last_key_code;

    /*if ((last_key_code & Key_V) == 0 && remote_control.key_code & Key_V)
        vPressTimeStamp = INS_GetTimeline() / 1000.0f;
    if (last_key_code & Key_V && (remote_control.key_code & Key_V) == 0)
    {
        vReleaseTimeStamp = INS_GetTimeline() / 1000.0f;
        if (vReleaseTimeStamp - vPressTimeStamp > 0.75f)
            AimAssist.Mode = HIT_RUNE_MAX;
        else
            AimAssist.Mode = HIT_RUNE_MIN;
    }*/
    if ((last_key_code & Key_W) == 0 && remote_control.key_code & Key_W)
        wPressTimeStamp = INS_GetTimeline() / 1000.0f;
    if (last_key_code & Key_W && (remote_control.key_code & Key_W) == 0) {
        wReleaseTimeStamp = INS_GetTimeline() / 1000.0f;
        if (wReleaseTimeStamp - wPressTimeStamp > 1.5f)
            AimAssist.Mode = AUTO_AIM;
    }
    if (remote_control.key_code & Key_G ||
        (remote_control.key_code & Key_W && remote_control.key_code & Key_SHIFT))
        AimAssist.Mode = AUTO_AIM;

    last_key_code = remote_control.key_code;
}

static void AimAssist_Send_FdbFrame(void) {
    static float TxTimeStamp = 0, TxTimeStamp1 = 0;

    AimAssist.FdbFrame.FDBF_SOF = 0x66;
    if (robot_state.robot_id <= 7)
        AimAssist.FdbFrame.myteam = RED;
    else
        AimAssist.FdbFrame.myteam = BLUE;
    if (AimAssist.FdbFrame.myteam == RED)
        if (game_robot_HP.blue_outpost_HP == 0)
            AimAssist.FdbFrame.outpost_alive = 0;
        else AimAssist.FdbFrame.outpost_alive = 1;
    if (AimAssist.FdbFrame.myteam == BLUE)
        if (game_robot_HP.red_outpost_HP == 0)
            AimAssist.FdbFrame.outpost_alive = 0;
        else AimAssist.FdbFrame.outpost_alive = 1;
    AimAssist.FdbFrame.pitch = 0;
    AimAssist.FdbFrame.yaw = 0;
    AimAssist.FdbFrame.bullet_speed = TgtPosPredict.BulletVelocity * 10;
    AimAssist.FdbFrame.time_stamp_ms = INS_GetTimeline();
/*    if (outpost_HP > 200)
        AimAssist.FdbFrame.outpost_alive = 1;
    else*/
    //AimAssist.FdbFrame.mode = AimAssist.Mode;
    AimAssist.FdbFrame.mode = 1;
    AimAssist.FdbFrame.FDBF_EOF = 0x88;
    memcpy(AimAssist_Tx_Buf, &AimAssist.FdbFrame, AimAssist_TX_BUF_NUM);
    if (t - TxTimeStamp1 > 0.05f && AimAssist.FdbFlag == 0) {
        HAL_UART_Transmit_DMA(AimAssist.AA_USART, AimAssist_Tx_Buf, AimAssist_TX_BUF_NUM);
        TxTimeStamp1 = t;
    }
    // 注意，应当避免在发送数据的过程中再次发送数据，否则会导致数据丢失，或卡死
    if (AimAssist.FdbFlag && t - TxTimeStamp1 > 0.003f) {
        AimAssist.FdbFlag = 0;
        if (HAL_UART_GetState(AimAssist.AA_USART) != HAL_UART_STATE_BUSY_TX &&
            (__HAL_UART_GET_FLAG(AimAssist.AA_USART, UART_FLAG_TC) == SET))
            HAL_UART_Transmit_DMA(AimAssist.AA_USART, AimAssist_Tx_Buf, AimAssist_TX_BUF_NUM);
    }
}

uint8_t AimAssist_Shoot_Evaluation(void) {
    if (ShootEvaluation.MaxForwardTime <= ShootEvaluation.SafeForwardTime)
        ShootEvaluation.MaxForwardTime = ShootEvaluation.SafeForwardTime + 0.2f;
    ShootEvaluation.Shoot_Freq = 0;

    if (TgtPosPredict.isSpinning) {
        if (fabsf(HitSpinning.TranslationalVelocity) > 0.125f) {
            ShootEvaluation.Shoot_Freq = 0;
            ShootEvaluation.Allow_Shoot = 0;
            return ShootEvaluation.Allow_Shoot;
        }
        if (ShootEvaluation.SpinningKeepShooting || HitSpinning.SpinningSpeedStatus == HighSpeedSpinning) {
            ShootEvaluation.YawAngle = HitSpinning.SpinningYawPosition;
            if (INS.Yaw - ShootEvaluation.YawAngle > 180)
                ShootEvaluation.YawError =
                        (INS.Yaw - ShootEvaluation.YawAngle - 360) / RADIAN_COEF * TgtPosPredict.HorizontalDistance;
            else if (INS.Yaw - ShootEvaluation.YawAngle < -180)
                ShootEvaluation.YawError =
                        (INS.Yaw - ShootEvaluation.YawAngle + 360) / RADIAN_COEF * TgtPosPredict.HorizontalDistance;
            else
                ShootEvaluation.YawError =
                        (INS.Yaw - ShootEvaluation.YawAngle) / RADIAN_COEF * TgtPosPredict.HorizontalDistance;
            if (ShootEvaluation.YawError > 250) {
                ShootEvaluation.Shoot_Freq = 0;
                ShootEvaluation.Allow_Shoot = 0;
                return ShootEvaluation.Allow_Shoot;
            } else {
                ShootEvaluation.Shoot_Freq = 3;
                ShootEvaluation.Allow_Shoot = 1;
                return ShootEvaluation.Allow_Shoot;
            }
        }
    }

    if (TgtPosPredict.Status == TgtLost || INS_GetTimeline() - ShootEvaluation.TargetValidTime > 25) {
        ShootEvaluation.Shoot_Freq = 0;
        ShootEvaluation.Allow_Shoot = 0;
        return ShootEvaluation.Allow_Shoot;
    }

    if (TgtPosPredict.ForwardTime > ShootEvaluation.MaxForwardTime && TgtPosPredict.isSpinning == 0) {
        ShootEvaluation.Shoot_Freq = 3;
        ShootEvaluation.Allow_Shoot = 0;
        return ShootEvaluation.Allow_Shoot;
    }

    float forwardTime;
    if (TgtPosPredict.isSpinning)
        forwardTime = TgtPosPredict.ForwardTime + ShootEvaluation.BulletShootDelay;
    else
        forwardTime = TgtPosPredict.ForwardTime;
    for (uint8_t i = 0; i < 3; i++) {
        ShootEvaluation.PreTargetEarthFrame[i] = TgtPosPredict.Position[i] + TgtPosPredict.Velocity[i] * forwardTime +
                                                 ShootEvaluation.UseAccel * TgtPosPredict.Accel[i] * forwardTime *
                                                 forwardTime * 0.5f;
    }

    ShootEvaluation.YawAngle = atan2f(-ShootEvaluation.PreTargetEarthFrame[X],
                                      ShootEvaluation.PreTargetEarthFrame[Y]) * RADIAN_COEF +
                               OffsetCorrection.BulletYaw;
    if (ShootEvaluation.YawAngle > 180.0f)
        ShootEvaluation.YawAngle -= -360.0f;
    if (ShootEvaluation.YawAngle < -180.0f)
        ShootEvaluation.YawAngle += 360.0f;

    if (INS.Yaw - ShootEvaluation.YawAngle > 180)
        ShootEvaluation.YawError =
                (INS.Yaw - ShootEvaluation.YawAngle - 360) / RADIAN_COEF * TgtPosPredict.HorizontalDistance;
    else if (INS.Yaw - ShootEvaluation.YawAngle < -180)
        ShootEvaluation.YawError =
                (INS.Yaw - ShootEvaluation.YawAngle + 360) / RADIAN_COEF * TgtPosPredict.HorizontalDistance;
    else
        ShootEvaluation.YawError =
                (INS.Yaw - ShootEvaluation.YawAngle) / RADIAN_COEF * TgtPosPredict.HorizontalDistance;

    if (forwardTime < ShootEvaluation.SafeForwardTime) {
        switch (TgtPosPredict.ArmorType) {
            case SMALL_ARMOR:
                if (fabsf(ShootEvaluation.YawError) < 135.0f / 2.0f * 1.15f) {
                    ShootEvaluation.Shoot_Freq = ShootEvaluation.MaxFreqGain;
                    ShootEvaluation.Allow_Shoot = 1;
                    return ShootEvaluation.Allow_Shoot;
                }
                break;

            case BIG_ARMOR:
                if (fabsf(ShootEvaluation.YawError) < 230.0f / 2.0f * 1.15f) {
                    ShootEvaluation.Shoot_Freq = ShootEvaluation.MaxFreqGain;
                    ShootEvaluation.Allow_Shoot = 1;
                    return ShootEvaluation.Allow_Shoot;
                }
                break;
        }
    }

    switch (TgtPosPredict.ArmorType) {
        case SMALL_ARMOR:
            if (fabsf(ShootEvaluation.YawError) > 135.0f / 2.0f * 1.35f *
                                                  (1 + 0.15f * TgtPosPredict.isSpinning)) {
                ShootEvaluation.Shoot_Freq = 6;
                ShootEvaluation.Allow_Shoot = 0;
                return ShootEvaluation.Allow_Shoot;
            }
            break;

        case BIG_ARMOR:
            if (fabsf(ShootEvaluation.YawError) > 230.0f / 2.0f * 1.35f *
                                                  (1 + 0.15f * TgtPosPredict.isSpinning)) {
                ShootEvaluation.Shoot_Freq = 6;
                ShootEvaluation.Allow_Shoot = 0;
                return ShootEvaluation.Allow_Shoot;
            }
            break;
    }

    ShootEvaluation.Allow_Shoot = 1;
    if (AimAssist.Mode == AUTO_AIM && TgtPosPredict.isSpinning == 0)
        ShootEvaluation.Shoot_Freq =
                ShootEvaluation.MaxFreqGain / (ShootEvaluation.SafeForwardTime - ShootEvaluation.MaxForwardTime) *
                (forwardTime - ShootEvaluation.MaxForwardTime);
    else
        ShootEvaluation.Shoot_Freq = 0;

    return ShootEvaluation.Allow_Shoot;
}

/********************************* Ballistic **********************************/
static float Ballistic_Compensation(float x, float y, float y_dot, float v, float *forwardTime) {
    static float k;
    static float temp_x, temp_y, dy;
    static float pitch;
    static float gain = 0.6;
    static float t;
    static float heightGain = 0.1;
    static uint8_t useyModel = 0;
    static float t0, vy0, C, h, top_h;
    static float useYdot = 0;

    temp_x = x;
    temp_y = y;
    k = bulletModelK;
    for (int i = 0; i < 20; i++) {
        pitch = atan2f(temp_y, temp_x);
        t = Ballistic_Model(x, v, pitch);
        // ������������
        if (t < 1e-4f) {
            *forwardTime = 0;
            return atan2f(y, x) * RADIAN_COEF;
        }

        if (useyModel) {
            // ������ֵ�����������ģ�ͣ���զ����
            vy0 = v * arm_sin_f32(pitch);
            if (pitch > 0) {
                t0 = atanf(vy0 / sqrtf(9.8f / k)) / sqrtf(k * 9.8f);
                if (t0 > t) {
                    C = -1 / k * logf(fabsf(arm_cos_f32(atanf(vy0 / sqrtf(9.8f / k)))));
                    h = 1 / k * logf(fabsf(arm_cos_f32(atanf(vy0 * sqrtf(k / 9.8f)) - sqrtf(9.8f * k) * t))) + C;
                } else {
                    C = -1 / k * logf(fabsf(arm_cos_f32(atanf(vy0 / sqrtf(9.8f / k)))));
                    top_h = 1 / k * logf(fabsf(arm_cos_f32(atanf(vy0 * sqrtf(k / 9.8f)) - sqrtf(9.8f * k) * t0))) + C;
                    C = logf(2) / k;
                    h = top_h +
                        (1 / k * ((t - t0) * sqrtf(9.8f * k) - logf(fabsf(expf(2 * sqrtf(9.8f * k) * (t - t0)) + 1))) +
                         C);
                }
                dy = y + y_dot * t * useYdot - h;
            } else
                dy = y + y_dot * t * useYdot - (v * arm_sin_f32(pitch) * t - 4.9f * t * t);
        } else {
            // �߶�����
            if (y > 0.6f)
                dy = y + y_dot * t * useYdot + heightGain * (y - 0.6f) - (v * arm_sin_f32(pitch) * t - 4.9f * t * t);
            else
                dy = y + y_dot * t * useYdot - (v * arm_sin_f32(pitch) * t - 4.9f * t * t);
        }
        temp_y += dy * gain;

        if (fabsf(dy) < 0.005f)
            break;
    }
    pitch = atan2f(temp_y, temp_x);
    *forwardTime = t;
    return pitch * RADIAN_COEF;
}

static float Ballistic_Model(float x, float v, float pitch) {
    static float vx0;
    static float t;
    // static float ft, dft_dt, temp_t;
    vx0 = v * arm_cos_f32(pitch);

    t = (expf(bulletModelK * x) - 1) / (bulletModelK * vx0);
    if (t > 1)
        t = 1;
    if (!isnormal(t))
        t = 0;

    //    for (int i = 0; i < 20; i++)
    //    {
    //        ft = logf(k * vx0 * t + 1) / k - x_dot * t - x;
    //        dft_dt = vx0 / (k * vx0 * t + 1) - t;
    //        temp_t = t - ft / dft_dt;

    //        if (temp_t > 1.5f)
    //            temp_t = 1.5f;
    //        if (temp_t < 0)
    //            temp_t = 0;

    //        t = temp_t;
    //    }

    return t;
}

/******************************* miniPC Handle ********************************/
//static void GetTargetPositionFull(ControlFrameFull CtrlFrameTempFull, uint8_t *buff)
//{
//    AimAssist.miniPC_Online = 1;
//    AimAssist.Frame_dt = DWT_GetDeltaT(&AimAssist.UpdatePeriodCount);
//    AimAssist.TimeConsuming = CtrlFrameTempFull.time_stamp_ms / 10.0f / 1000.0f;
//
//    // ��������ϵƫ��
//    AimAssist.FrameTimeStamp = INS_GetTimeline();
//
//    if (CtrlFrameTempFull.flg != 0)
//    {
//        static uint16_t qFrameNum;
//        // static float q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
//        static float rawPos[3];
//        static float tempPitch, tempYaw, tempRoll, q[4], cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
//        static float y, theta, phi, tantheta, tanphi, costheta, cosphi;
//
//        memcpy(&AimAssist.CtrlFrameFull, buff, AimAssist_RX_BUF_NUM);
//        TgtPosPredict.LostCount = 0;
//
//        AimAssist.dataValid = 1;
//        AimAssist.TargetID = AimAssist.CtrlFrameFull.flg;
//
//        ShootEvaluation.TargetValidTime = INS_GetTimeline();
//
//        // �������ϵ��������Э�������
//        y = AimAssist.CtrlFrameFull.py;
//        theta = atan2f(AimAssist.CtrlFrameFull.px, AimAssist.CtrlFrameFull.py);
//        phi = atan2f(AimAssist.CtrlFrameFull.pz, AimAssist.CtrlFrameFull.py);
//        tantheta = (float)AimAssist.CtrlFrameFull.px / AimAssist.CtrlFrameFull.py;
//        tanphi = (float)AimAssist.CtrlFrameFull.pz / AimAssist.CtrlFrameFull.py;
//        costheta = arm_cos_f32(theta);
//        cosphi = arm_cos_f32(phi);
//
//        if (UseCovariance)
//        {
//            TgtPosPredict.Rc_data[0] = sigmaSqY * tantheta * tantheta + sigmaSqTheta * y * y / powf(costheta, 4);
//            TgtPosPredict.Rc_data[1] = sigmaSqY * tantheta;
//            TgtPosPredict.Rc_data[2] = sigmaSqY * tantheta * tanphi;
//            TgtPosPredict.Rc_data[3] = TgtPosPredict.Rc_data[1];
//            TgtPosPredict.Rc_data[4] = sigmaSqY;
//            TgtPosPredict.Rc_data[5] = sigmaSqY * tanphi;
//            TgtPosPredict.Rc_data[6] = TgtPosPredict.Rc_data[2];
//            TgtPosPredict.Rc_data[7] = TgtPosPredict.Rc_data[5];
//            TgtPosPredict.Rc_data[8] = sigmaSqY * tanphi * tanphi + sigmaSqPhi * y * y / powf(cosphi, 4);
//        }
//        else
//        {
//            TgtPosPredict.Rc_data[0] = 5;
//            TgtPosPredict.Rc_data[1] = 0;
//            TgtPosPredict.Rc_data[2] = 0;
//            TgtPosPredict.Rc_data[3] = 0;
//            TgtPosPredict.Rc_data[4] = 25;
//            TgtPosPredict.Rc_data[5] = 0;
//            TgtPosPredict.Rc_data[6] = 0;
//            TgtPosPredict.Rc_data[7] = 0;
//            TgtPosPredict.Rc_data[8] = 5;
//        }
//
//        // ���ϵ�任����̨ϵ
//        rawPos[X] = AimAssist.CtrlFrameFull.px;
//        rawPos[Y] = AimAssist.CtrlFrameFull.py;
//        rawPos[Z] = AimAssist.CtrlFrameFull.pz;
//
//        CameraOffsetCorrection(rawPos, AimAssist.TargetBodyFrame);
//
//        arm_sqrt_f32(AimAssist.TargetBodyFrame[X] * AimAssist.TargetBodyFrame[X] +
//                         AimAssist.TargetBodyFrame[Y] * AimAssist.TargetBodyFrame[Y] +
//                         AimAssist.TargetBodyFrame[Z] * AimAssist.TargetBodyFrame[Z],
//                     &AimAssist.Distance);
//
//        TgtPosPredict.ArmorType = AimAssist.CtrlFrameFull.type;
//
//        for (uint8_t i = 0; i < 3; i++)
//            TgtPosPredict.TargetBodyFrame[i] = AimAssist.TargetBodyFrame[i];
//
//        // ��������״̬
//        AimAssist.Status = TargetValid;
//
//        // ��ȡ�������ϵ��Ŀ��λ��
//        AimAssist.DelayTime = AimAssist.FrameDelayToINS + AimAssist.TimeConsuming;
//        qFrameNum = Find_qFrame(&INS.qFrame, INS_GetTimeline() - (uint32_t)(AimAssist.DelayTime * 1000));
//        // qFrameNum = FindTimeMatchFrame(&QuaternionBuffer, INS_GetTimeline() - (uint32_t)(AimAssist.DelayTime * 1000));
//
//        memcpy(q, INS.qFrame[qFrameNum].q, sizeof(q));
//        tempYaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
//        //tempPitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
//        tempPitch = 0;
//        tempRoll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3]));
//
//        cosYaw = arm_cos_f32(tempYaw);
//        cosPitch = arm_cos_f32(tempPitch);
//        cosRoll = arm_cos_f32(tempRoll);
//        sinYaw = arm_sin_f32(tempYaw);
//        sinPitch = arm_sin_f32(tempPitch);
//        sinRoll = arm_sin_f32(tempRoll);
//
//        // ������ת����     1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
//        TgtPosPredict.Cbn_data[0] = cosYaw * cosRoll - sinYaw * sinPitch * sinRoll;
//        TgtPosPredict.Cbn_data[1] = -cosPitch * sinYaw;
//        TgtPosPredict.Cbn_data[2] = cosYaw * sinRoll + cosRoll * sinYaw * sinPitch;
//        TgtPosPredict.Cbn_data[3] = cosYaw * sinPitch * sinRoll + cosRoll * sinYaw;
//        TgtPosPredict.Cbn_data[4] = cosYaw * cosPitch;
//        TgtPosPredict.Cbn_data[5] = sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
//        TgtPosPredict.Cbn_data[6] = -cosPitch * sinRoll;
//        TgtPosPredict.Cbn_data[7] = sinPitch;
//        TgtPosPredict.Cbn_data[8] = cosPitch * cosRoll;
//        AimAssist.TargetEarthFrame[X] = TgtPosPredict.Cbn_data[0] * AimAssist.TargetBodyFrame[X] +
//                                        TgtPosPredict.Cbn_data[1] * AimAssist.TargetBodyFrame[Y] +
//                                        TgtPosPredict.Cbn_data[2] * AimAssist.TargetBodyFrame[Z];
//        AimAssist.TargetEarthFrame[Y] = TgtPosPredict.Cbn_data[3] * AimAssist.TargetBodyFrame[X] +
//                                        TgtPosPredict.Cbn_data[4] * AimAssist.TargetBodyFrame[Y] +
//                                        TgtPosPredict.Cbn_data[5] * AimAssist.TargetBodyFrame[Z];
//        AimAssist.TargetEarthFrame[Z] = TgtPosPredict.Cbn_data[6] * AimAssist.TargetBodyFrame[X] +
//                                        TgtPosPredict.Cbn_data[7] * AimAssist.TargetBodyFrame[Y] +
//                                        TgtPosPredict.Cbn_data[8] * AimAssist.TargetBodyFrame[Z];
//        //AimAssist.TargetEarthFrame[X] = rawPos[X] - 16;
//        //AimAssist.TargetEarthFrame[Y] = rawPos[Y] - 24;
//        //AimAssist.TargetEarthFrame[Z] = rawPos[Z] + 370;
//        for (uint8_t i = 0; i < 3; i++)
//        {
//            TgtPosPredict.TargetEarthFrame[i] = AimAssist.TargetEarthFrame[i];
//            TgtPosPredict.TgtMotionEst.MeasuredVector[i] = AimAssist.TargetEarthFrame[i];
//        }
//
//        AimAssist.YawPosition = -atan2f(AimAssist.TargetEarthFrame[X], AimAssist.TargetEarthFrame[Y]) * RADIAN_COEF;
//        AimAssist.PitchPosition = atan2f(AimAssist.TargetEarthFrame[Z], AimAssist.TargetEarthFrame[Y]) * RADIAN_COEF;
//    }
//    else
//    {
//        // �Ӿ�δʶ��Ŀ��
//        AimAssist.CtrlFrameFull.flg = 0;
//
//        if (TgtPosPredict.LostCount == 0)
//            TgtPosPredict.TargetLostTime = INS_GetTimeline();
//
//        if (INS_GetTimeline() - TgtPosPredict.TargetLostTime < 300)
//        {
//            TgtPosPredict.Status = TgtConjecture;
//        }
//        else
//        {
//            // Ŀ�궪ʧ ��λ�˶���Ϣ
//            AimAssist.Status = TargetLost;
//            TgtPosPredict.Status = TgtLost;
//            TgtPosPredict.TrackingCount = 0;
//
//            // �������˲�����λ
//            TgtMotionEst_Reset(NULL);
//        }
//        TgtPosPredict.LostCount++;
//    }
//}
#if ENABLE_CALIBRATION
static void CalibrationCallback(CalibrationFrame_t CaliFrameTemp)
{
    AimAssist.TimeConsuming = CaliFrameTemp.time_stamp_ms / 10.0f / 1000.0f;
    if (CaliFrameTemp.flg != 0)
    {
        static uint16_t qFrameNum;
        static float tempPitch, tempYaw, tempRoll;
        static float q[4];

        if (INS.Gyro[X] * INS.Gyro[X] * INS.Gyro[Y] * INS.Gyro[Y] * INS.Gyro[Z] * INS.Gyro[Z] < 0.01f)
            AimAssist.dataValid = 1;

        // ��ȡ�������ϵ��Ŀ��λ��
        AimAssist.DelayTime = AimAssist.FrameDelayToINS + AimAssist.TimeConsuming;
        qFrameNum = FindTimeMatchFrame(&QuaternionBuffer, INS_GetTimeline() - (uint32_t)(AimAssist.DelayTime * 1000));

        memcpy(q, QuaternionBuffer.qFrame[qFrameNum].q, sizeof(q));
        tempYaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
        tempPitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
        tempRoll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3]));
    }
}
#endif

void Callback_AimAssist_Handle(uint8_t *buff) {
#if ENABLE_CALIBRATION
    memcpy(&CalibrationFrame, buff, CALIBRATION_RX_BUF_NUM);
#else
    if (buff[0] == 0x66)
        memcpy(&CtrlFrameTempFull, buff, AimAssist_RX_BUF_NUM);

    // if (CtrlFrameTempFull.CF_SOF != 0x66)
    // {
    //     USART_IDLE_Init(AimAssist.AA_USART, AimAssist_Rx_Buf, AimAssist_RX_BUF_NUM);
    //     return;
    // }
#endif

#if ENABLE_CALIBRATION
    CalibrationCallback(CalibrationFrame);
#else
//    GetTargetPositionFull(CtrlFrameTempFull, buff);
#endif
    AimAssist.FdbFlag = 1;
}

static void CameraOffsetCorrection(float *inputPos, float *outputPos) {
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    static uint8_t flag = 1;

    if (fabsf(OffsetCorrection.CameraYaw - lastYawOffset) > 0.001f ||
        fabsf(OffsetCorrection.CameraPitch - lastPitchOffset) > 0.001f ||
        fabsf(OffsetCorrection.CameraRoll - lastRollOffset) > 0.001f || flag) {
        cosYaw = arm_cos_f32(OffsetCorrection.CameraYaw / 57.295779513f);
        cosPitch = arm_cos_f32(OffsetCorrection.CameraPitch / 57.295779513f);
        cosRoll = arm_cos_f32(OffsetCorrection.CameraRoll / 57.295779513f);
        sinYaw = arm_sin_f32(OffsetCorrection.CameraYaw / 57.295779513f);
        sinPitch = arm_sin_f32(OffsetCorrection.CameraPitch / 57.295779513f);
        sinRoll = arm_sin_f32(OffsetCorrection.CameraRoll / 57.295779513f);

        // ������ת����     1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll - sinYaw * sinPitch * sinRoll;
        c_12 = -cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll + cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll + cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        TgtPosPredict.Ccb_data[0] = c_11;
        TgtPosPredict.Ccb_data[1] = c_12;
        TgtPosPredict.Ccb_data[2] = c_13;
        TgtPosPredict.Ccb_data[3] = c_21;
        TgtPosPredict.Ccb_data[4] = c_22;
        TgtPosPredict.Ccb_data[5] = c_23;
        TgtPosPredict.Ccb_data[6] = c_31;
        TgtPosPredict.Ccb_data[7] = c_32;
        TgtPosPredict.Ccb_data[8] = c_33;
        flag = 0;
    }

    outputPos[X] = c_11 * inputPos[X] +
                   c_12 * inputPos[Y] +
                   c_13 * inputPos[Z] + OffsetCorrection.CamX;
    outputPos[Y] = c_21 * inputPos[X] +
                   c_22 * inputPos[Y] +
                   c_23 * inputPos[Z] + OffsetCorrection.CamY;
    outputPos[Z] = c_31 * inputPos[X] +
                   c_32 * inputPos[Y] +
                   c_33 * inputPos[Z] + OffsetCorrection.CamZ;

    //outputPos[Y] -= OffsetCorrection.axis_offset * arm_cos_f32(Gimbal.PitchMotor.para.pos);   //达妙返回位置即弧度制
    //outputPos[Z] -= OffsetCorrection.axis_offset * arm_sin_f32(Gimbal.PitchMotor.para.pos);
    outputPos[Y] -= OffsetCorrection.axis_offset * arm_cos_f32(Gimbal.PitchMotor.AngleInDegree * PI / 180.0f);
    outputPos[Z] += OffsetCorrection.axis_offset * arm_sin_f32(Gimbal.PitchMotor.AngleInDegree * PI / 180.0f);

    lastYawOffset = OffsetCorrection.CameraYaw;
    lastPitchOffset = OffsetCorrection.CameraPitch;
    lastRollOffset = OffsetCorrection.CameraRoll;
}

void USER_UART_RxIdleCallback(UART_HandleTypeDef *huart) {
    if (huart == remote_control.RC_USART)
        Callback_RC_Handle(&remote_control, sbus_rx_buf);
    if(huart == VT03_Usart && is_TOE_Error(RC_TOE) && Gimbal.isGaming)
    {
        VT03_FIFO_Handle(VT03_Receive_Buff);
        VT03_Recover_DT7(&VT03_RC,&remote_control);
    }
    if (huart == AimAssist.AA_USART)
#if ENABLE_CALIBRATION
        Callback_AimAssist_Handle(Calibration_Rx_Buf);
#else
        Callback_AimAssist_Handle(AimAssist_Rx_Buf);
#endif
}

/*************************** TgtMotionEst KF Func *****************************/
static void TgtMotionEst_Init(void) {
    TgtPosPredict.TgtMotionEst.UseAutoAdjustment = TRUE;
    Kalman_Filter_Init(&TgtPosPredict.TgtMotionEst, 6, 0, 3);
    TgtPosPredict.TgtMotionEst.MeasurementMap[0] = 1;
    TgtPosPredict.TgtMotionEst.MeasurementMap[1] = 3;
    TgtPosPredict.TgtMotionEst.MeasurementMap[2] = 5;
    TgtPosPredict.TgtMotionEst.MeasurementDegree[0] = 1;
    TgtPosPredict.TgtMotionEst.MeasurementDegree[1] = 1;
    TgtPosPredict.TgtMotionEst.MeasurementDegree[2] = 1;
    TgtPosPredict.TgtMotionEst.MatR_DiagonalElements[0] = 100;
    TgtPosPredict.TgtMotionEst.MatR_DiagonalElements[1] = 100;
    TgtPosPredict.TgtMotionEst.MatR_DiagonalElements[2] = 100;
    memcpy(TgtPosPredict.TgtMotionEst.F_data, TgtMotionEst_F, sizeof(TgtMotionEst_F));
    memcpy(TgtPosPredict.TgtMotionEst.P_data, TgtMotionEst_Pinit, sizeof(TgtMotionEst_Pinit));
    memcpy(TgtPosPredict.TgtMotionEst.Q_data, TgtMotionEst_Q, sizeof(TgtMotionEst_Q));
    TgtPosPredict.TgtMotionEst.SkipEq3 = TRUE;
    TgtPosPredict.TgtMotionEst.SkipEq4 = TRUE;
    TgtPosPredict.TgtMotionEst.User_Func0_f = TgtMotionEst_Tuning;
    TgtPosPredict.TgtMotionEst.User_Func2_f = TgtMotionEst_Set_R;
    TgtPosPredict.TgtMotionEst.User_Func3_f = TgtMotionEst_ChiSquare_Test;
}

static void TgtMotionEst_Update(float dt) {
    static float sigmaSqrt[3];
    /*
     0  1  2  3  4  5
     6  7  8  9 10 11
    12 13 14 15 16 17
    18 19 20 21 22 23
    24 25 26 27 28 29
    30 31 32 33 34 35
    */
    for (uint8_t i = 0; i < 3; i++)
        sigmaSqrt[i] = TgtMotionEst_Sigma[i] * TgtMotionEst_Sigma[i];

    TgtPosPredict.TgtMotionEst.F_data[1] = dt;
    TgtPosPredict.TgtMotionEst.F_data[15] = dt;
    TgtPosPredict.TgtMotionEst.F_data[29] = dt;

    TgtMotionEst_Q[0] = 0.25f * dt * dt * dt * dt * sigmaSqrt[X];
    TgtMotionEst_Q[1] = 0.5f * dt * dt * dt * sigmaSqrt[X];
    TgtMotionEst_Q[6] = 0.5f * dt * dt * dt * sigmaSqrt[X];
    TgtMotionEst_Q[7] = dt * dt * sigmaSqrt[X];

    TgtMotionEst_Q[14] = 0.25f * dt * dt * dt * dt * sigmaSqrt[Y];
    TgtMotionEst_Q[15] = 0.5f * dt * dt * dt * sigmaSqrt[Y];
    TgtMotionEst_Q[20] = 0.5f * dt * dt * dt * sigmaSqrt[Y];
    TgtMotionEst_Q[21] = dt * dt * sigmaSqrt[Y];

    TgtMotionEst_Q[28] = 0.25f * dt * dt * dt * dt * sigmaSqrt[Z];
    TgtMotionEst_Q[29] = 0.5f * dt * dt * dt * sigmaSqrt[Z];
    TgtMotionEst_Q[34] = 0.5f * dt * dt * dt * sigmaSqrt[Z];
    TgtMotionEst_Q[35] = dt * dt * sigmaSqrt[Z];

    if (TgtPosPredict.LastStatus == TgtSwitch && TgtPosPredict.isSpinning && HitSpinning.UseVelocityPriori) {
        TgtPosPredict.TgtMotionEst.xhat_data[1] = HitSpinning.VelocityPriori.VelocityX;
        TgtPosPredict.TgtMotionEst.xhat_data[3] = HitSpinning.VelocityPriori.VelocityY;
    }

    Kalman_Filter_Update(&TgtPosPredict.TgtMotionEst);

    for (uint8_t i = 0; i < TgtPosPredict.TgtMotionEst.xhatSize; i++) {
        if (!isnormal(TgtPosPredict.TgtMotionEst.xhat_data[i]))
            TgtPosPredict.TgtMotionEst.xhat_data[i] = 0;
        if (!isnormal(TgtPosPredict.TgtMotionEst.FilteredValue[i]))
            TgtPosPredict.TgtMotionEst.FilteredValue[i] = 0;
    }

    for (uint8_t i = 0; i < 3; i++) {
        // TgtPosPredict.AccLPF
        TgtPosPredict.Accel[i] = (TgtPosPredict.TgtMotionEst.FilteredValue[i * 2 + 1] - TgtPosPredict.Velocity[i]) /
                                 (TgtPosPredict.AccLPF + dt) +
                                 TgtPosPredict.Accel[i] * TgtPosPredict.AccLPF / (TgtPosPredict.AccLPF + dt);
        TgtPosPredict.Position[i] = TgtPosPredict.TgtMotionEst.FilteredValue[i * 2];
        TgtPosPredict.Velocity[i] = TgtPosPredict.TgtMotionEst.FilteredValue[i * 2 + 1];
    }

    arm_sqrt_f32(TgtPosPredict.Velocity[X] * TgtPosPredict.Velocity[X] +
                 TgtPosPredict.Velocity[Y] * TgtPosPredict.Velocity[Y],
                 &TgtPosPredict.xOyVelocity);

    arm_sqrt_f32(TgtPosPredict.Position[X] * TgtPosPredict.Position[X] +
                 TgtPosPredict.Position[Y] * TgtPosPredict.Position[Y] +
                 TgtPosPredict.Position[Z] * TgtPosPredict.Position[Z],
                 &TgtPosPredict.Distance);
    float HorizontalDistance;
    arm_sqrt_f32(TgtPosPredict.Position[X] * TgtPosPredict.Position[X] +
                 TgtPosPredict.Position[Y] * TgtPosPredict.Position[Y],
                 &HorizontalDistance);
    if (HorizontalDistance < 1e-5f)
        HorizontalDistance = 1e-5f;
    TgtPosPredict.HorizontalDistance = HorizontalDistance * dt / (TgtPosPredict.HorizontalDistanceLPF + dt) +
                                       TgtPosPredict.HorizontalDistance * TgtPosPredict.HorizontalDistanceLPF /
                                       (TgtPosPredict.HorizontalDistanceLPF + dt);

    float HorizontalDistance_dot =
            TgtPosPredict.Position[X] / TgtPosPredict.HorizontalDistance * TgtPosPredict.Velocity[X] +
            TgtPosPredict.Position[Y] / TgtPosPredict.HorizontalDistance * TgtPosPredict.Velocity[Y];
    TgtPosPredict.HorizontalDistance_dot =
            HorizontalDistance_dot * dt / (TgtPosPredict.HorizontalDistance_dotLPF + dt) +
            TgtPosPredict.HorizontalDistance_dot * TgtPosPredict.HorizontalDistance_dotLPF /
            (TgtPosPredict.HorizontalDistance_dotLPF + dt);

    TgtPosPredict.TgtHeight = TgtPosPredict.Position[Z];
    TgtPosPredict.TgtHeight_dot = TgtPosPredict.Velocity[Z];
}

static void TgtMotionEst_Set_R(KalmanFilter_t *kf) {
    if (kf->Pminus_data[0] > TgtMotionEst_Pinit[0])
        kf->Pminus_data[0] = TgtMotionEst_Pinit[0];
    if (kf->Pminus_data[14] > TgtMotionEst_Pinit[14])
        kf->Pminus_data[14] = TgtMotionEst_Pinit[14];
    if (kf->Pminus_data[28] > TgtMotionEst_Pinit[28])
        kf->Pminus_data[28] = TgtMotionEst_Pinit[28];
    if (kf->MeasurementValidNum != 0 || kf->UseAutoAdjustment == 0) {
        Matrix_Transpose(&TgtPosPredict.Ccb, &TgtPosPredict.CcbT);
        Matrix_Multiply(&TgtPosPredict.Rc, &TgtPosPredict.CcbT, &TgtPosPredict.tempMat); // tempMat = Rc��CcbT
        Matrix_Multiply(&TgtPosPredict.Ccb, &TgtPosPredict.tempMat, &kf->R);             // R = Ccb��Rc��CcbT
        Matrix_Transpose(&TgtPosPredict.Cbn, &TgtPosPredict.CbnT);
        Matrix_Multiply(&kf->R, &TgtPosPredict.CbnT, &TgtPosPredict.tempMat); // tempMat = Ccb��Rc��CcbT��CbnT
        Matrix_Multiply(&TgtPosPredict.Cbn, &TgtPosPredict.tempMat, &kf->R);  // R = Cbn��Ccb��Rc��CcbT��CbnT
    }
}

static void TgtMotionEst_ChiSquare_Test(KalmanFilter_t *kf) {
    static uint8_t debugchart = 0, enChiSquareTest = 1;
    static float TgtSwitchThreshold = 20, dist, xhatMinusDist, xSq, ySq, zSq;
    static uint32_t CatchSwitchCount = 0;
    // ���� z(k) - H xhat'(k)
    kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT); // z|x => x|z
    kf->temp_matrix.numRows = kf->H.numRows;
    kf->temp_matrix.numCols = kf->Pminus.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix); // temp_matrix = H��P'(k)
    kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
    kf->temp_matrix1.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1); // temp_matrix1 = H��P'(k)��HT
    kf->S.numRows = kf->R.numRows;
    kf->S.numCols = kf->R.numCols;
    kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S); // S = H P'(k) HT + R
    kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);     // temp_matrix1 = inv(H��P'(k)��HT + R)

    kf->temp_vector.numRows = kf->H.numRows;
    kf->temp_vector.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->H, &kf->xhatminus, &kf->temp_vector); // temp_vector = H xhat'(k)
    kf->temp_vector1.numRows = kf->z.numRows;
    kf->temp_vector1.numCols = 1;
    kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1); // temp_vector1 = z(k) - H xhat'(k)

    // chi-square test
    // kf->temp_matrix.numRows = kf->temp_vector1.numRows;
    // kf->temp_matrix.numCols = 1;
    // kf->MatStatus = Matrix_Multiply(&kf->temp_matrix1, &kf->temp_vector1, &kf->temp_matrix); // temp_matrix = inv(H��P'(k)��HT + R)��(z(k) - H xhat'(k))
    // kf->temp_vector.numRows = 1;
    // kf->temp_vector.numCols = kf->temp_vector1.numRows;
    // kf->MatStatus = Matrix_Transpose(&kf->temp_vector1, &kf->temp_vector); // temp_vector = (z(k) - H xhat'(k))'
    // kf->temp_matrix.numRows = 1;
    // kf->temp_matrix.numCols = 1;
    // kf->MatStatus = Matrix_Multiply(&kf->temp_vector, &kf->temp_vector1, &kf->temp_matrix);

    // arm_sqrt_f32(kf->z_data[X] * kf->z_data[X] +
    //                  kf->z_data[Y] * kf->z_data[Y],
    //              &dist);
    // if (dist < 100)
    //     dist = 100;

    // ���㿨������в�z(k) - h(xhat'(k))�뿨������H����
    arm_sqrt_f32(kf->z_data[X] * kf->z_data[X] +
                 kf->z_data[Y] * kf->z_data[Y],
                 &dist);
    arm_sqrt_f32(kf->xhatminus_data[0] * kf->xhatminus_data[0] +
                 kf->xhatminus_data[2] * kf->xhatminus_data[2],
                 &xhatMinusDist);
    xSq = kf->xhatminus_data[0] * kf->xhatminus_data[0];
    ySq = kf->xhatminus_data[2] * kf->xhatminus_data[2];
    zSq = kf->xhatminus_data[4] * kf->xhatminus_data[4];
    TgtPosPredict.H_data[0] = kf->xhatminus_data[2] / (xSq + ySq);
    TgtPosPredict.H_data[1] = 0;
    TgtPosPredict.H_data[2] = -kf->xhatminus_data[0] / (xSq + ySq);
    TgtPosPredict.H_data[3] = 0;
    TgtPosPredict.H_data[4] = 0;
    TgtPosPredict.H_data[5] = 0;
    TgtPosPredict.H_data[6] = -kf->xhatminus_data[0] * kf->xhatminus_data[4] / (xSq + ySq + zSq) / xhatMinusDist;
    TgtPosPredict.H_data[7] = 0;
    TgtPosPredict.H_data[8] = -kf->xhatminus_data[2] * kf->xhatminus_data[4] / (xSq + ySq + zSq) / xhatMinusDist;
    TgtPosPredict.H_data[9] = 0;
    TgtPosPredict.H_data[10] = xhatMinusDist / (xSq + ySq + zSq);
    TgtPosPredict.H_data[11] = 0;

    kf->MatStatus = Matrix_Transpose(&TgtPosPredict.H, &TgtPosPredict.HT);
    TgtPosPredict.M1.numRows = 6;
    TgtPosPredict.M1.numCols = 2;
    kf->MatStatus = Matrix_Multiply(&kf->Pminus, &TgtPosPredict.HT, &TgtPosPredict.M1); // M1 = P'(k)��HT
    TgtPosPredict.M2.numRows = 2;
    TgtPosPredict.M2.numCols = 2;
    kf->MatStatus = Matrix_Multiply(&TgtPosPredict.H, &TgtPosPredict.M1, &TgtPosPredict.M2); // M2 = H��P'(k)��HT
    TgtPosPredict.M2_data[0] += sigmaSqTheta;
    TgtPosPredict.M2_data[3] += sigmaSqPhi;
    TgtPosPredict.M1.numRows = 2;
    TgtPosPredict.M1.numCols = 2;
    kf->MatStatus = Matrix_Inverse(&TgtPosPredict.M2, &TgtPosPredict.M1); // M1 = inv(H��P'(k)��HT)
    TgtPosPredict.ResErr_data[0] = atan2f(kf->z_data[X], kf->z_data[Y]) - atan2f(kf->xhatminus_data[0],
                                                                                 kf->xhatminus_data[2]);
    TgtPosPredict.ResErr_data[1] = atan2f(kf->z_data[Z], dist) - atan2f(kf->xhatminus_data[4], xhatMinusDist);
    kf->MatStatus = Matrix_Transpose(&TgtPosPredict.ResErr, &TgtPosPredict.ResErrT);
    TgtPosPredict.M2.numRows = 2;
    TgtPosPredict.M2.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&TgtPosPredict.M1,
                                    &TgtPosPredict.ResErr,
                                    &TgtPosPredict.M2); // M2 = inv(H��P'��HT)��(z - h(xhat'))
    kf->temp_matrix.numRows = 1;
    kf->temp_matrix.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&TgtPosPredict.ResErrT,
                                    &TgtPosPredict.M2,
                                    &kf->temp_matrix); // temp_matrix = (z - h(xhat'))T��inv(H��P'��HT)��(z - h(xhat')) �õ���⺯��r

    if (debugchart)
        Serial_Debug(&huart1,
                     1,
                     kf->temp_matrix.pData[0] * dist * 0.00001f,
                     AimAssist.TargetEarthFrame[X] / 100,
                     AimAssist.TargetEarthFrame[Y] / 100,
                     AimAssist.TargetEarthFrame[Z] / 100,
                     TgtSwitchThreshold,
                     dist / 100);
    // ��������
    arm_sqrt_f32((kf->z_data[X] - kf->xhatminus_data[0]) * (kf->z_data[X] - kf->xhatminus_data[0]) +
                 (kf->z_data[Y] - kf->xhatminus_data[2]) * (kf->z_data[Y] - kf->xhatminus_data[2]),
                 &xyPositonError);
    ChiSquare = kf->temp_matrix.pData[0] * dist * 0.00001f;
    if ((ChiSquare > TgtSwitchThreshold ||
         (xyPositonError > 250 && INS_GetTimeline() - TgtPosPredict.TrackingTimeStamp > 125)) &&
        enChiSquareTest) {
        if (CatchSwitchCount <= 4) {
            // ��Ϣδͨ���������� ��Ԥ��
            // xhat(k) = xhat'(k)
            // P(k) = P'(k)
            memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
            memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
            kf->SkipEq5 = TRUE;
            if (CatchSwitchCount >= 2)
                TgtPosPredict.Status = TgtSwitch;
            CatchSwitchCount++;
            return;
        } else {
            kf->SkipEq5 = FALSE;
        }
    } else {
        TgtPosPredict.TrackingTimeStamp = INS_GetTimeline();
        TgtPosPredict.Status = TgtTracking;
        CatchSwitchCount = 0;
        kf->SkipEq5 = FALSE;
    }

    kf->temp_matrix.numRows = kf->Pminus.numRows;
    kf->temp_matrix.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix); // temp_matrix = P'(k)��HT
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);

    kf->temp_vector.numRows = kf->K.numRows;
    kf->temp_vector.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->K,
                                    &kf->temp_vector1,
                                    &kf->temp_vector); // temp_vector = K(k)��(z(k) - H��xhat'(k))
    kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
}

static void TgtMotionEst_Tuning(KalmanFilter_t *kf) {
    memcpy(TgtMotionEst_F, kf->F_data, sizeof(TgtMotionEst_F));
    memcpy(TgtMotionEst_P, kf->P_data, sizeof(TgtMotionEst_P));
    memcpy(kf->Q_data, TgtMotionEst_Q, sizeof(TgtMotionEst_Q));
}

static void TgtMotionEst_Reset(float *new_position) {
    for (uint8_t i = 0; i < 6; i++) {
        if (i < 3) {
            TgtPosPredict.Velocity[i] = 0;
            TgtPosPredict.Accel[i] = 0;
        }
        if (i == 0 || i == 2 || i == 4)
            continue;
        TgtPosPredict.TgtMotionEst.xhat_data[i] = 0;
        TgtPosPredict.TgtMotionEst.FilteredValue[i] = 0;
    }
    TgtPosPredict.HorizontalDistance_dot = 0;
    memcpy(TgtPosPredict.TgtMotionEst.P_data, TgtMotionEst_Pinit, sizeof(TgtMotionEst_Pinit));
    if (new_position != NULL) {
        TgtPosPredict.TgtMotionEst.xhat_data[0] = new_position[X];
        TgtPosPredict.TgtMotionEst.xhat_data[2] = new_position[Y];
        TgtPosPredict.TgtMotionEst.xhat_data[4] = new_position[Z];
        TgtPosPredict.TgtMotionEst.FilteredValue[0] = new_position[X];
        TgtPosPredict.TgtMotionEst.FilteredValue[2] = new_position[Y];
        TgtPosPredict.TgtMotionEst.FilteredValue[4] = new_position[Z];
        arm_sqrt_f32(new_position[X] * new_position[X] +
                     new_position[Y] * new_position[Y],
                     &TgtPosPredict.HorizontalDistance);
        TgtPosPredict.TgtMotionEst.P_data[0] = TgtPosPredict.TgtMotionEst.R_data[0];
        TgtPosPredict.TgtMotionEst.P_data[2] = TgtPosPredict.TgtMotionEst.R_data[1];
        TgtPosPredict.TgtMotionEst.P_data[4] = TgtPosPredict.TgtMotionEst.R_data[2];
        TgtPosPredict.TgtMotionEst.P_data[12] = TgtPosPredict.TgtMotionEst.R_data[3];
        TgtPosPredict.TgtMotionEst.P_data[14] = TgtPosPredict.TgtMotionEst.R_data[4];
        TgtPosPredict.TgtMotionEst.P_data[16] = TgtPosPredict.TgtMotionEst.R_data[5];
        TgtPosPredict.TgtMotionEst.P_data[24] = TgtPosPredict.TgtMotionEst.R_data[6];
        TgtPosPredict.TgtMotionEst.P_data[26] = TgtPosPredict.TgtMotionEst.R_data[7];
        TgtPosPredict.TgtMotionEst.P_data[28] = TgtPosPredict.TgtMotionEst.R_data[8];
    }
}

static void AimAssist_Debug(void) {
    static uint32_t count = 0;
    static float AimAssistYaw, AimAssistPitch;
    if (TgtPosPredict.isSpinning) {
        AimAssistYaw = HitSpinning.SpinningYawPosition;
        AimAssistPitch = HitSpinning.SpinningPitchPosition;
    } else {
        AimAssistYaw = TgtPosPredict.YawPosition;
        AimAssistPitch = TgtPosPredict.PitchPosition;
    }
    if (Mode == -1)
        Serial_Debug(&huart1, 1, AimAssist.TargetBodyFrame[0], AimAssist.TargetBodyFrame[1],
                     AimAssist.TargetBodyFrame[2], 0,
                     0, 0);
    if (Mode == 0)
        Serial_Debug(&huart1, 1, AimAssist.YawPosition, INS.Yaw, AimAssistYaw,
                     AimAssistPitch, INS.Pitch, TgtPosPredict.Status);
    if (Mode == 1 && count % 2 == 0)
        Serial_Debug(&huart1,
                     1,
                     TgtPosPredict.Position[0],
                     TgtPosPredict.Position[1],
                     TgtPosPredict.Position[2],
                     TgtPosPredict.PreTargetEarthFrame[0],
                     TgtPosPredict.PreTargetEarthFrame[1],
                     TgtPosPredict.PreTargetEarthFrame[2]);
    if (Mode == 2 && count % 2 == 0)
        Serial_Debug(&huart1,
                     1,
                     TgtPosPredict.TargetEarthFrame[0],
                     TgtPosPredict.TargetEarthFrame[1],
                     TgtPosPredict.TargetEarthFrame[2],
                     TgtPosPredict.PreTargetEarthFrame[0],
                     TgtPosPredict.PreTargetEarthFrame[1],
                     TgtPosPredict.PreTargetEarthFrame[2]);
    if (Mode == 3)
        Serial_Debug(&huart1, 1, AimAssist.YawPosition, AimAssist.PitchPosition, 0,
                     atan2f(AimAssist.TargetEarthFrame[X], AimAssist.TargetEarthFrame[Y]) * RADIAN_COEF,
                     atan2f(AimAssist.TargetEarthFrame[Z], AimAssist.TargetEarthFrame[Y]) * RADIAN_COEF, 0);
    if (Mode == 4)
        Serial_Debug(&huart1, 1, AimAssist.YawPosition, 0, 0,
                     0, 0, TgtPosPredict.Status);
    if (Mode == 5)
        Serial_Debug(&huart1, 1, TgtPosPredict.YawVelocity, TgtPosPredict.PitchVelocity, 0,
                     0, 0, TgtPosPredict.Status);
    if (Mode == 6)
        Serial_Debug(&huart1, 1, TgtPosPredict.YawAccel, TgtPosPredict.PitchAccel, 0,
                     0, 0, TgtPosPredict.Status);
    if (Mode == 7 && AimAssist.dataValid) {
        Serial_Debug(&huart1,
                     1,
                     CalibrationFrame.x / 10,
                     CalibrationFrame.y / 10,
                     CalibrationFrame.z / 10,
                     CalibrationFrame.rVecx * 100,
                     CalibrationFrame.rVecx * 100,
                     CalibrationFrame.rVecx * 100);
        AimAssist.dataValid = 0;
    }
    if (Mode == 10) {
        float HorizontalDistance;
        arm_sqrt_f32(TgtPosPredict.Position[X] * TgtPosPredict.Position[X] +
                     TgtPosPredict.Position[Y] * TgtPosPredict.Position[Y],
                     &HorizontalDistance);
        Serial_Debug_Indeterminate_Length(2, HorizontalDistance, TgtPosPredict.HorizontalDistance);
    }
    if (Mode == 11) {
        float HorizontalDistance_dot =
                TgtPosPredict.Position[X] / TgtPosPredict.HorizontalDistance * TgtPosPredict.Velocity[X] +
                TgtPosPredict.Position[Y] / TgtPosPredict.HorizontalDistance * TgtPosPredict.Velocity[Y];
        Serial_Debug_Indeterminate_Length(2, HorizontalDistance_dot, TgtPosPredict.HorizontalDistance_dot);
    }
    if (Mode == 12)
        Serial_Debug_Indeterminate_Length(2, ShootEvaluation.YawAngle, INS.Yaw);
    /*
     0  1  2  3  4  5
     6  7  8  9 10 11
    12 13 14 15 16 17
    18 19 20 21 22 23
    24 25 26 27 28 29
    30 31 32 33 34 35
    */
    if (Mode == 13)
        Serial_Debug_Indeterminate_Length(3, TgtMotionEst_P[0], TgtMotionEst_P[14], TgtMotionEst_P[28]);
    count++;
}
