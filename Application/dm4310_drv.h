#ifndef __DM4310_DRV_H__
#define __DM4310_DRV_H__

#include "main.h"
#include "can.h"
#include "controller.h"


#define MIT_MODE            0x000
#define POS_MODE            0x100
#define SPEED_MODE        0x200

// DM4310 parameter range
#define P_MIN -12.5        //位置最小值
#define P_MAX 12.5        //位置最大值
#define V_MIN -45            //速度最小值
#define V_MAX 45            //速度最大值
#define KP_MIN 0.0        //Kp最小值
#define KP_MAX 500.0    //Kp最大值
#define KD_MIN 0.0        //Kd最小值
#define KD_MAX 5.0        //Kd最大值
#define T_MIN -18            //转矩最大值
#define T_MAX 18            //转矩最小值

#define NEGATIVE 1

typedef enum {
    DM_CMD_MOTOR_MODE = 0xfc,   // 使能,会响应指令
    DM_CMD_RESET_MODE = 0xfd,   // 停止
    DM_CMD_ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
    DM_CMD_CLEAR_ERROR = 0xfb // 清除电机过热错误
} DMMotor_Mode_e;

typedef __packed struct {
    int id;  //电机ID
    int state; //电机状态
    int p_int; //电机位置获取 //编码器值(整数)
    int v_int; //电机速度获取 //编码器值(整数)
    int t_int; //电机扭矩获取 //编码器值(整数)
    int kp_int;
    int kd_int;
    float pos; // (-12.5,12.5)rad  //真实值(浮点数)
    float vel; // (-45.0,45.0)rad/s
    float tor;  // (-18.0,18.0)NM
    float Kp;
    float Kd;
    float Tmos;  //驱动MOS管温度
    float Tcoil;  //电机内部线圈温度
    float last_pos;  //上次角度记录
    float last_p_int; //上次编码器值记录
} motor_fbpara_t;

//电机控制参数设置
typedef struct {
    int8_t mode;
    float pos_set;
    float vel_set;
    float tor_set;
    float kp_set;
    float kd_set;
} motor_ctrl_t;

//达妙总结构体
typedef struct dmmotor_t {
    uint16_t CAN_ID; // 电机的CAN ID
    motor_fbpara_t para;  //反馈参数
    motor_ctrl_t ctrl;   //控制量
    motor_ctrl_t cmd;    //发送命令
    uint8_t Direction; // 电机正方向

    int32_t EncoderRoundCnt;
    int32_t EncoderTotalAngle;
    float Angle;
    int zero_offset;//pint offset
    int offset_angle;
    float pos_offset;//pos offset
    float AngleInDegree; // 电机经过零位校准后的角度(度数制)
    float AngleInDegree_LPF;
    float AngleVelocity_LPF;
    int32_t msg_cnt;
    CAN_RxHeaderTypeDef Rx_pHeader;

    float Output; // 电机电流输出(数值制)
    float Max_Out; // 电机最大输出

    PID_t PID_Torque; // 力矩环PID
    PID_t PID_Velocity; // 速度环PID
    PID_t PID_Angle; // 角度环PID

    Feedforward_t FFC_Torque;
    Feedforward_t FFC_Velocity;
    Feedforward_t FFC_Angle;

    LDOB_t LDOB;

    Lpf_t Lpf_AngleInDegree;
    Lpf_t Lpf_AngleVelocity;

    void (*TorqueCtrl_User_Func_f)(struct dmmotor_t *motor);

    void (*SpeedCtrl_User_Func_f)(struct dmmotor_t *motor);

    void (*AngleCtrl_User_Func_f)(struct dmmotor_t *motor);
} DMMotor_t;


float uint_to_float(int x_int, float x_min, float x_max, int bits);

int float_to_uint(float x_float, float x_min, float x_max, int bits);

#endif /* __DM4310_DRV_H__ */

