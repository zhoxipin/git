#pragma once
#ifndef PROTOCOL_2025_H
#define PROTOCOL_2025_H

// Serial Communication Protocol
/**
 * @author Zheng Junzhe
 * @version 2.0.0
 * @brief 基础上传协议，仅包含需要高FPS发送的信息
 * @warning 禁止更改
 */
typedef struct
{
    float   INSTimeCost;  // from BMI088_Read to CDC_Send2miniPC_Data Time cost, in milliseconds
    float   q_w;          // Quaternion w
    float   q_x;          // Quaternion x
    float   q_y;          // Quaternion y
    float   q_z;          // Quaternion z
    uint8_t autoaim_mode; // 自瞄模式 (normal: 0x00, autoaim: 0x01, small_rune: 0x02, big_rune: 0x03)
    uint8_t auxiaim_mode; // 辅助瞄准模式
} __attribute__((__packed__)) UploadCMD_1;

/**
 * @author Zheng Junzhe
 * @version 2.0.0
 * @brief 标定上传协议，仅在标定时使用，不应上场
 * @warning 禁止更改
 */
typedef struct
{
    float   INSTimeCost;  // from BMI088_Read to CDC_Send2miniPC_CI_Cali_Data Time cost, in milliseconds
    float   q_w;          // Quaternion w
    float   q_x;          // Quaternion x
    float   q_y;          // Quaternion y
    float   q_z;          // Quaternion z
    float   accel[3];     // INS accel
    float   gyro[3];      // INS gyro
    uint8_t autoaim_mode; // 自瞄模式 (normal: 0x00, autoaim: 0x01, small_rune: 0x02, big_rune: 0x03)

} __attribute__((__packed__)) UploadCMD_2;

/**
 * @author Zheng Junzhe
 * @version 2.0.0
 * @brief 裁判系统信息上传协议
 */
typedef struct
{
    uint8_t my_team_color; // 我方队伍颜色 (Red: 0x1F, Blue: 0xF1)
} __attribute__((__packed__)) UploadCMD_3;

/**
 * @author Zheng Junzhe
 * @version 2.0.0
 * @brief 哨兵裁判系统信息上传协议
*/
typedef struct
{
    uint8_t InvincibleTarget;      // 无敌目标,第0位为裁判系统读取检测，1~5位对应对方前5号车，6位为哨兵
    uint8_t AllowToAttackOutpost;  // 允许攻击前哨站
    uint8_t AllowToAttackEngineer; // 允许攻击工程
}  __attribute__((__packed__)) UploadCMD_4;

/**
 * @author Zheng Junzhe
 * @version 2.0.0
 * @brief 发射信息上传协议
 */
typedef struct
{
    uint8_t bullet_id;         // 弹丸编号
    uint8_t is_shoot_success;  // 是否发射成功
    uint8_t is_reference_recv; // 裁判系统反馈正常
    float gimbal_pitch;        // 云台Pitch
    float bullet_speed;        // 弹速
    float shoot_delay;         // 发弹延迟
} __attribute__((__packed__)) UploadCMD_5;

/**
 * @author Zheng Junzhe
 * @version 2.0.0
 * @brief 小Yaw电机信息上传协议
 */
typedef struct
{
    float yaw_motor_angle; // 小Yaw Yaw电机角度
    float pitch_motor_angle; // 小Yaw Pitch电机角度
} __attribute__((__packed__)) UploadCMD_6;

/**
 * @author Zheng Junzhe
 * @version 2.0.0
 * @brief 自瞄信息下载协议
 */
typedef struct
{
    uint8_t auxstate;            //0:关闭， 1:开启
    uint8_t target_state;        // 0: 没有捕获目标， 1: 捕获平移目标， 2: 捕获旋转目标
    uint8_t target_label;        // 锁定的目标
    uint8_t allow_shoot;         // 0: 停火， 1: 开火
    uint8_t shoot_frequency;     // 射击频率(目前电控不保证该参数被正确执行)
    float yaw_speed;
    float yaw;                   // rad
    float pitch;                 // rad
    float yaw_allow_range;       // 开火允许的Yaw范围 rad
    float pitch_allow_range;     // 开火允许的Pitch范围 rad
    uint16_t autoaim_debug_show; // 自瞄调试信息
    uint8_t health_check;        // 健康检测
} __attribute__((__packed__)) DownloadCMD_1;

/**
 * @author Zheng Junzhe
 * @version 2.0.0
 * @brief 哨兵大Yaw控制信息下载协议
 */
typedef struct
{
    float control_angle; // 哨兵大Yaw控制角度
} __attribute__((__packed__)) DownloadCMD_2;

#endif
