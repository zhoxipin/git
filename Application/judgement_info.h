#ifndef _JUDGEMENT_INFO_1__
#define _JUDGEMENT_INFO_1__

#include "usart.h"
#include "stdint.h"

#define JUDGE_SOF (uint8_t)0xA5
#define GAME_DATA 0x0201
#define ROBOT_STATE_DATA 0x0001
#define POWER_HEAT_DATA 0x0202
#define ROBOT_POS_DATA 0x0203
#define ROBOT_HURT_DATA 0x0206
#define USER_DATA 0x0301

typedef enum
{
    GAME_STATUS_CMD_ID = 0x0001,
    GAME_RESULT_CMD_ID = 0x0002,
    GAME_ROBOT_HP_CMD_ID = 0x0003,
    EVEN_DATA_CMD_ID = 0x0101,
    REFEREE_WARNING_CMD_ID = 0x0104,
    DART_INFO_CMD_ID = 0x0105,
    GAME_ROBOT_STATUS_CMD_ID = 0x0201,
    POWER_HEAT_DATA_CMD_ID = 0x0202,
    GAME_ROBOT_POS_CMD_ID = 0x0203,
    BUFF_MUSK_CMD_ID = 0x0204,
    ROBOT_HURT_CMD_ID = 0x0206,
    SHOOT_DATA_CMD_ID = 0x0207,
    PROJECTILE_ALLOWANCE_CMD_ID = 0x0208,
    RFID_STATUS_CMD_ID = 0x0209,
    DART_CLIENT_CMD_ID = 0x020A,
    GROUND_ROBOT_POSITION_CMD_ID = 0x020B,
    RADAR_MARK_CMD_ID = 0x020C,
    SENTRY_INFO_CMD_ID = 0x020D,
    RADAR_INFO_CMD_ID = 0x020E,
    ROBOT_INTERACTION_DATA_CMD_ID = 0x0301,
    CUSTOM_ROBOT_DATA_CMD_ID = 0x0302,
    MAP_COMMAND_CMD_ID = 0x0303,
    REOMTE_CONTROL_CMD_ID = 0x0304,
    MAP_ROBOT_DATA_CMD_ID = 0x0305,
    CUSTOM_CLIENT_DATA_CMD_ID = 0x0306,
    MAP_DATA_CMD_ID = 0x0307,
    CUSTOM_INFO_CMD_ID = 0x0308,
    ROBOT_CUSTOM_DATA = 0x0309,
    IDCustomData,
} referee_cmd_id_t;

typedef enum
{
    RED_HERO = 1,
    RED_ENGINEER = 2,
    RED_STANDARD_1 = 3,
    RED_STANDARD_2 = 4,
    RED_STANDARD_3 = 5,
    RED_AERIAL = 6,
    RED_SENTRY = 7,
    BLUE_HERO = 101,
    BLUE_ENGINEER = 102,
    BLUE_STANDARD_1 = 103,
    BLUE_STANDARD_2 = 104,
    BLUE_STANDARD_3 = 105,
    BLUE_AERIAL = 106,
    BLUE_SENTRY = 107,
} robot_id_t;

typedef enum
{
    RMUC = 1,
    RMUT = 2,
    ICRA = 3,
    RMUL_3V3 = 4,
    RMUL_StandardSolo = 5,
}game_type_t;

typedef enum
{
    PROGRESS_UNSTART = 0,
    PROGRESS_PREPARE = 1,
    PROGRESS_SELFCHECK = 2,
    PROGRESS_5sCOUNTDOWN = 3,
    PROGRESS_BATTLE = 4,
    PROGRESS_CALCULATING = 5,
    PROGRESS_INIT = 9,
} game_progress_t;

typedef struct
{
    uint8_t buf[136];
    uint8_t bufup[136];
    uint8_t header[5];
    uint16_t cmd;
    uint8_t data[127];
    uint8_t tail[2];
} judge_receive_t;

// 0X0001 比赛状态数据
typedef __packed struct
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} __attribute__((__packed__)) ext_game_status_t;

// 0x0002 比赛结果数据
typedef __packed struct
{
    uint8_t winner;
} ext_game_result_t;

// 0x0003 机器人血量数据 (reserved为保留位)
typedef __packed struct
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t reserved1;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t reserved2;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} ext_game_robot_HP_t;

// 0x0101 场地事件数据
typedef __packed struct
{
    uint32_t event_data;
} ext_event_data_t;

// 0x0104 裁判警告信息
typedef __packed struct
{
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
} ext_referee_warning_t;

// 0x0105 飞镖发射信息
typedef __packed struct
{
    uint8_t dart_remaining_time;
    uint16_t dart_info;
} ext_dart_info_t;

// 0x0201 机器人状态数据
typedef __packed struct
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
} ext_robot_status_t;

// 0x0202 实时功率热量数据
typedef __packed struct
{
    uint16_t resrved1;
    uint16_t resrved2;
    float resrved3;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_1_barrel_heat;
    uint16_t shooter_17mm_2_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
} ext_power_heat_data_t;

// 0x0203 机器人位置数据
typedef __packed struct
{
    float x;
    float y;
    float angle;
} ext_robot_pos_t;

// 0x0204 机器人增益数据
typedef __packed struct
{
    uint8_t recovery_buff;
    uint8_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
    uint8_t remaining_energy;
} ext_buff_t;

// 0x0206 受伤数据
typedef __packed struct
{
    uint8_t armor_id : 4;
    uint8_t HP_deduction_reason : 4;
} ext_hurt_data_t;

// 0x0207 实时射击信息
typedef __packed struct
{
    uint8_t bullet_type;
    uint8_t shooter_number;
    uint8_t launching_frequency;
    float initial_speed;
} ext_shoot_data_t;

// 0x0208 剩余发弹量
typedef __packed struct
{
    uint16_t projectile_allowance_17mm;
    uint16_t projectile_allowance_42mm;
    uint16_t remaining_gold_coin;
    uint16_t projectile_allowance_fortress;
} ext_projectile_allowance_t;

// 0x0209 RFID状态
typedef __packed struct
{
    // uint32_t rfid_status;
    uint8_t BaseBuff : 1;                // 己方基地增益点
    uint8_t CentralHightsBuff : 1;       // 己方中央高地增益点
    uint8_t EnemyCentralHighsBuff : 1;   // 对方中央高地增益点
    uint8_t TraHightsBuff : 1;           // 己方梯形高地增益点
    uint8_t EnemyTraHightsBuff : 1;      // 对方梯形高地增益点
    uint8_t TerrainCrossingBeforeSlopeBuff : 1;   // 己方地形跨越增益点（飞坡）（靠近己方一侧飞坡前）
    uint8_t TerrainCrossingAfterSlopeBuff : 1;   // 己方地形跨越增益点（飞坡）（靠近己方一侧飞坡后）
    uint8_t EnergyTerrainCrossingBeforeSlopeBuff : 1;       // 对方地形跨越增益点（飞坡）（靠近对方一侧飞坡前）
    uint8_t EnergyTerrainCrossingAfterSlopeBuff : 1;       // 对方地形跨越增益点（飞坡）（靠近对方一侧飞坡后）
    uint8_t TerrainCrossingDownCentralHighsBuff : 1;          // 己方地形跨越增益点（中央高地下方）
    uint8_t TerrainCrossingUpCentralHighsBuff : 1;    // 己方地形跨越增益点（中央高地上方）
    uint8_t EnergyTerrainCrossingDownCentralHighsBuff : 1;     // 对方地形跨越增益点（中央高地下方）
    uint8_t EnergyTerrainCrossingUpCentralHighsBuff : 1;             // 对方地形跨越增益点（中央高地上方）
    uint8_t TerrainCrossingDownHighwayBuff : 1;             // 己方地形跨越增益点（公路下方）
    uint8_t TerrainCrossingUpHighwayBuff : 1;        // 己方地形跨越增益点（公路上方）
    uint8_t EnergyTerrainCrossingDownHighwayBuff : 1;   // 对方地形跨越增益点（公路下方）
    uint8_t EnergyTerrainCrossingUpHighwayBuff  : 1;      // 对方地形跨越增益点（公路上方）
    uint8_t FortBuff : 1;                // 己方堡垒增益点
    uint8_t OutPostBuff : 1;             // 己方前哨站增益点
    uint8_t NoOverleapRechargeArea : 1;  // 己方与兑换区不重叠的补给区/RMUL补给区
    uint8_t RechargeArea : 1;             // 己方与兑换区重叠的补给区
    uint8_t BigResourceIsland : 1;         // 己方大资源岛增益点
    uint8_t EnergyBigResourceIsland : 1;   // 对方大资源岛增益点
    uint8_t CentralBuffPointforRMUL : 1;          // 中心增益点（仅 RMUL 适用）
    uint8_t EnergyFortBuff : 1;           //敌方堡垒增益点
    uint8_t resrved;                      // 保留
} ext_rfid_status_t;

// 0x020A 飞镖发射站数据
typedef __packed struct
{
    uint8_t dart_launch_opening_status;
    uint8_t reserved;
    uint16_t target_change_time;
    uint16_t latest_launch_cmd_time;
} ext_dart_client_cmd_t;

// 0x020B 地面机器人位置数据
typedef __packed struct
{
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
    float reserved1;
    float reserved2;
} ext_ground_robot_position_t;

// 0x020C 雷达标记数据
typedef __packed struct
{
    uint8_t mark_hero_progress : 1;
    uint8_t mark_engineer_progress : 1;
    uint8_t mark_standard_3_progress : 1;
    uint8_t mark_standard_4_progress : 1;
    uint8_t mark_sentry_progress : 1;
} ext_radar_mark_data_t;

// 0x020D 烧饼自主决策数据
typedef __packed struct
{
    // uint32_t sentry_info;
    uint16_t ExchangedBulletNum : 11;
    uint8_t ExchangedBulletNumRemotely : 4;
    uint8_t ExchangedHealthRemotely : 4;
    uint8_t IsSentryCanRevive : 1;
    uint8_t IsSentryCanReviveRemotely : 1;
    uint16_t GoldToReviveRemotely : 10;
    uint8_t Reserved1 : 1;
    uint8_t IsSentryGetOutOfWar : 1;
    uint16_t RemainBulletNumToExchange : 11;
    uint8_t Reserved2 : 4;
} __attribute__((__packed__)) ext_sentry_info_t;
// 0x020E 雷达自主决策数据
typedef __packed struct
{
    uint8_t NumOfStartDoubleHurt : 2;
    uint8_t IfEnemyDoubleHurt : 1;
    uint8_t Reserved : 5;
} __attribute__((__packed__)) ext_radar_info_t;

// 学生机器人间通信 cmd_id 0x0301，内容ID:0x0200~0x02FF
typedef __packed struct
{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[112];
} ext_robot_interaction_data_t;

//0x0100 选手端删除图层
typedef __packed struct
{
    uint8_t operate_tpye;
    uint8_t layer;
} ext_client_custom_graphic_delete_t;

//0x0101 图形配置
typedef __packed struct
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye : 3;
    uint32_t graphic_tpye : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t details_a : 9;
    uint32_t details_b : 9;
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    uint32_t details_c : 10;
    uint32_t details_d : 11;
    uint32_t details_e : 11;
} graphic_data_struct_t;

//0x0102 选手端绘制两个图形
typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_double_t;

//0x0103 选手端绘制五个图形
typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct[5];
} ext_client_custom_graphic_five_t;

//0x0104 选手端绘制七个图形
typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_seven_t;

//0x0110 选手端绘制字符
typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct;
    uint8_t data[30];
} ext_client_custom_character_t;

//0x0120 烧饼自主决策指令
typedef __packed struct
{
    uint32_t sentry_cmd;
} ext_sentry_cmd_t;

//0x0121 雷达自主决策指令
typedef __packed struct
{
    uint8_t radar_cmd;
} ext_radar_cmd_t;

//**********************************小地图交互数据**********************************//

// 0x0303 小地图交互数据用于发送
typedef __packed struct
{
    float target_position_x;
    float target_position_y;
    uint8_t cmd_keyboard;
    uint8_t target_robot_id;
    uint16_t cmd_source;
} ext_map_command_t;

// 0x0305 小地图交互数据用于接收
typedef __packed struct
{
    uint16_t hero_position_x;
    uint16_t hero_position_y;
    uint16_t engineer_position_x;
    uint16_t engineer_position_y;
    uint16_t infantry_3_position_x;
    uint16_t infantry_3_position_y;
    uint16_t infantry_4_position_x;
    uint16_t infantry_4_position_y;
    uint16_t infantry_5_position_x;
    uint16_t infantry_5_position_y;
    uint16_t sentry_position_x;
    uint16_t sentry_position_y;
} ext_map_robot_data_t;
// 0x0307 选手端接收烧饼或半自动数据
typedef __packed struct
{
    uint8_t intention;
    uint16_t start_position_x;
    uint16_t start_position_y;
    int8_t delta_x[49];
    int8_t delta_y[49];
    uint16_t sender_id;
} ext_map_data_t;

// 0x0308 选手端接收机器人数据
typedef __packed struct
{
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[30];
} ext_custom_info_t ;

//**********************************图传链路交互数据**********************************
// 0x0302 使用自定义控制器通过图传链路向对应的机器人发送数据//
typedef __packed struct
{
    uint8_t data[100];
}ext_custom_robot_data_t;

// 0x0309 机器人可通过图传链路向对应的操作手选手端连接的自定义控制器发送数据（RMUL暂不适用）
typedef __packed struct
{
    uint8_t data[100];
}ext_robot_custom_data_t;
// 0x0304 通过遥控器发送的键鼠遥控数据将同步通过图传链路发送给对应机器人
typedef __packed struct
{
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    int8_t left_button_down;
    int8_t right_button_down;
    uint16_t keyboard_value;
    uint16_t reserved;
}ext_remote_control_t;
//**********************************非图传链路交互数据**********************************//
//0x0306 操作手可使用自定义控制器模拟键鼠操作选手端
typedef __packed struct
{
    uint16_t key_value;
    uint16_t x_position:12;
    uint16_t mouse_left:4;
    uint16_t y_position:12;
    uint16_t mouse_right:4;
    uint16_t reserved;
}ext_custom_client_data_t;

//

extern ext_game_status_t game_status;                           // 比赛状态数据
extern ext_game_result_t game_result;                           // 比赛结果数据
extern ext_game_robot_HP_t game_robot_HP;                       // 机器人血量数据
extern ext_event_data_t event_data;                             // 场地事件数据
extern ext_referee_warning_t referee_warning;                   // 裁判警告信息
extern ext_dart_info_t dart_info;                               // 飞镖发射数据
extern ext_robot_status_t robot_state;                          // 机器人状态数据
extern ext_power_heat_data_t power_heat_data;                   // 实时功率热量数据
extern ext_robot_pos_t robot_pos;                               // 机器人位置
extern ext_buff_t buff_musk;                                    // 机器人增益
extern ext_hurt_data_t hurt_data;                               // 伤害状态数据
extern ext_shoot_data_t shoot_data;                             // 实时射击数据
extern uint8_t shoot_data_count;                                // 判断射击数据是否更新
extern uint8_t last_shoot_data_count;
extern ext_projectile_allowance_t projectile_allowance;         // 允许发弹量
extern ext_rfid_status_t rfid_status;                           // RFID模块状态
extern ext_dart_client_cmd_t dart_client_cmd;                   // 飞镖发射站状态
extern ext_ground_robot_position_t ground_robot_pos;            // 地面机器人位置
extern ext_radar_mark_data_t radar_mark_data;                   // 雷达标记数据
extern ext_sentry_info_t sentry_info;                           // 哨兵自主决策数据
extern ext_radar_info_t radar_info;                             // 雷达自主决策数据
extern ext_map_command_t map_command;                           // 选手端发送数据
extern ext_map_robot_data_t map_robot_data;                     // 选手端接收雷达数据
extern ext_map_data_t map_data;                                 // 选手端接收哨兵或半自动数据
extern ext_custom_info_t custom_info;                           // 选手端接收机器人数据
extern ext_robot_interaction_data_t robot_interaction_data;     // 机器人交互数据
extern UART_HandleTypeDef *JudgeUSART;
extern HAL_StatusTypeDef IT_DMA_Begain(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
extern judge_receive_t judgement_receive;

extern uint8_t Shoot_Updata;

void Judge_Control_Init(UART_HandleTypeDef *huart);
void judgement_info_handle(void);
void unpack_fifo_handle(uint8_t *prxbuf);
void judgement_info_updata(void);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void judgement_data_decode(void);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
uint8_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void DMA_Tx2Gimbal(UART_HandleTypeDef *huart);

#endif

/*
1.������ȡ��DMA��������
2.�ڻص�������UART_IdleRxCallback���е��ô�������unpack_fifo_handle��
3.��������������judgement_race_data�ṹ���У�
*/