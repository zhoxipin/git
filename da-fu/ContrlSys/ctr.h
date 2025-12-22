#ifndef __ctr_H__
#define __ctr_H__

#include "tim.h"
#include "WS2812.h"
#include "dma.h"

//灯条结构体
typedef struct{
	uint8_t mode;
	uint8_t last_mode;
	TIM_HandleTypeDef *tim;
	uint8_t Channel;
  uint8_t last;
  uint8_t next;
}Fan_t;

//能量机关结构体
typedef struct{
	Fan_t fan[6];
	uint32_t timeStamp;//切换
  uint32_t last_mode_timeStamp;//亮
  uint32_t Time_difference; // the mode continue time unit as s
	uint8_t state;
	uint8_t last_state;
	uint8_t HitErrFlag;
	uint8_t need_hit;
	uint8_t no_first_hid;
	uint8_t StateSwitch_flag;
	uint8_t Alread_hit[5];
	uint8_t count;
  uint8_t last_need_hit;
  uint8_t status_same;
}EnergySys;

typedef enum
{
    wait_activate = 0,   //等待激活（0个叶片激活）
    activate_1,          //激活1个灯片
    activate_2,          //激活2个灯片
    activate_3,          //激活3个灯片
    activate_4,          //激活4个灯片
    success_activate,    //成功激活（5个都激活）
    forbid_activate
} EnergyMechanismState;

typedef enum
{
    little = 0,
    big = 1
} EnergyMechanismMode;

typedef enum
{
    OFF = 0,   //灭
    ON ,       //亮
    Flow       //流动
} FanMode;


void close_all(void);
void light_on(int RGB);

void light_flow(uint8_t s, uint32_t RGB, uint8_t flag) ;

void fan_init(void);
void led_circle_Init(int RGB);
void line_init(int RGB);
void ctr_fan(uint8_t s,int RGB,int fan_change_time);
void ctr_fan1(uint8_t s, int RGB,int fan_change_time);
void target_init(void);
void fan_logic(uint8_t s);
void fan_list_init();
void fan_light(uint8_t flag,uint8_t light_num);
inline void del_list(uint8_t num);
void Test(int s,int RGB,int now_light);
void Test1(int s, int RGB);
void Test2(int s, int RGB);
//inline uint8_t rand_num(uint8_t s);
void boot_init();
// void control_template_buff(uint16_t left,uint16_t right,uint32_t value); // control the base fan
// void control_template_bit(uint16_t left,uint16_t right,uint32_t bitmask); // control the base fan
void ALL_SetBuf(void);
inline void clear_all(void);
inline void update_all(void);
inline void LED_YUAN_Init(uint32_t Color);
void draw_arrows(uint8_t flag,uint8_t Step,uint32_t Color);
void MY_Delay(uint32_t Delay);

void Test3(int s, int RGB);//全亮

extern int disturbance;
#endif