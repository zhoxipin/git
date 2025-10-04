#ifndef VT03_H
#define VT03_H

#include "usart.h"
#include "stdint.h"
#include "string.h"
#include "judgement_info.h"
#include "detect_task.h"
#include "VTM_info.h"
#include "remote_control.h"
#include "bsp_usart_idle.h"

#define RC_FRAME_LENGTH 21
#define LENGTH_KEY_MOUSE 12

#define RC_CH_OFFSET 1024

#define VTM_SOFT 0xA5
#define RC_SOFT1 0xA9
#define RC_SOFT2 0x53

#define CRC16_INIT 0xFFFF

#define Key_W ((uint16_t)0x01 << 0)
#define Key_S ((uint16_t)0x01 << 1)
#define Key_A ((uint16_t)0x01 << 2)
#define Key_D ((uint16_t)0x01 << 3)
#define Key_SHIFT ((uint16_t)0x01 << 4)
#define Key_CTRL ((uint16_t)0x01 << 5)
#define Key_Q ((uint16_t)0x01 << 6)
#define Key_E ((uint16_t)0x01 << 7)
#define Key_R ((uint16_t)0x01 << 8)
#define Key_F ((uint16_t)0x01 << 9)
#define Key_G ((uint16_t)0x01 << 10)
#define Key_Z ((uint16_t)0x01 << 11)
#define Key_X ((uint16_t)0x01 << 12)
#define Key_C ((uint16_t)0x01 << 13)
#define Key_V ((uint16_t)0x01 << 14)
#define Key_B ((uint16_t)0x01 << 15)


typedef __packed struct
{
    uint8_t soft_1;
    uint8_t soft_2;

    int16_t ch_0;
    int16_t ch_1;
    int16_t ch_2;
    int16_t ch_3;

    uint8_t mode_sw;
    uint8_t go_home;
    uint8_t fn;
    uint8_t botton;
    int16_t whell;
    uint8_t shutter;

    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;

    uint8_t mouse_left;
    uint8_t mouse_right;
    uint8_t mouse_middle;

    uint16_t key;
    uint16_t crc16;

    uint8_t crc16_flag; // 1-通过 0-未通过
} vt03_rc_t;

typedef __packed struct
{
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    int8_t left_button_down;
    int8_t right_button_down;
    uint16_t keyboard_value;
    uint16_t reserved;

    uint8_t buf[70];
    uint8_t header[5];
    uint8_t cmd[2];
    uint8_t data[30];
    uint8_t tail[2];
} vt03_vtm_t;

void VT03_Init(UART_HandleTypeDef *huart);
void VT03_FIFO_Handle(uint8_t *buff);
void VT03_VTM_Decode(vt03_vtm_t *vtm,uint8_t *buff);
void VT03_RC_Decode(vt03_rc_t *rc,uint8_t *buff);

void VT03_Recover_DT7(vt03_rc_t *vt03,RC_Type *rc);

extern UART_HandleTypeDef *VT03_Usart;
extern uint8_t VT03_Receive_Buff[70];
extern vt03_rc_t VT03_RC;
extern vt03_vtm_t VT03_VTM;
#endif