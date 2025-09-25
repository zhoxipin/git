#ifndef _VTM_INFO
#define _VTM_INFO

#include "usart.h"
#include "stdint.h"

#define Pictorial_SOF (uint8_t)0xA5

#define ROBOT_COMMAND_CMD_ID 0x0304

typedef struct
{
    uint8_t buf[70];
    uint8_t header[5];
    uint16_t cmd;
    uint8_t data[30];
    uint8_t tail[2];
} VTM_Receive_t;

typedef __packed struct
{
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    int8_t left_button_down;
    int8_t right_button_down;
    uint16_t keyboard_value;
    uint16_t reserved;
} ext_robot_command_t;

extern ext_robot_command_t robot_command;
extern VTM_Receive_t VTM_Receive;
extern UART_HandleTypeDef *VTM_USART;
extern uint8_t VTM_Data_Buf[12];
extern uint8_t VTM_Update;

void VTM_Info_Handler(void);
void unpack_VTM_FIFO_Handler(uint8_t *prxbuf);
void VTM_Data_Decode(void);
void VTM_Control_Init(UART_HandleTypeDef *huart);

#endif
