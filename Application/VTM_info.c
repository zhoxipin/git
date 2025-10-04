#include "VTM_info.h"
#include "judgement_info.h"
#include "bsp_usart_idle.h"
#include "includes.h"
#include "bsp_CAN.h"

#include "string.h"

VTM_Receive_t VTM_Receive;

UART_HandleTypeDef *VTM_USART;

static uint8_t *p_header, count;
static uint16_t data_length;
uint8_t VTM_Data_Buf[12];
uint8_t VTM_Update;
uint8_t VTM_Flag;

ext_robot_command_t robot_command;

void VTM_Control_Init(UART_HandleTypeDef *huart)
{
    VTM_USART = huart;
    USART_IDLE_Init(huart, VTM_Receive.buf, 30);
}

void unpack_VTM_FIFO_Handler(uint8_t *prxbuf)
{
    p_header = prxbuf; //找到帧头然后处理
    count = 0;
    while (count <= 30)
    {
        if (*p_header == Pictorial_SOF)
        {
            VTM_Info_Handler();
        }
        else
            p_header++;
        count = p_header - prxbuf;
    }
}

void VTM_Info_Handler(void)
{
    memcpy(&data_length, (p_header + 1), 2);
    if (data_length >= 30)
    {
        p_header++;
        return;
    }
    if ((Verify_CRC8_Check_Sum( p_header, 5) == 0) || (Verify_CRC16_Check_Sum(p_header, (9 + data_length)) == 0))
    {
        p_header++;
        return;
    }

    memcpy(VTM_Receive.header, (p_header - 1), (data_length + 9));
    VTM_Data_Decode();
    p_header = p_header + data_length + 1;
}

void VTM_Data_Decode(void)
{
    switch (VTM_Receive.cmd)
    {
    case ROBOT_COMMAND_CMD_ID:
    {
        Detect_Hook(VTM_TOE);
        memcpy(&robot_command, VTM_Receive.data, sizeof(ext_robot_command_t));
        if (is_TOE_Error(RC_TOE))
        {
            remote_control.switch_left = Switch_Middle;
            remote_control.switch_right = Switch_Middle;
            remote_control.mouse.x = robot_command.mouse_x;
            remote_control.mouse.y = robot_command.mouse_y;
            remote_control.mouse.z = robot_command.mouse_z;
            remote_control.mouse.press_right = robot_command.right_button_down;
            remote_control.mouse.press_left = robot_command.left_button_down;
            remote_control.key_code = robot_command.keyboard_value;

            memcpy(VTM_Data_Buf, &robot_command, 12);
            VTM_Update = 1;
        }
    }
    break;
    }
}
