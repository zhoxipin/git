#include "vt03.h"

UART_HandleTypeDef *VT03_Usart;
uint8_t VT03_Receive_Buff[70];
vt03_rc_t VT03_RC;
vt03_vtm_t VT03_VTM;

void VT03_Init(UART_HandleTypeDef *huart)
{
    VT03_Usart = huart;

    USART_IDLE_Init(huart,VT03_Receive_Buff,30);
}

void VT03_FIFO_Handle(uint8_t *buff)
{
    Detect_Hook(VTM_TOE);

    if(*buff == VTM_SOFT)
        VT03_VTM_Decode(&VT03_VTM,buff);
    else if(*buff == RC_SOFT1 && *(buff+1) == RC_SOFT2)
        VT03_RC_Decode(&VT03_RC,buff);
}

void VT03_VTM_Decode(vt03_vtm_t *vtm,uint8_t *buff)
{
    static uint16_t VTM_data_length;
    memcpy(&VTM_data_length,(buff+1),2);

    if(VTM_data_length >= 30)
        return;

    if ((Verify_CRC8_Check_Sum(buff, 5) == 0) || (Verify_CRC16_Check_Sum(buff, (9 + VTM_data_length)) == 0))
        return;

    memcpy(vtm->header, buff, (VTM_data_length + 9));

    if(vtm->cmd[1]<<8|vtm->cmd[0] == 0x0304)
        memcpy(vtm,vtm->data, LENGTH_KEY_MOUSE);
}

void VT03_RC_Decode(vt03_rc_t *rc,uint8_t *buff)
{
    if(rc == NULL || buff == NULL)
        return;

    rc->soft_1 = buff[0];
    rc->soft_2 = buff[1];

    //CRC Check
    rc->crc16 = buff[19] | ((int16_t)buff[20]<<8);
    rc->crc16_flag = Verify_CRC16_Check_Sum(buff,RC_FRAME_LENGTH);

    if(rc->soft_1 == RC_SOFT1 && rc->soft_2 == RC_SOFT2 && rc->crc16_flag == 1)
    {
        rc->ch_0 = ((int16_t)buff[2] | (int16_t)buff[3]<<8) & 0x7FF;
        rc->ch_0 -= RC_CH_OFFSET;
        rc->ch_1 = ((int16_t)buff[3]>>3 | (int16_t)buff[4]<<5) & 0x7FF;
        rc->ch_1 -= RC_CH_OFFSET;
        rc->ch_2 = ((int16_t)buff[4]>>6 | (int16_t)buff[5]<<2 | (int16_t)buff[6]<<10) & 0x7FF;
        rc->ch_2 -= RC_CH_OFFSET;
        rc->ch_3 = ((int16_t)buff[6]>>1 | (int16_t)buff[7]<<7) & 0x7FF;
        rc->ch_3 -= RC_CH_OFFSET;

        rc->mode_sw = (buff[7]>>4) & 0x03;
        rc->go_home = (buff[7]>>6) & 0x01;
        rc->fn = (buff[7]>>7) & 0x01;

        rc->botton = buff[8] & 0x01;
        rc->whell = ((int16_t)buff[8]>>1 | (int16_t)buff[9]<<7) & 0x7FF;
        rc->whell -= RC_CH_OFFSET;
        rc->shutter = (buff[9]>>4) & 0x01;

        rc->mouse_x = buff[10] | ((int16_t)buff[11]<<8);
        rc->mouse_y = buff[12] | ((int16_t)buff[13]<<8);
        rc->mouse_z = buff[14] | ((int16_t)buff[15]<<8);

        rc->mouse_left = buff[16] & 0x03;
        rc->mouse_right = (buff[16]>>2) & 0x03;
        rc->mouse_middle = (buff[16]>>4) & 0x03;

        rc->key = buff[17] | ((int16_t)buff[18]<<8);
    }
    else
        return;
}

void VT03_Recover_DT7(vt03_rc_t *vt03,RC_Type *rc)
{
    rc->ch1 = vt03->ch_0;
    rc->ch2 = vt03->ch_1;
    rc->ch3 = vt03->ch_3;
    rc->ch4 = vt03->ch_2;

    rc->switch_left = Switch_Middle;
    rc->switch_right = Switch_Middle;

    rc->mouse.x = vt03->mouse_x;
    rc->mouse.y = vt03->mouse_y;
    rc->mouse.z = vt03->mouse_z;

    rc->mouse.press_right = vt03->mouse_right;
    rc->mouse.press_left = vt03->mouse_left;
    rc->key_code = vt03->key;

    RC_Data_Buffer[0] = 0x0;
    RC_Data_Buffer[1] = 0x4;
    RC_Data_Buffer[2] = 0x20;
    RC_Data_Buffer[3] = 0x0;
    RC_Data_Buffer[4] = 0x01;
    RC_Data_Buffer[5] = 0xF8;
    RC_Data_Buffer[14] = rc->key_code;
    RC_Data_Buffer[15] = rc->key_code >> 8;

    VTM_Update = 1;
}