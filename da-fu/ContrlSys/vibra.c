#include "vibra.h"
#include "ctr.h"
#include "gpio.h"

int vibra_flag[5];

int disturbance_flag[5] = {0,0,0,0,0};

extern EnergySys ES;

/*
*ï¿½ï¿½ï¿½ï¿½Ëµï¿½ï¿½
*VB1-PA4(ï¿½ï¿½1) VB2-PA6(ï¿½ï¿½2) VB3-PA5(ï¿½ï¿½4) VB4-PA7(ï¿½ï¿½3) VB5-PC4(ï¿½ï¿½5)
*ï¿½ï¿½ï¿½ï¿½Î´ï¿½ï¿½Ó²ï¿½ï¿½Ô­ï¿½ï¿½Í¼ï¿½ï¿½ï¿½ñ¶¯ºÍ¶ï¿½Ó¦ï¿½ï¿½Ç©ï¿½ï¿½Ò»ï¿½ï¿½
*
**/

/*
*
*ï¿½ñ¶¯´ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ÅºÅ³ï¿½Ê¼ï¿½ï¿½
*
*
**/

//ÖØÖÃ¸ÉÈÅ¼ì²â
void disturbance_init(void)
{
  for(uint8_t i=0;i<=4;i++)
	disturbance_flag[i]=0;
	disturbance = 0;
}

//Õñ¶¯¼ì²â³õÊ¼»¯
void vibra_init(void)
{
  for(uint8_t i=0;i<=4;i++)
	vibra_flag[i]=0;
}

//Õñ¶¯¼ì²â
void Vibra_monitor(void)
{

	if(HAL_GPIO_ReadPin(Vibration0_GPIO_Port,Vibration0_Pin) == SET){
		vibra_flag[0]=1;
		if(ES.fan[0].mode == ON)
		{
		disturbance_flag[0]++;
		}
	}
	
	
	if(HAL_GPIO_ReadPin(Vibration3_GPIO_Port,Vibration3_Pin) == SET){
		vibra_flag[1]=1;	
		if(ES.fan[1].mode == ON)
		{
		disturbance_flag[1]++;
		}
	}
	
	if(HAL_GPIO_ReadPin(Vibration1_GPIO_Port,Vibration1_Pin) == SET){
		vibra_flag[2]=1;
		if(ES.fan[2].mode == ON)
		{
		disturbance_flag[2]++;
		}
	}
	
	if(HAL_GPIO_ReadPin(Vibration2_GPIO_Port,Vibration2_Pin) == SET){
		vibra_flag[3]=1;
	//	HAL_GPIO_WritePin( Vibration2_GPIO_Port,Vibration2_Pin ,GPIO_PIN_SET  );
		if(ES.fan[3].mode == ON)
		{
		disturbance_flag[3]++;
		}
	}
	
	
	if(HAL_GPIO_ReadPin(Vibration4_GPIO_Port,Vibration4_Pin) == SET){
		vibra_flag[4]=1;
		if(ES.fan[4].mode == ON)
		{
		disturbance_flag[4]++;
		}
	}	
}