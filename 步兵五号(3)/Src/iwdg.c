#include "iwdg.h"

IWDG_HandleTypeDef IWDG_Handler; //独立看门狗句柄 

//prer:分频数:IWDG_PRESCALER_4~IWDG_PRESCALER_256 
//rlr:自动重装载值,0~0XFFF. 
//时间计算:Tout=((4*2^prer)*rlr)/32 (ms). 
//LSI时钟32kHz,看门狗时间2*500ms = 1s

void IWDG_Init(uint8_t prer,uint16_t rlr) 
{     
	IWDG_Handler.Instance=IWDG;     
	IWDG_Handler.Init.Prescaler=prer;   //设置 IWDG 分频系数     
	IWDG_Handler.Init.Reload=rlr;   	//重装载值     
	HAL_IWDG_Init(&IWDG_Handler);
}   


void IWDG_Feed(void) 
{       
	 HAL_IWDG_Refresh(&IWDG_Handler);  //重装载 
} 
