#include "HW_steer.h"

uint8_t STEER_PERIOD = 100;
uint8_t STEER_DUTY_RATIO  = 20;
uint8_t steer_count = 0;



void steer_task()
{
	steer_count++;
	if(steer_count>STEER_PERIOD)
	{
		steer_count = 0;
	}
	if((steer_count>=0)&&(steer_count<=STEER_DUTY_RATIO))
	{
		Set_steer_high();
	}
	else if((steer_count<=STEER_PERIOD)&&(steer_count>STEER_DUTY_RATIO))
	{
		Set_steer_low();
	}
}

void Set_Steer_Duty(uint8_t duty)
{
	STEER_DUTY_RATIO = duty;
}

void Set_steer_high()
{
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_SET);
}


void Set_steer_low()
{
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_RESET);
}

void Steer_Init()
{
	
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin = GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	//HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_SET);
}
