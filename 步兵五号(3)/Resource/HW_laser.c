/**************************************************
* @file      HW_laser.c
* @author    wwy
* @version   supposed final version
* @date      2019.1.29
* @brief     初始化激光瞄准，激光瞄准开关
***************************************************
* @attention
使用PA0作为IO，IO口的初始化写在这里
***************************************************
*/

/* Include ---------------------------------------*/
#include "HW_laser.h"

/* Global Variable -----------------------------*/

/**
* @brief  激光瞄准初始化
* @param  none
* @retval none
* @note
**/
void Laser_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.Pin   = LASER_PIN;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LASER_GPIO_PORT, &GPIO_InitStruct);
}

/**
* @brief  激光瞄准开
* @param  none
* @retval none
* @note
**/
void Laser_Turn_ON(void)
{
	HAL_GPIO_WritePin(LASER_GPIO_PORT, LASER_PIN, GPIO_PIN_SET);
}

/**
* @brief  激光瞄准关
* @param  none
* @retval none
* @note
**/
void Laser_Turn_OFF(void)
{
	HAL_GPIO_WritePin(LASER_GPIO_PORT, LASER_PIN, GPIO_PIN_RESET);
}

	