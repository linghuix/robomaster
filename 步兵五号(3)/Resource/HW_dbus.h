/**
  ********************************(C) COPYRIGHT 2017 DJI************************
  * @file       bsp_uart.h
	* @brief      uart receive data from DBus/bt/judge_system/manifold etc.
	* @update
  * @note       use DMA receive, but donot trigger DMA interrupt
	*             handle received data in usart idle interrupt handle function
	*	@history
	* Version     Date          Author           Modification
  * V1.0.0      Jun-01-2017   Richard.luo      remove some useless module
  * @verbatim
	*		idle interrupt --> handle data --> clear it flag --> initialize DMA again
	*
	********************************(C) COPYRIGHT 2017 DJI************************
	*/

#ifndef __BSP_UART_H__
#define __BSP_UART_H__

//#include "sys.h"
#include "usart.h"
#include "string.h"

#define DBUS_HUART huart1
#define DBUS_RX_MAX_BUFLEN	21
#define MAX_DMA_COUNT		100
#define RC_UP 1
#define RC_DN 2
#define RC_MI 3
#pragma pack()

//extern uint32_t tick_controller;


/**
 * 控制接收结构体
 */
typedef struct {

/** 航模控制器 */
	float LV;
	float LH;//航模控制器 左 水平方向  -1 ~ +1
	float RV;//航模控制器 右 垂直方向
	float RH;
	
	//ch value: -660 ~ 660
	int16_t ch1;	
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;
	
	//switch val: 1 3 2
	uint8_t sw1;//航模控制器 左
	uint8_t sw2;//航模控制器 右

/** 电脑控制 */
	struct {
		int16_t x;
		int16_t y;
		int16_t z;//no use
	
		uint8_t l;//left  press:1 release:0
		uint8_t r;
	}mouse;	//鼠标，电脑控制
	
	union {
		uint16_t key_code;
/**********************************************************************************
 * keyboard :15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
 *            V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
 ************************************************************************************/
		struct {
			uint16_t W:1;
			uint16_t S:1;
			uint16_t A:1;
			uint16_t D:1;
			uint16_t SHIFT:1;
			uint16_t CTRL:1;
			uint16_t Q:1;
			uint16_t E:1;
			uint16_t R:1;
			uint16_t F:1;
			uint16_t G:1;
			uint16_t Z:1;
			uint16_t X:1;
			uint16_t C:1;
			uint16_t V:1;
			uint16_t B:1;
		}bit;
		
	}kb;	//keyboard 键盘
	
	int16_t wheel;
	
}RC_Type;

extern RC_Type rc;
extern RC_Type last_rc;
extern uint32_t rc_tick;

void Dbus_Init(void);
void Uart_Receive_Handler(UART_HandleTypeDef *huart);

#endif
