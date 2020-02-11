/**************************************************
* @file      HW_dbus.c
* @author    not wwy
* @version   version 1.0
* @date      2019.1.29
* @brief     遥控器接收，用DMA
***************************************************
* @attention
使用USART1，IO口为PB6 7，这些初始化在usart.c里完成
存储使用DMA2,通道4，无需配置，主函数初始化即可
***************************************************
*/

/* Include ---------------------------------------*/

#include "HW_dbus.h"
#include "iwdg.h"
#define  RC_OFFSET 660.0f

/* Global Variable -----------------------------*/
RC_Type  rc;                                     //当前键鼠、遥控器数据
RC_Type  last_rc;                                //上次键鼠、遥控器数据
uint8_t  dbus_buff[DBUS_RX_MAX_BUFLEN];
uint32_t rc_tick;

void Uart_Callback_RC_Handle(RC_Type *rc, uint8_t *buff)
{
	IWDG_Feed();
	rc_tick = HAL_GetTick();
    if (buff[0] == 0 && buff[1] == 0 && buff[2] == 0 && buff[3] == 0 && buff[4] == 0 && buff[5] == 0)
        return;

    rc->ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
    rc->ch1 -= 1024;
    rc->ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
    rc->ch2 -= 1024;
    rc->ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
    rc->ch3 -= 1024;
    rc->ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
    rc->ch4 -= 1024;
		
	rc->RH = rc->ch1/RC_OFFSET;
	rc->RV = rc->ch2/RC_OFFSET;
	rc->LH = rc->ch3/RC_OFFSET;
	rc->LV = rc->ch4/RC_OFFSET;

    rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
    rc->sw2 = (buff[5] >> 4) & 0x0003;

    rc->mouse.x = buff[6] | (buff[7] << 8); // x axis
    rc->mouse.y = buff[8] | (buff[9] << 8);
    rc->mouse.z = buff[10] | (buff[11] << 8);

    rc->mouse.l = buff[12];
    rc->mouse.r = buff[13];

    rc->kb.key_code = buff[14] | buff[15] << 8; // key borad code
	rc->wheel = buff[16] | buff[17] << 8;
	
//	tick[eController] = HAL_GetTick();
}

/**
  * @brief   clear idle it flag after uart receive a frame data
  * @param   uart IRQHandler id
  * @retval  none
  * @usage   call in uart_receive_handler() function
  */
void uart_reset_idle_rx_callback(UART_HandleTypeDef* huart)
{
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
	{
		
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		// clear idle it flag
		uint32_t DMA_FLAGS = __HAL_DMA_GET_TC_FLAG_INDEX(huart->hdmarx); 
		//according uart clear corresponding DMA flag

		__HAL_DMA_DISABLE(huart->hdmarx);
		__HAL_DMA_CLEAR_FLAG(huart->hdmarx, DMA_FLAGS);
		__HAL_DMA_SET_COUNTER(huart->hdmarx, MAX_DMA_COUNT);
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
}

/**
  * @brief   callback this function when uart interrupt 
  * @param   uart IRQHandler id
  * @retval  none
  * @usage   call in uart handler function USARTx_IRQHandler()
  */
void Uart_Receive_Handler(UART_HandleTypeDef *huart)
{
	if (huart == &DBUS_HUART)
	{
		Uart_Callback_RC_Handle(&rc, dbus_buff);
	}
	uart_reset_idle_rx_callback(huart);
}




/**
  * @brief   enable global uart it and do not use DMA transfer done it
  * @param   uart IRQHandler id, receive buff, buff size
  * @retval  set success or fail
  */
static int UART_Receive_DMA_No_IT(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{
	uint32_t tmp1 = 0;

	tmp1 = huart->RxState;
	if (tmp1 == HAL_UART_STATE_READY)
	{
		if ((pData == NULL) || (Size == 0))
		{
				return HAL_ERROR;
		}

		/* Process Locked */
		__HAL_LOCK(huart);

		huart->pRxBuffPtr = pData;
		huart->RxXferSize = Size;
		huart->ErrorCode  = HAL_UART_ERROR_NONE;

		/* Enable the DMA Stream */
		HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR,
									(uint32_t)pData, Size);

		/* Enable the DMA transfer for the receiver request by setting the DMAR bit
		in the UART CR3 register */
		huart->Instance->CR3 |= USART_CR3_DMAR;

		/* Process Unlocked */
		__HAL_UNLOCK(huart);

		return HAL_OK;
	}
	else
	{
		return HAL_BUSY;
	}
}



/**
  * @brief   initialize uart device 
  * @usage   after MX_USARTx_UART_Init() use these function
  */
void Dbus_Init()
{
    __HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
    __HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);
    // clear idle it flag & open idle it
    UART_Receive_DMA_No_IT(&DBUS_HUART, dbus_buff, DBUS_RX_MAX_BUFLEN);
}

