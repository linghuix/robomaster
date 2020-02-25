/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "HW_nuc.h"
#include "HW_power.h"
#include "HW_steer.h"
#include "HW_dbus.h"
#include "control_task.h"
#include "judge_task.h"
#include "adc.h"



uint16_t temp_v[2]; //???ADC?
uint16_t tim5_count = 0;

/*ADC 测量*/
float distance[2];//???????
uint16_t ADCone;
uint16_t ADCtwo;

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6; 

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/
void NMI_Handler(void)
{
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  while (1)
  {
  }
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  while (1)
  {
  }
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  while (1)
  {
  }
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}
/**
* @brief This function handles EXTI line1 interrupt.
*/
void EXTI1_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}

/**
* @brief This function handles CAN1 TX interrupts.
*/
void CAN1_TX_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan1);
}

/**
* @brief This function handles CAN1 RX0 interrupts.
*/
void CAN1_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan1);
}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart1);
  Uart_Receive_Handler(&huart1);
}

/**
* @brief This function handles USART2 global interrupt.
*/
   
void USART2_IRQHandler(void)
{
  //POWER_USART_IRQHandler();
	Judge_Task_UART_IRQHandler();
                                           //接收到一字节数据
}

/**
* @brief This function handles USART3 global interrupt.
*/
void USART3_IRQHandler(void)
{
    NUC_USART_IRQHandler();
}

/**
* @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
*/

void TIM2_IRQHandler(void)
{

	HAL_TIM_IRQHandler(&htim2);
}


void TIM5_IRQHandler(void)
{
	tim5_count++;
	steer_task();
	if(tim5_count>=10)
	{
		Get_Adc_Average(temp_v,20);   //???ADC??????temp_v[2]
		distance[0]=29.988*pow(temp_v[0]*3.3/4096,-1.173);
		distance[1]=29.988*pow(temp_v[1]*3.3/4096,-1.173);
		ADCone = distance[0];
		ADCtwo = distance[1];
		tim5_count = 0;
	}
	HAL_TIM_IRQHandler(&htim5);
}


void TIM6_DAC_IRQHandler(void)
{
    Control_Task();
    HAL_TIM_IRQHandler(&htim6);

}

/**
* @brief This function handles DMA2 stream2 global interrupt.
*/
void DMA2_Stream2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

/**
* @brief This function handles CAN2 TX interrupts.
*/
void CAN2_TX_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan2);
}

/**
* @brief This function handles CAN2 RX0 interrupts.
*/
void CAN2_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan2);
}

/**
* @brief This function handles USB On The Go FS global interrupt.
*/
void OTG_FS_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}

/**
* @brief This function handles USART6 global interrupt.
*/
void USART6_IRQHandler(void)
{
  Judge_Task_UART_IRQHandler();
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
