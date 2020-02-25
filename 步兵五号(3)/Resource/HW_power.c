#include "HW_power.h"

//uint8_t temp[2];


/*Power信号串口接收数据*/
uint8_t power_head[2];//信号包头帧
uint8_t power_state = 0;//接收状态机
uint8_t PowerRxBuf[9];
uint8_t chassisoutput_ratio;//底盘输出功率比例,用于控制底盘输出功率


/*ADC*/
float ADC_datax;
float ADC_datay;
uint8_t ADC_datax_raw[2];
uint8_t ADC_datay_raw[2];

/**
 * @brief: 接收power串口数据。头标志0xaabb
 */

void POWER_USART_IRQHandler()
{
	uint32_t isrflags = READ_REG(huart2.Instance->SR);  //串口中断标志寄存器
	uint32_t cr1its   = READ_REG(huart2.Instance->CR1); //串口使能寄存器
	
	//上溢错误中断，直接返回
//    if(__HAL_UART_GET_FLAG(&NUC_UART_HANDLER, UART_FLAG_ORE) != RESET)
//	{
//		return;
//	}
	//判断为接收中断并且此时中断打开
	if(((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET)){
		if(power_state ==0)
		{
			power_head[0] = (uint8_t)(huart2.Instance->DR & 0x00FF);
			if(power_head[0] == 0xaa)
			{
				power_state = 1;
			}
			else
			{
				power_state = 0;
			}
		}
		else if(power_state == 1)
		{
			power_head[1] = (uint8_t)(huart2.Instance->DR & 0x00FF);
			if(power_head[1] == 0xbb)
			{
				power_state = 2;
			}
			else
			{
				power_state = 0;
			}
		}
		else if(power_state == 2)
		{
			chassisoutput_ratio = (uint8_t)(huart2.Instance->DR & 0x00FF);
			power_state = 3;
		}
		else if(power_state == 3)
		{
			ADC_datax_raw[0] = (uint8_t)(huart2.Instance->DR & 0x00FF);
			power_state = 4;
		}
		else if(power_state == 4)
		{
			ADC_datax_raw[1] = (uint8_t)(huart2.Instance->DR & 0x00FF);
			power_state = 5;
		}
		else if(power_state == 5)
		{
			ADC_datay_raw[0] = (uint8_t)(huart2.Instance->DR & 0x00FF);
			power_state = 6;
		}
		else if(power_state == 6)
		{
			ADC_datay_raw[1] = (uint8_t)(huart2.Instance->DR & 0x00FF);
			power_state = 0;
		}
		else
		{
			power_state = 0;
		}
	}

	if(chassisoutput_ratio>100)
		chassisoutput_ratio = 100;
	if(chassisoutput_ratio<0)
		chassisoutput_ratio = 0;
	
    ADC_process();
	
    __HAL_UART_CLEAR_PEFLAG(&huart2);        //清除一下各种标志位
	
    HAL_UART_Receive_IT(&huart2,PowerRxBuf , 1);  //再打开中断接收一字节
}



void ADC_process()
{
    ADC_datax = (ADC_datax_raw[0]*256+ADC_datax_raw[1]);
	ADC_datay = (ADC_datay_raw[0]*256+ADC_datay_raw[1]);

	ADC_datax = 29.988*pow(ADC_datax*3.3/4096,-1.173);
	ADC_datay = 29.988*pow(ADC_datay*3.3/4096,-1.173);
}

