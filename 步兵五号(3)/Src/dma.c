/**
  ******************************************************************************
  * File Name          : dma.c
  * Description        : This file provides code for the configuration
  *                      of all the requested memory to memory DMA transfers.
  ******************************************************************************
  *
  * Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "dma.h"
#include "adc.h"
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure DMA                                                              */
/*----------------------------------------------------------------------------*/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/* USER CODE BEGIN 2 */

DMA_HandleTypeDef  ADC1DMA_Handler;      //DMA句柄

//DMAx的各通道配置
//这里的传输形式是固定的,这点要根据不同的情况来修改
//从存储器->外设模式/8位数据宽度/存储器增量模式
//DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7
//chx:DMA通道选择,@ref DMA_channel DMA_CHANNEL_0~DMA_CHANNEL_7
void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx,uint32_t chx)
{ 
	if((uint32_t)DMA_Streamx>(uint32_t)DMA2)//得到当前stream是属于DMA2还是DMA1
	{
        __HAL_RCC_DMA2_CLK_ENABLE();//DMA2时钟使能	
	}else 
	{
        __HAL_RCC_DMA1_CLK_ENABLE();//DMA1时钟使能 
	}
    
    __HAL_LINKDMA(&ADC1_Handler,DMA_Handle,ADC1DMA_Handler);    //将DMA与ADC1联系起来(发送DMA)
    
    //Tx DMA配置
    ADC1DMA_Handler.Instance=DMA_Streamx;                            //数据流选择
    ADC1DMA_Handler.Init.Channel=chx;                                //通道选择
    ADC1DMA_Handler.Init.Direction=DMA_PERIPH_TO_MEMORY;             //外设到存储器
    ADC1DMA_Handler.Init.PeriphInc=DMA_PINC_DISABLE;                 //外设非增量模式
    ADC1DMA_Handler.Init.MemInc=DMA_MINC_ENABLE;                     //存储器增量模式
    ADC1DMA_Handler.Init.PeriphDataAlignment=DMA_PDATAALIGN_HALFWORD;    //外设数据长度:16位
    ADC1DMA_Handler.Init.MemDataAlignment=DMA_PDATAALIGN_HALFWORD;       //存储器数据长度:16位
    ADC1DMA_Handler.Init.Mode=DMA_CIRCULAR;                            //循环模式
    ADC1DMA_Handler.Init.Priority=DMA_PRIORITY_MEDIUM;               //中等优先级
    ADC1DMA_Handler.Init.FIFOMode=DMA_FIFOMODE_DISABLE;              
    ADC1DMA_Handler.Init.FIFOThreshold=DMA_FIFO_THRESHOLD_FULL;      
    ADC1DMA_Handler.Init.MemBurst=DMA_MBURST_SINGLE;                 //存储器突发单次传输    我理解的是每次开启DMA可以传输的数据
    ADC1DMA_Handler.Init.PeriphBurst=DMA_PBURST_SINGLE;              //外设突发单次传输
    
    HAL_DMA_DeInit(&ADC1DMA_Handler);   
    HAL_DMA_Init(&ADC1DMA_Handler);
} 


//开启一次DMA传输
//pData：传输的数据指针
//Size:传输的数据量
void MYDMA_ADC1_Transmit(ADC_HandleTypeDef *huart, uint32_t *pData, uint16_t Size)
{
    HAL_DMA_Start(huart->DMA_Handle, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);//开启DMA传输
    HAL_ADC_Start_DMA(huart,(uint32_t *)pData, Size);  //此函数的开启时间可能影响最终的结果，如果不行，就率先开启这个函数
}	  

 
/* USER CODE END 2 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
