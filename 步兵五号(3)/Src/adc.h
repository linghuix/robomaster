#ifndef __ADC_H
#define __ADC_H
#include "stm32f4xx_hal.h"
#include "main.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_adc_ex.h"
	

//void  MY_ADC_DMA_Init(void); 		  
//void  Get_Adc_Average(uint8_t times);
//uint16_t Get_Adc(ADC_HandleTypeDef* AdcHandle,uint8_t times);
//extern ADC_HandleTypeDef ADC1_Handler;
//extern ADC_HandleTypeDef ADC2_Handler;
//extern DMA_HandleTypeDef DMA2_Handler;
//extern uint16_t ADCReadings[2]; 
//extern uint16_t ADCValue[2

void MY_ADC_Init(void); 				//ADCͨ����ʼ��
uint16_t  Get_Adc(uint32_t ch); 		        //���ĳ��ͨ��ֵ 
void Get_Adc_Average(uint16_t *temp,uint8_t times);//�õ�ĳ��ͨ����������������ƽ��ֵ

extern ADC_HandleTypeDef ADC1_Handler;//ADC���
extern uint16_t ADC_DMA_ConvertedValue[2];

#endif 
