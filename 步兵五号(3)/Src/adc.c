#include "adc.h"


ADC_HandleTypeDef ADC1_Handler;//ADC句柄
uint16_t ADC_DMA_ConvertedValue[2];

//初始化ADC
//ch: ADC_channels 
//通道值 0~16取值范围为：ADC_CHANNEL_0~ADC_CHANNEL_16
void MY_ADC_Init(void)
{ 
	
	ADC_ChannelConfTypeDef ADC1_ChanConf;
	
    ADC1_Handler.Instance=ADC1;
    ADC1_Handler.Init.ClockPrescaler=ADC_CLOCK_SYNC_PCLK_DIV4;   //4分频，ADCCLK=PCLK2/4=90/4=22.5MHZ
    ADC1_Handler.Init.Resolution=ADC_RESOLUTION_12B;             //12位模式
    ADC1_Handler.Init.DataAlign=ADC_DATAALIGN_RIGHT;             //右对齐
    ADC1_Handler.Init.ScanConvMode=ENABLE;                      //开启扫描模式		
    ADC1_Handler.Init.EOCSelection=DISABLE;                      //关闭EOC中断
    ADC1_Handler.Init.ContinuousConvMode=DISABLE;                //关闭连续转换   如果将 CONT 位置 1，规则通道转换不会在组中最后一个所选通道处停止，而是再 次从第一个所选通道继续转换
    ADC1_Handler.Init.NbrOfConversion=2;                         //2个转换在规则序列中 也就是只转换规则序列2 
    ADC1_Handler.Init.DiscontinuousConvMode=DISABLE;             //禁止不连续采样模式
    ADC1_Handler.Init.NbrOfDiscConversion=0;                     //不连续采样通道数为0
    ADC1_Handler.Init.ExternalTrigConv=ADC_SOFTWARE_START;       //软件触发
    ADC1_Handler.Init.ExternalTrigConvEdge=ADC_EXTERNALTRIGCONVEDGE_NONE;//使用软件触发
    ADC1_Handler.Init.DMAContinuousRequests=ENABLE;             //开启DMA请求
    HAL_ADC_Init(&ADC1_Handler);                                 //初始化 
	
	
	ADC1_ChanConf.Channel=ADC_CHANNEL_3;                                   //通道
    ADC1_ChanConf.Rank=2;                                       //第2个序列，序列1
    ADC1_ChanConf.SamplingTime=ADC_SAMPLETIME_480CYCLES;        //采样时间
    ADC1_ChanConf.Offset=0;                 
    HAL_ADC_ConfigChannel(&ADC1_Handler,&ADC1_ChanConf);        //通道配置
	
    ADC1_ChanConf.Channel=ADC_CHANNEL_4;                                   //通道
    ADC1_ChanConf.Rank=1;         	//第1个序列，序列1   
    ADC1_ChanConf.SamplingTime=ADC_SAMPLETIME_480CYCLES; 	
    HAL_ADC_ConfigChannel(&ADC1_Handler,&ADC1_ChanConf);        //通道配置    
	
//	HAL_ADC_PollForConversion(&ADC1_Handler,10);                //轮询转换       DMA模式不使用这个函数
}

//ADC底层驱动，引脚配置，时钟使能
//此函数会被HAL_ADC_Init()调用
//hadc:ADC句柄
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_ADC1_CLK_ENABLE();            //使能ADC1时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();			//开启GPIOA时钟
	
    GPIO_Initure.Pin=GPIO_PIN_3 | GPIO_PIN_4;            //PA3 PA4
    GPIO_Initure.Mode=GPIO_MODE_ANALOG;     //模拟
    GPIO_Initure.Pull=GPIO_NOPULL;          //不带上下拉
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
}


//获取指定通道的转换值，取times次,然后平均 
//times:获取次数
//返回值:通道ch的times次转换结果平均值
void Get_Adc_Average(uint16_t *temp,uint8_t times)
{	
	uint32_t temp_val[2]={0};
	uint8_t t;
	for(t=0;t<times;t++)
	{
		HAL_ADC_Start(&ADC1_Handler);                               //开启ADC
		temp_val[0]+=ADC_DMA_ConvertedValue[0];
		temp_val[1]+=ADC_DMA_ConvertedValue[1];
	}
	 temp[0]=temp_val[0]/times;
	 temp[1]=temp_val[1]/times;
	 temp_val[0]=0;
	 temp_val[1]=0;
} 
