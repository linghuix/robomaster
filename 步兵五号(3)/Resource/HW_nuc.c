/**************************************************
* @file      HW_nuc.c
* @author    wwy
* @version   bankrupt_version
* @date      2019.1.13
* @brief     通信成功，角度结算成功
***************************************************
* @attention
使用USART3，IO为PD8,PD9，这些初始化在usart.c里完成
***************************************************
*/

/* Include ---------------------------------------*/
#include "HW_nuc.h"
#include "filter.h"
extern float imu_yaw;

/* Global Variable -----------------------------*/
uint8_t NucRxBuf[9];
short temp1, temp2, temp3, temp4;
short pre_temp3;
extern int   IMUerro;
extern uint8_t nuc_send_auto_flag;
//
float IMU_yaw;
//int IMU_yaw_Detect;
int nuc_flag=1;
int nuc_update_flag = 0;


/**/
int mi_f = 0;
int mi_b = 0;

/**
* @brief  NUC与单片机通信初始化
* @param  none
* @retval none
* @note
**/

uint8_t isNUC;



void NUC_Init(void)
{
	__HAL_UART_ENABLE_IT(&NUC_UART_HANDLER,UART_IT_RXNE);
}

/**
* @brief  NUC串口中断处理函数
* @param  none
* @retval none
* @note   1.12 上溢错误中断有问题，先注释掉，最后两行一定要加
**/
void NUC_USART_IRQHandler(void)
{
	uint8_t byte_data;                                            //接收到一字节数据
	uint32_t isrflags = READ_REG(NUC_UART_HANDLER.Instance->SR);  //串口中断标志寄存器
	uint32_t cr1its   = READ_REG(NUC_UART_HANDLER.Instance->CR1); //串口使能寄存器
	
	//上溢错误中断，直接返回
//    if(__HAL_UART_GET_FLAG(&NUC_UART_HANDLER, UART_FLAG_ORE) != RESET)
//	{
//		return;
//	}
	//判断为接收中断并且此时中断打开
	if(((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
	{
		byte_data = (uint8_t)(NUC_UART_HANDLER.Instance->DR & 0x00FF);
		//if(Mode.Shoot == Shoot_Auto_Mode)
			NUC_Receive_Check(byte_data);
		
	}
	
	__HAL_UART_CLEAR_PEFLAG(&NUC_UART_HANDLER);        //清除一下各种标志位
	HAL_UART_Receive_IT(&NUC_UART_HANDLER, NucRxBuf, 1);  //再打开中断接收一字节
	
}
/**
* @brief 接收NUC发来的数据
* @param uint8_t data 发来的一个字节的数据
* @note
**/
void NUC_Receive_Check(uint8_t data)
{
	static uint8_t RxCount;                              //计数用
	static uint8_t Rxstate;                              //状态机状态变量
	
	if(Rxstate == 0 && data == NUC_UART_RxHEAD1)         //帧头第一个字节
	{
		Rxstate = 1;
		NucRxBuf[0] = data;
	}
	else if(Rxstate == 1 && data == NUC_UART_RxHEAD2)    //帧头第二个字节
	{
		Rxstate = 2;
		NucRxBuf[1] = data;
	}
	else if(Rxstate == 2 && data == NUC_UART_RxHEAD3)    //帧头第三个字节
	{
		Rxstate = 3;
		NucRxBuf[2] = data;
	}
	else if(Rxstate == 3)        //接收数据
	{
		NucRxBuf[RxCount+3] = data;                         
		RxCount++;
		if(RxCount == 6)
		{
			NUC_Receive_Process(NucRxBuf+3);
		  Rxstate = 0;
		  RxCount = 0;
		}
	}
  else
	{
		  Rxstate = 0;
		  RxCount = 0;
	}
	
}

/**
* @brief 对NUC发来的数据进行解析
* @param uint8_t * Buf 完整的一帧数据
**/


extern uint32_t time_1ms;            //1ms计数
int starttime;
int endtime;
int errotime;
float imu_yaw_buf[512];
uint8_t imu_yaw_buf_pos = 0;
int xerro1;
int xerro2;
uint32_t nuc_tick;


float pre_IMU_yaw;
float pre_temp1;
int   xerro;
extern float v;
void NUC_Receive_Process(uint8_t * Buf)
{
	isNUC  =  1;
	nuc_tick = HAL_GetTick();
  
	//要求颜色请求
	//if((Buf[0]==0xff)&&(Buf[1]==0xff)&&(Buf[2]==0xff)&&(Buf[3]==0xff)&&(Buf[4]==0xff)&&(Buf[5]==0xff))
	//{
	//	nuc_send_auto_flag = 1;
	//	temp1 = 0;
	//	temp2 = 0;
	//	temp3 = 0;
	//}
	//else
	//{
	if(Mode.Shoot == Shoot_Auto_Mode)
	{
		if(nuc_flag==1)
		{
			IMU_yaw=imu_yaw;
			pre_IMU_yaw=imu_yaw;
			pre_temp1=temp1;
			pre_temp3=temp3;
			nuc_flag=0;
		}
		starttime=endtime;
		endtime=time_1ms;
		errotime=endtime-starttime;
		temp1 = (Buf[0] << 8) | Buf[1];//x
		temp2 = (Buf[2] << 8) | Buf[3];//y
		temp3 = (Buf[4] << 8) | Buf[5];//z

	
		IMU_yaw=imu_yaw;

		IMU_yaw=imu_yaw_buf[(uint8_t)(imu_yaw_buf_pos-40)];
		v=(1000.0f*(atan(temp1*1.0f/temp3)-atan(pre_temp1*1.0f/pre_temp3)-(IMU_yaw-pre_IMU_yaw)/50.0f))/errotime;
		IMUerro=(IMU_yaw-pre_IMU_yaw)*1000;

		mi_f = IMU_yaw*1000;
		mi_b = pre_IMU_yaw*1000;

		pre_temp1=temp1;    ////////////////////////////////////////
		pre_IMU_yaw=IMU_yaw;
		pre_temp3=temp3;
		nuc_update_flag = 1;	
	}

	//}
}

