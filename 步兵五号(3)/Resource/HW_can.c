/**************************************************
* @file      HW_can.c
* @author    wwy
* @version   version 1.2
* @date      2019.2.19
* @brief     拨弹电机2006，减速比36:1  3508改减速比
***************************************************
* @attention
CAN1的IO为PD0,PD1,CAN2的IO为PB12,PB13，这些初始化在can.c里完成
for hero robot
***************************************************
*/

/* Include ---------------------------------------*/
#include "HW_can.h"
#include "gimbal_task.h"
/* Global Variable -----------------------------*/





//LF 左前 0
//RF 右前 1
//LB 左后 3
//RB 右后 4
uint32_t wheel_can_rc[4];	/*接收CAN的时间戳*/
uint32_t wheel_can_rcTimes[4];/*xlh测试每次控制循环中接收的次数*/



GMEncoder_t GMYawEncoder;
GMEncoder_t GMPitchEncoder;
CMEncoder_t CMEncoder[4];
SHOOTEncoder_t SHOOTEncoder[4];      //步兵少一个摩擦轮
int32_t feed_round = 0;             //拨弹轮旋转圈数
int32_t gimbal_round = 0;
int32_t gimbal_round_now;
uint32_t power_tick = 0;
int angle;
uint8_t first_time_flag;
extern uint8_t chassisoutput_ratio;
extern uint8_t ADC_datax_raw[2];
extern uint8_t ADC_datay_raw[2];

//电容部分
extern uint8_t power_chaisispower;
extern uint8_t isErro;
extern uint8_t isChange;
extern uint8_t RemainPower;
extern uint32_t isStop_tick;



/**
* @brief  can filter的初始化，直接抄的官方代码
* @param  can的结构体
* @retval none
* @note   必须在主函数中初始化
**/
void CanFilter_Init(CAN_HandleTypeDef* hcan) {
  CAN_FilterConfTypeDef canfilter;
  
  //create memory to save the message, if not will raise error
  static CanTxMsgTypeDef  Tx1Message;
  static CanRxMsgTypeDef  Rx1Message;
  static CanTxMsgTypeDef  Tx2Message;
  static CanRxMsgTypeDef  Rx2Message;
  
  canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
  
  //filtrate any ID you want here
  canfilter.FilterIdHigh = 0x0000;
  canfilter.FilterIdLow = 0x0000;
  canfilter.FilterMaskIdHigh = 0x0000;
  canfilter.FilterMaskIdLow = 0x0000;
  
  canfilter.FilterFIFOAssignment = CAN_FilterFIFO0;
  canfilter.FilterActivation = ENABLE;
  canfilter.BankNumber = 14;
  
  //use different filter for can1&can2
  if(hcan == &hcan1)
  {
    canfilter.FilterNumber = 0;
    hcan->pTxMsg = &Tx1Message;
    hcan->pRxMsg = &Rx1Message;
  }
  if(hcan == &hcan2)
  {
    canfilter.FilterNumber = 14;
    hcan->pTxMsg = &Tx2Message;
    hcan->pRxMsg = &Rx2Message;
  }
  
  HAL_CAN_ConfigFilter(hcan, &canfilter);
  
}

/**
* @brief  can中断的回调函数,解析全部在这里
* @param  can的结构体
* @retval none
* @note   注意拨弹部分步兵和英雄用的电机不同，步兵要做一点修改
**/
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
  uint8_t index;           //作几个摩擦轮底盘电机的索引用
  if(hcan == &hcan1)
  {
    switch(hcan->pRxMsg->StdId)
    {
      case GIMBAL_PITCHRxID:      //Pitch轴角度解析
      {
        GMPitchEncoder.last_angle_raw_value = GMPitchEncoder.angle_raw_value;
        GMPitchEncoder.angle_raw_value = (hcan->pRxMsg->Data[0] << 8 | hcan->pRxMsg->Data[1]);
		    GMPitchEncoder.real_angle = GMPitchEncoder.angle_raw_value * 1.0 / 8191 * 360;
      }
      break;
	  case GIMBAL_YAWRxID:        //Yaw轴角度解析
      {
		//			GMYawEncoder.last_angle_raw_value = GMYawEncoder.angle_raw_value;
		//      GMYawEncoder.angle_raw_value = (hcan->pRxMsg->Data[0] << 8 | hcan->pRxMsg->Data[1]);
		//	    GMYawEncoder.real_angle = GMYawEncoder.angle_raw_value * 1.0 / 8191 * 360;
		if(first_time_flag)
			GMYawEncoder.last_angle_raw_value = GMYawEncoder.angle_raw_value;
				
        GMYawEncoder.angle_raw_value = (hcan->pRxMsg->Data[0] << 8 | hcan->pRxMsg->Data[1]);
        GMYawEncoder.rotate_speed = (hcan->pRxMsg->Data[2] << 8 | hcan->pRxMsg->Data[3]);
		
		if(first_time_flag == 0)
		{
			//GMYawEncoder.last_angle_raw_value = 4000;
			first_time_flag = 1;
			if(GMYawEncoder.angle_raw_value>4000)
				gimbal_round = 0;
			else if(GMYawEncoder.angle_raw_value<4000)
				gimbal_round = 0;
		}
		
		if(GMYawEncoder.angle_raw_value < 2000 && GMYawEncoder.last_angle_raw_value > 6000)
		{
			gimbal_round++;
		}
		else if(GMYawEncoder.angle_raw_value > 6000 && GMYawEncoder.last_angle_raw_value < 2000)
		{
			gimbal_round--;
		}
				
        GMYawEncoder.angular_velocity = 2 * PI * GMYawEncoder.rotate_speed / 60;
		GMYawEncoder.real_angle = GMYawEncoder.angle_raw_value + 8191 * gimbal_round;
		GMYawEncoder.real_angle = GMYawEncoder.real_angle * 0.0439453125f - YAW_OFFSET_ANGLE;
		//GMYawEncoder.real_angle = GMYawEncoder.angle_raw_value*0.0439453125f - YAW_OFFSET_ANGLE;
	}
      break;
      case FEED_RxID:             //拨弹电机、摩擦轮解析
      {
        SHOOTEncoder[0].last_angle_raw_value = SHOOTEncoder[0].angle_raw_value;
        SHOOTEncoder[0].angle_raw_value = (hcan->pRxMsg->Data[0] << 8 | hcan->pRxMsg->Data[1]);
        SHOOTEncoder[0].rotate_speed = (hcan->pRxMsg->Data[2] << 8 | hcan->pRxMsg->Data[3]);
        if(SHOOTEncoder[0].last_angle_raw_value > 4500 && SHOOTEncoder[0].angle_raw_value < 3500)
        {
          feed_round++;
        }
		else if(SHOOTEncoder[0].last_angle_raw_value < 3000 && SHOOTEncoder[0].angle_raw_value > 5000)
		{
			feed_round--;
		}
				//电机的减速比，大约是36，所以编码器分辨率为 360度/36/8192;
				SHOOTEncoder[0].real_angle = SHOOTEncoder[0].angle_raw_value + feed_round * 8191;
				SHOOTEncoder[0].real_angle = SHOOTEncoder[0].real_angle * 10.0f / 8191.0f;
      }
			break;
      case FRIC1_RxID:
      case FRIC2_RxID:
      case FRIC3_RxID:
      {
        index = hcan->pRxMsg->StdId - FEED_RxID;
        SHOOTEncoder[index].last_angle_raw_value = SHOOTEncoder[index].angle_raw_value;
        SHOOTEncoder[index].angle_raw_value = (hcan->pRxMsg->Data[0] << 8 | hcan->pRxMsg->Data[1]);
        SHOOTEncoder[index].rotate_speed = (hcan->pRxMsg->Data[2] << 8 | hcan->pRxMsg->Data[3]);
        SHOOTEncoder[index].angular_velocity = 2 * PI * SHOOTEncoder[index].rotate_speed / 60;
      }
			break;
    }
  }
  else
  {
    switch(hcan->pRxMsg->StdId)
    {	
		case CHASSIS1_RxID:       //底盘数据解析
		case CHASSIS2_RxID:
		case CHASSIS3_RxID:
		case CHASSIS4_RxID:
		{
			index = hcan->pRxMsg->StdId - CHASSIS1_RxID;
			CMEncoder[index].velocity = (hcan->pRxMsg->Data[2] << 8 | hcan->pRxMsg->Data[3]);
			wheel_can_rc[index] = HAL_GetTick();
			wheel_can_rcTimes[index]++;
		}
		break;
		case POWER_ID:
		{
			power_chaisispower = hcan->pRxMsg->Data[0];
			isErro             = hcan->pRxMsg->Data[1];
			isChange           = hcan->pRxMsg->Data[2];
			RemainPower        = hcan->pRxMsg->Data[3];
			if(isChange == 1)
			{
				isStop_tick = HAL_GetTick();
			}
			power_tick = HAL_GetTick(); 
		}
    }
  }

  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
  HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);
}


/**
* @brief  向can总线发送数据，抄官方的
* @param  can的结构体
* @param  发送的内容
* @param  发送ID
* @param  数据长度
* @retval none
* @note   
**/
void CAN_Send_Msg(CAN_HandleTypeDef* hcan, uint8_t* msg, uint32_t id, uint8_t len)
{
  uint8_t index = 0;
  hcan->pTxMsg->StdId = id;
  hcan->pTxMsg->IDE = CAN_ID_STD;
  hcan->pTxMsg->RTR = CAN_RTR_DATA;
  hcan->pTxMsg->DLC = len;

  for(index = 0; index < len; index++)
  {
    hcan->pTxMsg->Data[index] = msg[index];
  }
  HAL_CAN_Transmit_IT(hcan);
}

/**
* @brief  设定云台电流
* @param  can的结构体
* @param  yaw轴电流
* @param  pitch电流
* @retval none
* @note   发送ID为0x1ff
**/
void Set_Gimbal_Current(CAN_HandleTypeDef* hcan, int16_t yaw_current, int16_t gimbal_current)
{
  uint8_t TxMessage[8];
  TxMessage[0] = (unsigned char)(yaw_current >> 8);
  TxMessage[1] = (unsigned char)yaw_current;
  TxMessage[2] = (unsigned char)(gimbal_current >> 8);
  TxMessage[3] = (unsigned char)gimbal_current;
  TxMessage[4] = 0;
  TxMessage[5] = 0;
  TxMessage[6] = 0;
  TxMessage[7] = 0;  
  CAN_Send_Msg(hcan, TxMessage, 0x1ff, 8);
}

/**
* @brief  设定底盘电流
* @param  can的结构体
* @param  左前轮电流
* @param  右前轮电流
* @param  右后轮电流
* @param  左后轮电流
* @retval none
* @note   发送ID为0x200
**/
void Set_Chassis_Current(CAN_HandleTypeDef* hcan, int16_t fl_current, int16_t fr_current, int16_t br_current, int16_t bl_current)
{
  uint8_t TxMessage[8];
  TxMessage[0] = (unsigned char)(fl_current >> 8);
  TxMessage[1] = (unsigned char)fl_current;
  TxMessage[2] = (unsigned char)(fr_current >> 8);
  TxMessage[3] = (unsigned char)fr_current;
  TxMessage[4] = (unsigned char)(br_current >> 8);
  TxMessage[5] = (unsigned char)br_current;
  TxMessage[6] = (unsigned char)(bl_current >> 8);
  TxMessage[7] = (unsigned char)bl_current;
  CAN_Send_Msg(hcan, TxMessage, 0x200, 8);
}

/**
* @brief  设定电机控制电流
* @param  hcan can的结构体
* @param  feed_current 拨弹电机电流
* @param  fric_current 摩擦轮电机电流
* @retval none
* @note   发送ID为0x200，步兵把最后两个Message设为0
**/
void Set_Shoot_Current(CAN_HandleTypeDef* hcan, int16_t feed_current, int16_t* fric_current)
{
  uint8_t TxMessage[8];
  TxMessage[0] = (unsigned char)(feed_current >> 8);
  TxMessage[1] = (unsigned char)feed_current;
  TxMessage[2] = (unsigned char)(fric_current[0] >> 8);
  TxMessage[3] = (unsigned char)fric_current[0];
  TxMessage[4] = (unsigned char)(fric_current[1] >> 8);
  TxMessage[5] = (unsigned char)fric_current[1];
  TxMessage[6] = (unsigned char)(fric_current[2] >> 8);
  TxMessage[7] = (unsigned char)fric_current[2];
  CAN_Send_Msg(hcan, TxMessage, 0x200, 8);
}
