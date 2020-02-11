/**************************************************
* @file      mode_update.c
* @author    wwy
* @version   version 2.0
* @date      2019.4.17
* @brief     简化了遥控器和键鼠操作，增加了last_mode
***************************************************
* @attention
***************************************************
*/

/* Include ---------------------------------------*/
#include "mode_update.h"
extern int32_t gimbal_round, gimbal_round_now;
//NUC自瞄相关内容
//包含：按键和读取裁判系统两种方式设置
extern uint8_t my_robot_id;                               //机器人ID设置
uint8_t Send_To_NUC[3] = {0x3f, 0x4f, 0x00};              //发给NUC的数据，初始为不工作
uint8_t nuc_send_key_flag = 0;                                //由键盘和NUC接收数据控制是否发送
uint8_t nuc_send_auto_flag = 0;

//电容
extern uint8_t isAccelerate;
extern uint8_t isBoom;
extern int8_t twist_direction;
extern uint8_t isHighSpeed;
extern float shoot_ratio;
/* Global Variable -------------------------------*/
Mode_t Mode;                         //模式
Mode_t Last_Mode;                    //上次模式
R_or_K_Control_t RK_Control;         //模式控制来源     
R_or_K_Control_t Last_RK_Control;    //上次模式控制来源
extern float current_yaw;
extern float imu_yaw;

/**
* @brief  开机模式由遥控器决定
* @param  none
* @retval none
* @note
**/
void RK_Control_Init(void)
{
    RK_Control = Remote_Control;               
    Last_RK_Control = Remote_Control;
	last_rc = rc;
	Last_Mode = Mode;
}

/**
* @brief  模式由最后设定的响应，当键盘q e shift ctrl 右键 左键有动作时模式响应键盘
          当遥控器设定改变时响应遥控器
* @param  none
* @retval none
* @note   该函数为源，调用Keyboard_Mode_Refresh，Remote_Mode_Refresh函数
**/
void Control_Change(void)
{
	if(Last_Mode.Gimbal == Gimbal_Follow_Mode &&Mode.Gimbal == Gimbal_Lock_Mode)
		gimbal_round_now = gimbal_round;
	
	Last_Mode = Mode;

    if(Last_RK_Control == Remote_Control)
    {
        if(rc.kb.bit.Q || rc.kb.bit.E || rc.kb.bit.CTRL || rc.kb.bit.SHIFT ||rc.kb.bit.Z||rc.kb.bit.R|| rc.mouse.r || rc.mouse.l||rc.kb.bit.Z||rc.kb.bit.X||rc.kb.bit.C||rc.kb.bit.V ) 
        {
            RK_Control = Keyboard_Control;
        }
        else
        {
            RK_Control = Remote_Control;
        }
        
    }
    else if(Last_RK_Control == Keyboard_Control)
    {
        if((last_rc.sw1 != rc.sw1) || (last_rc.sw2 != rc.sw2))
        {
            RK_Control = Remote_Control;
        }
        else
        {
            RK_Control = Keyboard_Control;
        }
    }

	//自瞄ID处理
	NUC_Send_ID_Process();
	switch(RK_Control)
    {
        case Keyboard_Control: Keyboard_Mode_Refresh();break;
        case Remote_Control  : Remote_Mode_Refresh();break;
    }
	last_rc = rc;
    Last_RK_Control = RK_Control;  
}


/**
* @brief  由遥控器的设定决定模式
* @param  none
* @retval none
* @note
**/
void Remote_Mode_Refresh(void)
{
    switch(rc.sw1)
    {
        case RC_UP:                   
            switch(rc.sw2)
            {
                case RC_UP:                            	//左上右上，底盘自检，关闭自瞄           
                Mode.Gimbal  = Gimbal_Lock_Mode;
                Mode.Shoot   = Shoot_Normal_Mode;
                Mode.Chassis = Chassis_Check_Mode;
								//Mode.Chassis = Chassis_Follow_Mode;
								Mode.Shoot_Signal = 1;
								if(last_rc.sw2 != RC_UP)
								{
									nuc_send_key_flag = 1;
								}
								NUC_Send_Mode_Process(Close);
                break;
				
                case RC_MI:                            	//左上右中，底盘云台锁死，关闭自瞄;
                Mode.Gimbal  = Gimbal_Lock_Mode;
                Mode.Shoot   = Shoot_Normal_Mode;
                Mode.Chassis = Chassis_Lock_Mode;
				        Mode.Shoot_Signal = 0;
								if(last_rc.sw2 != RC_MI)
								{
									nuc_send_key_flag = 1;
								}
								NUC_Send_Mode_Process(Close);
                break;
				
			    case RC_DN:                           	//左上右下，小陀螺模式+云台跟随模式
				        Mode.Gimbal  = Gimbal_Follow_Mode;
				        Mode.Shoot   = Shoot_Normal_Mode;
				        Mode.Chassis = Chassis_Twist_Mode;
				        //Mode.Chassis = Chassis_Follow_Mode;
								Mode.Shoot_Signal = 0;
                break;
	          }	                             
	      break;
        case RC_MI:
            switch(rc.sw2)
            {
                case RC_UP:                            //左中右上，跟随模式，自瞄红色
                Mode.Shoot   = Shoot_Auto_Mode;  
                Mode.Gimbal  = Gimbal_Auto_Mode;
                Mode.Chassis = Chassis_Follow_Mode;
								//Mode.Chassis = Chassis_Twist_Mode;
				        Mode.Shoot_Signal = 1;
								if(last_rc.sw2 != RC_UP)
								{
									nuc_send_key_flag = 1;
								}
								NUC_Send_Mode_Process(Red);    //红色自瞄
                break;
				
                case RC_MI:                            //左中右中，跟随模式，自瞄蓝色
                Mode.Shoot   = Shoot_Auto_Mode;
                Mode.Gimbal  = Gimbal_Auto_Mode;
                Mode.Chassis = Chassis_Follow_Mode;
								//Mode.Chassis = Chassis_Twist_Mode;
				        Mode.Shoot_Signal = 0;
								if(last_rc.sw2 != RC_MI)
								{
									nuc_send_key_flag = 1;
								}
								NUC_Send_Mode_Process(Blue);   //蓝色自瞄
                break;
				
				case RC_DN:                            //左中右下，跟随模式，摆动过大有晃动
				Mode.Shoot   = Shoot_Normal_Mode;
        Mode.Gimbal  = Gimbal_Follow_Mode;
        Mode.Chassis = Chassis_Follow_Mode;
				Mode.Shoot_Signal = 0;
				break;
            }
			break;
		case RC_DN:
			switch(rc.sw2)
			{
				case RC_UP:                                  //左下右上，开始补弹
				Mode.Shoot  = Shoot_Normal_Mode;
				Mode.Gimbal = Gimbal_Lock_Mode;
				Mode.Chassis= Chassis_Hand_Start_Mode;
				Mode.Shoot_Signal = 0;
				break;
				
				case RC_MI:                                   //左下右中，退出补弹
				Mode.Shoot = Shoot_Normal_Mode;
				Mode.Gimbal = Gimbal_Lock_Mode;
				Mode.Chassis = Chassis_Inspecion_Mode;
				Mode.Shoot_Signal = 0;
				default: break;
				
				case RC_DN:                                    //左下，右边往下掰一下，射击一下
				Mode.Shoot  = Shoot_Normal_Mode;
				Mode.Gimbal = Gimbal_Lock_Mode;
				Mode.Chassis= Chassis_Inspecion_Mode;
				if(last_rc.sw2 != RC_DN)
					Mode.Shoot_Signal = 1;
				else
					Mode.Shoot_Signal = 0;
				break;
			}
			break;					      
    }
}

/**
* @brief  由键盘的设定决定模式
* @param  none
* @retval none
* @note
**/
void Keyboard_Mode_Refresh(void)
{
	Mode.Shoot_Signal = rc.mouse.l;                //左键按，射击
	if(rc.kb.bit.CTRL)
	{
		if(rc.kb.bit.SHIFT)
		{
			if(rc.kb.bit.A)
			{
				if(last_rc.kb.bit.A!=1)
				{
					  nuc_send_key_flag = 1;
					  
				}
				NUC_Send_Mode_Process(Close);
			}
			else if(rc.kb.bit.Q)                            //ctrl+shift+Q，自瞄设定红装甲
		  {
	      if(last_rc.kb.bit.Q!=1)
		    {
					  nuc_send_key_flag = 1;
				}	
				NUC_Send_Mode_Process(Red);
	  	}
	  	else if(rc.kb.bit.E)                       //ctrl+shift+E, 自瞄设定蓝装甲
		  {
			  if(last_rc.kb.bit.E!=1)
				{
					  nuc_send_key_flag = 1;
				}
				NUC_Send_Mode_Process(Blue);
      }	
		}
		else if((rc.kb.bit.Q)&&(rc.kb.bit.SHIFT!=1))                           //ctrl+Q，自瞄设定红装甲
		{
		  	Mode.Shoot   = Shoot_Normal_Mode;
	    	Mode.Chassis = Chassis_Follow_Mode;
		    Mode.Gimbal  = Gimbal_Follow_Mode;
	        isHighSpeed = 1;
		}	
		else if((rc.kb.bit.A)&&(rc.kb.bit.SHIFT!=1))       
		{
	      if(last_rc.kb.bit.A!=1)
		    {
					  shoot_ratio = 3.0;
				}	
		}
	    else if(rc.kb.bit.D)
	    {
		    if(last_rc.kb.bit.D!=1)
				{
					  shoot_ratio = 0.0;
				}
	    }
		else if(rc.kb.bit.W)                       //ctrl+W，锁死模式
		{
			Mode.Shoot  = Shoot_Normal_Mode;
			Mode.Gimbal = Gimbal_Lock_Mode;
			Mode.Chassis= Chassis_Inspecion_Mode;
		}
		else if(rc.kb.bit.S)
		{
			isBoom = 1;
		}
		
	}
	else if(rc.kb.bit.E)                          //按E，随动模式
	{
		Mode.Shoot   = Shoot_Normal_Mode;
		Mode.Chassis = Chassis_Follow_Mode;
		Mode.Gimbal  = Gimbal_Follow_Mode;
		isHighSpeed = 0;
	}
	else if(rc.kb.bit.Q)    //按Q，摆腰模式
	{
		Mode.Shoot   = Shoot_Normal_Mode;
		Mode.Chassis = Chassis_Twist_Mode;
		Mode.Gimbal  = Gimbal_Follow_Mode;
		if(last_rc.kb.bit.Q!=1)
		{
		twist_direction = -1*twist_direction;
		}
	}
				  


	
	
	if(rc.mouse.r == 1)                                 //按住右键，自瞄模式，底盘的模式为随动或者摆腰
	{
		Mode.Shoot   = Shoot_Auto_Mode;
		Mode.Gimbal  = Gimbal_Auto_Mode;
		if(Last_Mode.Chassis == Chassis_Lock_Mode)
			Mode.Chassis = Chassis_Follow_Mode;
		else
			Mode.Chassis = Last_Mode.Chassis;
		if (Last_Mode.Gimbal != Mode.Gimbal)
		{
			current_yaw = imu_yaw;
		}
	
	}

}

/**********************************************************/
/*函数内容：通过机器人ID给NUC赋初值                       */
/*说明：函数逻辑为在模式控制里面先根据ID设置发送内容，但是*/
/*      如果有按键控制则会覆盖原来的值                    */
/**********************************************************/
void NUC_Send_ID_Process()
{
	//自己红色，对面蓝色
	if((my_robot_id<=10)&&(my_robot_id)>0)
	{
		NUC_Send_Mode_Process(Blue);
	}
	//自己是蓝色，对面红色
	else if((my_robot_id>10)&&(my_robot_id<=20))
	{
		NUC_Send_Mode_Process(Red);
	}
}
/**********************************************************/
/*函数内容：负责把状态信息发送给NUC                       */
/**********************************************************/
void NUC_Send_Mode_Process(NUC_Send_Mode Mode)
{
	switch(Mode)
	{
		case Red        :Send_To_NUC[2] = 0x01;break;
		case Blue       :Send_To_NUC[2] = 0x02;break;
		case CHECK_OPEN :Send_To_NUC[2] = 0x03;break;
		case CHECK_CLOSE:Send_To_NUC[2] = 0x04;break;
		case Close :     Send_To_NUC[2] = 0x7f;break;
		//写入默认值
		//default:    Send_To_NUC[2] = 
	}
	
	if((nuc_send_key_flag == 1)||(nuc_send_auto_flag == 1))
	{
		Send_To_NUC[0] = 0x3f;
		Send_To_NUC[1] = 0x4f;
		HAL_UART_Transmit(&huart3, Send_To_NUC, 3, 1000);
		nuc_send_key_flag = 0;
		nuc_send_auto_flag = 0;
	}
	
	
}
void NUC_Send_Type_Process(NUC_Send_Type Mode)
{
	if((nuc_send_key_flag == 1))
	{
		Send_To_NUC[0] = 0x3f;
		Send_To_NUC[1] = 0x4f;
		HAL_UART_Transmit(&huart3, Send_To_NUC, 3, 1000);
		nuc_send_key_flag = 0;
	}
}

