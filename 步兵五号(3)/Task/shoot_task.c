/**************************************************
* @file      shoot_task.c
* @author    wwy
* @version   version 1.0
* @date      2019.2.10
* @brief     射击任务,本工程还未与裁判系统适配，枪口
             热量变量用shootHeat表示,机器人等级用
			 robotlevel表示
***************************************************
* @attention
For hero robot. Since hero robot has only two shoot
mode, so it's quite simpler than infantry robot.
Infantry need additional codes.
***************************************************
*/

/* Include ---------------------------------------*/
#include "shoot_task.h"

/* Global Variable -------------------------------*/
Trigger_Mode_t Trigger_Mode;                            //拨弹模式
const int16_t heat_buff[4] = {0, 240, 360, 480};       //热量上限
const int8_t heat_cd[4] = {0, 20, 40, 60};             //每秒冷却值
int16_t fric_current[3];                                //发个三个摩擦轮的电流值
int16_t feed_current;                                   //发给拨弹电机的电流值
int8_t shoot_allowed_num, shooted_num;                 //当前热量下允许射击的总数和热量未更新状态下已发射的弹丸数
uint32_t tick_last,tick_sub;                            //当前射击和上一次射击的时间差，若大于100则默认热量数据已更新
uint8_t Shoot_Signal_Old;                               //上一次的射击请求信号
uint8_t shoot_one;                                      //打出一发
uint32_t continuous_shoot_tick;                         //连发计时

extern short temp3;     //从nuc那里获得的深度


//发射一个子弹的热量
const int SHOOT_SPEED25 = 9;
const int SHOOT_SPEED20 = 12; 
const int SHOOT_SPEED15 = 18;

const int BULL_SPEED15 = 3500;
const int BULL_SPEED20 = 3500;
const int BULL_SPEED25 = 5500;

/**
* @brief  总射击处理函数，根据模式进行散转
* @param  none
* @retval none
* @note
**/
void shoot_task(void)
{
	if(Mode.Shoot_Signal == 0)                          //没有射击请求信号
	{
		shoot_one = 0;
		continuous_shoot_tick = 0;
	}

	shoot_signal_process();                             //有射击请求信号，处理射击请求信号

    switch(Mode.Shoot)
    {
    	case Shoot_Normal_Mode: _shoot_normal_handle();break;
    	case Shoot_Auto_Mode:   _shoot_auto_handle();break;
    	default:                _shoot_normal_handle();break;
    }

	Shoot_Signal_Old = Mode.Shoot_Signal;
    fric_wheel_ctrl();           
	Set_Shoot_Current(&hcan1, feed_current, fric_current);

}

/**
* @brief  处理设计信号，判断是单发还是连发
* @param  none
* @retval none
* @note 发送shoot_one信号量，连发间隔100ms，单发不设间隔看手速
**/
void shoot_signal_process(void)
{
	if(Mode.Shoot_Signal == 0)
		return;
	
	if(Mode.Shoot == Shoot_Normal_Mode)               //普通射击模式下判断是否是连发
	{
		if(Shoot_Signal_Old == 0)        
		{
			Trigger_Mode = Single_Shoot;              //发射模式单发
		}
		else
		{
			Trigger_Mode = Continuous_Shoot;          //发射模式连发
		}
		switch(Trigger_Mode)
		{
			case Single_Shoot:
				shoot_one = 1;
				break;
			case Continuous_Shoot:
				continuous_shoot_tick++;
				if(continuous_shoot_tick >= 300)        //按住鼠标300ms触发连发
				{
					if(continuous_shoot_tick % 100 == 0)//连发射频10Hz
					{
						shoot_one = 1;
					}
					else
					{
						shoot_one = 0;
					}	
				}
				else
				{
					shoot_one = 0;
				}
				break;
		}
	}
	
}

/**
* @brief  普通射击处理函数
* @param  none
* @retval none
* @note 获取shoot_one信号量
**/
static void _shoot_normal_handle(void)
{
	//发射间隔要大于100时，更新已发射数量
	tick_sub = HAL_GetTick() - tick_last;
	if(tick_sub>=100)
		shooted_num = 0;
	
	if(shoot_one == 1)
	{
		//允许继续发射的子弹数
		shoot_allowed_num = (int8_t)((heat_buff[robotlevel]-shooterheat0)/SHOOT_SPEED20);
		shoot_allowed_num = 50;
		if(shooted_num>=shoot_allowed_num)
			ShootFeedPID.Position.ref = SHOOTEncoder[FEED].real_angle;
		else
		{
			tick_last = HAL_GetTick();//记录发射时间
			ShootFeedPID.Position.ref = SHOOTEncoder[FEED].real_angle + ONE_SHOOT_ANGLE;
			shooted_num++;
		}
	}
	ShootFeedPID.Position.fdb = SHOOTEncoder[FEED].real_angle;
	PID_Calc(&ShootFeedPID.Position);
	ShootFeedPID.Speed.fdb = SHOOTEncoder[FEED].rotate_speed;
	ShootFeedPID.Speed.ref = ShootFeedPID.Position.output;
	PID_Calc(&ShootFeedPID.Speed);
	feed_current = ShootFeedPID.Speed.output;
}

/**
* @brief  自瞄处理函数
* @param  none
* @retval none
* @note   自瞄子弹速度设置，云台角度校准设置
**/
int shoot_speed;                               //子弹射速
uint8_t schmidtFlag=0;
static void _shoot_auto_handle(void)
{
//	if (temp3 < 2000)
//	{
//		shoot_speed = SHOOT_SPEED15;
//	}
//	else
//  {
//	  shoot_speed = SHOOT_SPEED25;
//  }

	if(schmidtFlag==0 && temp3<1200)
		schmidtFlag=1;
	else if(schmidtFlag==1 && temp3>1500)
		schmidtFlag=0;
	
	if(schmidtFlag==0)
		shoot_speed=SHOOT_SPEED25;
	else
		shoot_speed=SHOOT_SPEED15;
	
	//shoot_speed = 9;
	tick_sub = HAL_GetTick() - tick_last;
	if(tick_sub>=100)
		shooted_num = 0;

	if(isNUC == 1)
	{
		shoot_allowed_num = (int8_t)((heat_buff[robotlevel]-shooterheat0)/shoot_speed);//-1;
		if(shooted_num>=shoot_allowed_num)
			ShootFeedPID.Position.ref = SHOOTEncoder[FEED].real_angle;
		else
		{
			tick_last = HAL_GetTick();
			ShootFeedPID.Position.ref = SHOOTEncoder[FEED].real_angle + ONE_SHOOT_ANGLE;
			shooted_num++;
		}
	}
	ShootFeedPID.Position.fdb = SHOOTEncoder[FEED].real_angle;
	PID_Calc(&ShootFeedPID.Position);
	ShootFeedPID.Speed.fdb = SHOOTEncoder[FEED].rotate_speed;
	ShootFeedPID.Speed.ref = ShootFeedPID.Position.output;
	PID_Calc(&ShootFeedPID.Speed);
	feed_current = ShootFeedPID.Speed.output;
	isNUC = 0;
}

/**
* @brief  摩擦轮速度控制
* @param  none
* @retval none
* @note 根据 isHighSpeed 设置子弹速度
**/
int bull_speed;
uint8_t isHighSpeed = 0;
extern float PITCH_AUTO_OFFSET;
extern float YAW_AUTO_OFFSET;
static void fric_wheel_ctrl(void)
{
	if(Mode.Shoot == Shoot_Normal_Mode)
	{
		if(isHighSpeed==1)
		{
			bull_speed = 6500;
		}
		else
		{
			bull_speed = 5500;
		}
		PITCH_AUTO_OFFSET = 0.0;
		YAW_AUTO_OFFSET = 0.0;
	}
	else if(Mode.Shoot == Shoot_Auto_Mode)
	{
		  if (shoot_speed == SHOOT_SPEED15)
	    {
		      bull_speed = BULL_SPEED15;
		      PITCH_AUTO_OFFSET = 4.0;			//自瞄云台校准
			  YAW_AUTO_OFFSET = -0.5;
	    }
      else if (shoot_speed == SHOOT_SPEED25)
	    {
		      bull_speed = BULL_SPEED25;
		      PITCH_AUTO_OFFSET = 2.0;
			  YAW_AUTO_OFFSET = -0.4;
	    }
	    else
      {
			bull_speed = BULL_SPEED15;
			PITCH_AUTO_OFFSET =0.0;//2.0;//陀螺自瞄//自瞄哨兵0.0//自瞄地面 2.0;
			YAW_AUTO_OFFSET =-0.4;//1.0;//陀螺自瞄// -0.4;自瞄地面
      }
	}
	else
	{
		bull_speed = 5500;
		PITCH_AUTO_OFFSET = 0.0;
		YAW_AUTO_OFFSET = 0.0;
	}
	
	//bull_speed = 5500;
	//PITCH_AUTO_OFFSET = 2.0;
	ShootFric1PID.ref = -1*bull_speed;
	ShootFric2PID.ref = -1*ShootFric1PID.ref;
    ShootFric3PID.ref = 1*bull_speed;
	ShootFric1PID.fdb = SHOOTEncoder[FRIC1].rotate_speed;
	ShootFric2PID.fdb = SHOOTEncoder[FRIC2].rotate_speed;
	ShootFric3PID.fdb = SHOOTEncoder[FRIC3].rotate_speed;
	PID_Calc(&ShootFric1PID);
	PID_Calc(&ShootFric2PID);
	PID_Calc(&ShootFric3PID);
	fric_current[0] = (int16_t)ShootFric1PID.output;
	fric_current[1] = (int16_t)ShootFric2PID.output;
	fric_current[2] = (int16_t)ShootFric3PID.output;
}

/**
* @brief  摩擦轮、拨弹电机停转
* @param  none
* @retval none
* @note
**/
void shoot_stop() {
	uint8_t TxMessage[8];
	TxMessage[0] = 0;
	TxMessage[1] = 0;
	TxMessage[2] = 0;
	TxMessage[3] = 0;
	TxMessage[4] = 0;
	TxMessage[5] = 0;
	TxMessage[6] = 0;
	TxMessage[7] = 0;
	CAN_Send_Msg(&hcan1, TxMessage, 0x200 , 8);
}

