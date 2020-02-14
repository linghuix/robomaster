/**************************************************
* @file      control_task.c
* @author    wwy
* @version   version 1.1
* @date      2019.2.17
* @brief     2.17 在步兵上调了pitch和摩擦轮
***************************************************
* @attention

***************************************************
*/

#define JSCOPE   	//Jscope调试


/* Include ---------------------------------------*/
#include "control_task.h"
#include "imu_task.h"


/* Global Variable -------------------------------*/
int CUR_YAW;
extern float current_yaw;
float Pitch_error;

extern uint8_t RemainPower;
extern float twist_speed;
extern float shoot_ratio;
uint32_t time_1ms;            //1ms计数
uint32_t LoopTask_End, LoopTask_Start, LoopTask_Time;

void judge_periodSend_controller(void){
	
	if(time_1ms%100==0)
	{
		judge_send_custom_data(RemainPower,twist_speed,shoot_ratio,0);
	}
}

void startup_singing_controller(void){
	if(time_1ms % 80==0 && Startup_Success_music_index < Startup_Success_music_len)
		{
			Sing_Startup_music(Startup_Success_music_index);
			Startup_Success_music_index++;
		}
}

#ifdef JSCOPE      //Jscope调试
int pit_sp_now, pit_sp_tar, pit_pos_now, pit_pos_tar, fric1_sp_now, fric1_sp_tar, fric2_sp_now, fric2_sp_tar;
int feed_pos_now, feed_pos_tar, feed_sp_now,feed_sp_tar;
int yaw_sp_now ,yaw_sp_tar ,yaw_pos_now,yaw_pos_tar;

void jscope(void){
	pit_sp_now  = PitchPID.Speed.fdb * 1000;
	pit_sp_tar  = PitchPID.Speed.ref * 1000;
	pit_pos_now = PitchPID.Position.fdb * 1000;
	pit_pos_tar = PitchPID.Position.ref * 1000; 

	yaw_sp_now  = YawAutoPID.Speed.fdb * 1000;
	yaw_sp_tar  = YawAutoPID.Speed.ref*1000;
	yaw_pos_now = YawAutoPID.Position.fdb*1000;
	yaw_pos_tar =	YawAutoPID.Position.ref*1000;

	fric1_sp_now = ShootFric1PID.fdb * 1000;
	fric1_sp_tar = ShootFric1PID.ref * 1000;
	fric2_sp_now = ShootFric2PID.fdb * 1000;
	fric2_sp_tar = ShootFric2PID.ref * 1000;
	feed_pos_now = ShootFeedPID.Position.fdb;
	feed_pos_tar = ShootFeedPID.Position.ref;
	feed_sp_now  = ShootFeedPID.Speed.fdb;
	feed_sp_tar  = ShootFeedPID.Speed.ref;
	
}
#endif


/**
* @brief  由定时器6中断进入的主控制函数，云台、底盘等控制皆从此进入
* @param  none
* @retval none
* @note
**/

extern Mode_t Last_Mode;					 //上次模式
extern R_or_K_Control_t Last_RK_Control;	 //上次模式控制来源
extern R_or_K_Control_t RK_Control;
extern RC_Type  last_rc;
void Control_Task(void)
{
	
	time_1ms++;
	LoopTask_Start = HAL_GetTick();
	
	if(Startup_Success_music_index == Startup_Success_music_len)
	{
		Control_Change();
		imu_task();

		if(imu_init_tick >= IMU_INIT_TIME)
		{
			gimbal_task();
			shoot_task();
			chassis_task();
		}

		//保存本timestep的状态
		Last_RK_Control = RK_Control;  
		last_rc = rc;
		Last_Mode = Mode;
	}

	judge_periodSend_controller();
	
	startup_singing_controller();
	
	#ifdef JSCOPE      //Jscope调试
	jscope();
	#endif
	
	imu_yaw_buf_pos=(imu_yaw_buf_pos+1);
	imu_yaw_buf[imu_yaw_buf_pos] = imu_yaw;
	
	LoopTask_End = HAL_GetTick();
	LoopTask_Time = LoopTask_End-LoopTask_Start;
	
}
