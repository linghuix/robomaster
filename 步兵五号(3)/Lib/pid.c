/**************************************************
* @file      pid.c
* @author    not wwy
* @version   
* @date      
* @brief     
***************************************************
* @attention
for hero robot
***************************************************
*/

/* Include ---------------------------------------*/
#include "pid.h"

/* Global Variable -------------------------------*/
PID_Regulator_Double_Loop_t PitchPID;
PID_Regulator_Double_Loop_t PitchAutoPID;
PID_Regulator_Double_Loop_t YawPID;
PID_Regulator_Double_Loop_t YawAutoPID;
PID_Regulator_Double_Loop_t ShootFeedPID;




PID_Regulator_t ShootFric1PID;     //1和2是一对摩擦轮，3是辅助摩擦轮
PID_Regulator_t ShootFric2PID;
PID_Regulator_t ShootFric3PID;
PID_Regulator_t ChassisPID[4];

PID_Regulator_t ChassisVWPID;
PID_Regulator_t Chassis_Twist_PID;
PID_Regulator_t Chassis_ADCx_PID;
PID_Regulator_t Chassis_ADCy_PID;




void PID_Calc(PID_Regulator_t *pid)
{
	pid->err[0] = pid->err[1];
	pid->err[1] = pid->ref - pid->fdb;
	pid->inte+=pid->err[1];
		
	
	pid->componentKp  = pid->kp * pid->err[1];
	pid->componentKi  = pid->ki * pid->inte;
	pid->componentKd  = pid->kd * (pid->err[1] - pid->err[0]);
	
	if(pid->componentKp > pid->componentKpMax)
		pid->componentKp = pid->componentKpMax;
	else if (pid->componentKp < -pid->componentKpMax)
		pid->componentKp = -pid->componentKpMax;
		
	if(pid->componentKi > pid->componentKiMax)
		pid->componentKi = pid->componentKiMax;
	else if (pid->componentKi < -pid->componentKiMax)
		pid->componentKi = -pid->componentKiMax;
	
	pid->output = pid->componentKp + pid->componentKi+ pid->componentKd;
	
	if(pid->output > pid->outputMax)
		pid->output = pid->outputMax;
	else if (pid->output < -pid->outputMax)
		pid->output = -pid->outputMax;	
}

/* 电机编码器过零导致传感器位置信号突变,增加检测信号 */
void Yaw_PID_Calc(PID_Regulator_t *pid)
{
	float a;
	a = pid->ref - pid->fdb;
	if (a<0)
	{
		if (a < -180)
			a = 360 + a;
	}
	else
	{
		if (a>180)
			a = a - 360;
	}
	pid->err[0] = pid->err[1];
	pid->err[1] = a;
	pid->inte += pid->err[1];


	pid->componentKp = pid->kp * pid->err[1];
	pid->componentKi = pid->ki * pid->inte;
	pid->componentKd = pid->kd * (pid->err[1] - pid->err[0]);


	if (pid->componentKi > pid->componentKiMax)
		pid->componentKi = pid->componentKiMax;
	else if (pid->componentKi < -pid->componentKiMax)
		pid->componentKi = -pid->componentKiMax;

	pid->output = pid->componentKp + pid->componentKi + pid->componentKd;

	if (pid->output > pid->outputMax)
		pid->output = pid->outputMax;
	else if (pid->output < -pid->outputMax)
		pid->output = -pid->outputMax;
}

void PID_Init(PID_Regulator_t *pid,float kp,float ki,float kd,float componentKpMax,float componentKiMax,float componentKdMax,float outputMax)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->inte = 0;
	pid->componentKpMax = componentKpMax;
	pid->componentKiMax = componentKiMax;
	pid->componentKdMax = componentKdMax;
	pid->outputMax = outputMax;
}


void PID_ALL_Init(void)
{
	PID_Init(&PitchPID.Position,PITCH_POSITION_KP_DEFAULTS,PITCH_POSITION_KI_DEFAULTS,PITCH_POSITION_KD_DEFAULTS,\
					PITCH_POSITION_KPMAX,PITCH_POSITION_KIMAX,PITCH_POSITION_KDMAX,PITCH_POSITION_OUTPUTMAX);
	
	PID_Init(&PitchPID.Speed,PITCH_SPEED_KP_DEFAULTS,PITCH_SPEED_KI_DEFAULTS,PITCH_SPEED_KD_DEFAULTS,\
					PITCH_SPEED_KPMAX,PITCH_SPEED_KIMAX,PITCH_SPEED_KDMAX,PITCH_SPEED_OUTPUTMAX);
	
	PID_Init(&YawPID.Position,YAW_POSITION_KP_DEFAULTS,YAW_POSITION_KI_DEFAULTS,YAW_POSITION_KD_DEFAULTS,\
					YAW_POSITION_KPMAX,YAW_POSITION_KIMAX,YAW_POSITION_KDMAX,YAW_POSITION_OUTPUTMAX);
	
	PID_Init(&YawPID.Speed,200,YAW_SPEED_KI_DEFAULTS,YAW_SPEED_KD_DEFAULTS,\
					YAW_SPEED_KPMAX,YAW_SPEED_KIMAX,YAW_SPEED_KDMAX,YAW_SPEED_OUTPUTMAX);
	
	PID_Init(&PitchAutoPID.Position,PITCHAUTO_POSITION_KP_DEFAULTS,PITCHAUTO_POSITION_KI_DEFAULTS,PITCHAUTO_POSITION_KD_DEFAULTS,\
					PITCHAUTO_POSITION_KPMAX,PITCHAUTO_POSITION_KIMAX,PITCHAUTO_POSITION_KDMAX,PITCHAUTO_POSITION_OUTPUTMAX);
	
	PID_Init(&PitchAutoPID.Speed,PITCHAUTO_SPEED_KP_DEFAULTS,PITCHAUTO_SPEED_KI_DEFAULTS,PITCHAUTO_SPEED_KD_DEFAULTS,\
					PITCHAUTO_SPEED_KPMAX,PITCHAUTO_SPEED_KIMAX,PITCHAUTO_SPEED_KDMAX,PITCHAUTO_SPEED_OUTPUTMAX);
	
	PID_Init(&YawAutoPID.Position,YAWAUTO_POSITION_KP_DEFAULTS,YAWAUTO_POSITION_KI_DEFAULTS,YAWAUTO_POSITION_KD_DEFAULTS,\
					YAWAUTO_POSITION_KPMAX,YAWAUTO_POSITION_KIMAX,YAWAUTO_POSITION_KDMAX,YAWAUTO_POSITION_OUTPUTMAX);
	
	PID_Init(&YawAutoPID.Speed,YAWAUTO_SPEED_KP_DEFAULTS,YAWAUTO_SPEED_KI_DEFAULTS,YAWAUTO_SPEED_KD_DEFAULTS,\
					YAWAUTO_SPEED_KPMAX,YAWAUTO_SPEED_KIMAX,YAWAUTO_SPEED_KDMAX,YAWAUTO_SPEED_OUTPUTMAX);
	
	
	//Shoot PID
	PID_Init(&ShootFeedPID.Position,FEED_POSITION_KP_DEFAULTS,FEED_POSITION_KI_DEFAULTS,FEED_POSITION_KD_DEFAULTS,\
					FEED_POSITION_KPMAX,FEED_POSITION_KIMAX,FEED_POSITION_KDMAX,FEED_POSITION_OUTPUTMAX);

	PID_Init(&ShootFeedPID.Speed,FEED_SPEED_KP_DEFAULTS,FEED_SPEED_KI_DEFAULTS,FEED_SPEED_KD_DEFAULTS,\
					FEED_SPEED_KPMAX,FEED_SPEED_KIMAX,FEED_SPEED_KDMAX,FEED_SPEED_OUTPUTMAX);
					
	PID_Init(&ShootFric1PID,FRIC1_SPEED_KP_DEFAULTS,FRIC1_SPEED_KI_DEFAULTS,FRIC1_SPEED_KD_DEFAULTS,\
					FRIC1_SPEED_KPMAX,FRIC1_SPEED_KIMAX,FRIC1_SPEED_KDMAX,FRIC1_SPEED_OUTPUTMAX);
	
	PID_Init(&ShootFric2PID,FRIC2_SPEED_KP_DEFAULTS,FRIC2_SPEED_KI_DEFAULTS,FRIC2_SPEED_KD_DEFAULTS,\
					FRIC2_SPEED_KPMAX,FRIC2_SPEED_KIMAX,FRIC2_SPEED_KDMAX,FRIC2_SPEED_OUTPUTMAX);
	
	PID_Init(&ShootFric3PID,FRIC3_SPEED_KP_DEFAULTS,FRIC3_SPEED_KI_DEFAULTS,FRIC3_SPEED_KD_DEFAULTS,\
					FRIC3_SPEED_KPMAX,FRIC3_SPEED_KIMAX,FRIC3_SPEED_KDMAX,FRIC3_SPEED_OUTPUTMAX);

	for (int i = 0; i < 4; i++)
	{
		PID_Init(&ChassisPID[i], CHASSIS_WHEEL_KP_DEFAULTS, CHASSIS_WHEEL_KI_DEFAULTS, CHASSIS_WHEEL_KD_DEFAULTS, \
			CHASSIS_WHEEL_SPEED_KPMAX, CHASSIS_WHEEL_SPEED_KIMAX, CHASSIS_WHEEL_SPEED_KDMAX, CHASSIS_WHEEL_SPEED_OUTPUTMAX);
	}
	PID_Init(&Chassis_Twist_PID,CHASSIS_TWIST_KP_DEFAULTS,CHASSIS_TWIST_KI_DEFAULTS,CHASSIS_TWIST_KD_DEFAULTS,\
	        CHASSIS_TWIST_KPMAX,CHASSIS_TWIST_KIMAX,CHASSIS_TWIST_KIMAX,CHASSIS_TWIST_OUTPUTMAX);
	PID_Init(&Chassis_ADCx_PID,CHASSIS_ADCX_KP_DEFAULTS,CHASSIS_ADCX_KI_DEFAULTS,CHASSIS_ADCX_KD_DEFAULTS,\
	    CHASSIS_ADCX_KPMAX,CHASSIS_ADCX_KIMAX,CHASSIS_ADCX_KDMAX,CHASSIS_ADCX_OUTPUTMAX);
	PID_Init(&Chassis_ADCy_PID,CHASSIS_ADCY_KP_DEFAULTS,CHASSIS_ADCY_KI_DEFAULTS,CHASSIS_ADCY_KD_DEFAULTS,\
	    CHASSIS_ADCY_KPMAX,CHASSIS_ADCY_KIMAX,CHASSIS_ADCY_KDMAX,CHASSIS_ADCY_OUTPUTMAX);
	PID_Init(&ChassisVWPID,CHASSIS_VW_KP_DEFAULTS,CHASSIS_VW_KI_DEFAULTS,CHASSIS_VW_KD_DEFAULTS,\
	    CHASSIS_VW_KPMAX,CHASSIS_VW_KIMAX,CHASSIS_VW_KDMAX,CHASSIS_VW_OUTPUTMAX);
}


