#include "gimbal_task.h"
#include "HW_nuc.h"
#define NORMAL_K_MOUSE        0.0030f
#define NORMAL_K_CONTROLLER   0.18f
#define SUPPLY_K_MOUSE        0.0015f
#define SUPPLY_K_CONTROLLER   0.12f

#define PITCH_AUTO_WINDOWS    10
#define YAW_AUTO_WINDOWS       7



float PITCH_AUTO_OFFSET;//shoot_task.c fric_wheel_ctrl();   nuc超时    gimbal_test_auto()
float YAW_AUTO_OFFSET;  //shoot_task.c fric_wheel_ctrl();   nuc超时    gimbal_task_auto
float current_pitch = PITCH_OFFSET_ANGLE;            //pitch轴全局绝对位置量
//float current_yaw = YAW_OFFSET_ANGLE;                //yaw轴全局绝对位置量
float current_yaw;
extern float v;
float last_v = 0;
float my_temp_v = 0;

extern int nuc_flag, nuc_update_flag;
extern short pre_temp1,pre_temp3;
extern short temp1,temp2,temp3;

int FLAG = 0;
int   IMUerro;
int V_DETECT;






float Auto_Angle_Yaw, Auto_Angle_Pitch;       //解析出yaw和pitch的变化量,Auto_Angle_Yaw向右为正，Auto_Angle_Pitch向下为正
float last_Auto_Angle_Yaw;
int erro_Auto_Angle_Yaw;

int angle_yaw;

static float imu_yaw_last = 0;


int YAW_SPEED_OUTPUT;
//**********************************************************************
//* 功能： 云台模式总控制函数                                          *
//* 参数： 无                                                          *
//* 返回值：无                                                         *
//**********************************************************************
void gimbal_task(void)
{
	switch (Mode.Gimbal)
	{
		case Gimbal_Lock_Mode:   gimbal_lock_mode(); break;
		case Gimbal_Follow_Mode: gimbal_follow_mode(); break;
		case Gimbal_Auto_Mode:   gimbal_auto_mode();break;
		default:                 gimbal_lock_mode(); break;
	}		
}

//**********************************************************************
//* 功能： 云台锁死模式 
//* 参数： 无
//* 返回值：无
//* 说明：云台只接受遥控器的pitch指令，反馈采用电机的编码器，yaw角度控制为定值，
//* 	  云台姿态与底盘完全无关。
//**********************************************************************
void gimbal_lock_mode(void)
{
	float pitch_target;
	float pitch_feedback;
	float yaw_target;
	float yaw_feedback;
	pitch_target = set_pitch_angle();  //PITCH_OFFSET_ANGLE;  //通过遥控器设置目标位置
	pitch_feedback = GMPitchEncoder.real_angle;     //通过码盘反馈当前位置
	//yaw_target = set_yaw_angle();                 //通过遥控器设置目标位置
	current_yaw = imu_yaw;
	yaw_target = 8191 * gimbal_round_now * 0.0439453125f; //固定在同一个位置
	yaw_feedback = GMYawEncoder.real_angle;	        //通过码盘反馈当前位置
	gimbal_action(&PitchPID, pitch_target, pitch_feedback, &YawPID, yaw_target, yaw_feedback);
}


//**********************************************************************
//* 功能： 云台跟随模式                                      		     	      *
//* 参数： 无                                                      		      *
//* 返回值：无                                                   		      *
//* 说明：云台pitch,yaw跟随控制指令运动，yaw的反馈为imu，pitch反馈为编码器。                         *
//**********************************************************************
void gimbal_follow_mode(void)
{
	float pitch_target;
	float pitch_feedback;
	float yaw_target;
	float yaw_feedback;
	pitch_target = set_pitch_angle();               //通过遥控器设置目标位置
//	pitch_feedback = get_pitch_encoder_angle();     //通过码盘反馈当前位置
	pitch_feedback = GMPitchEncoder.real_angle;
	yaw_target = set_yaw_angle(NORMAL_K_MOUSE,NORMAL_K_CONTROLLER);	//通过遥控器设置目标位置
	yaw_feedback = imu_yaw;                         //通过IMU四元数结算反馈当前位置
//	yaw_feedback = GMYawEncoder.real_angle;
	gimbal_action(&PitchPID, pitch_target, pitch_feedback, &YawPID, yaw_target, yaw_feedback);
}


//**********************************************************************
//* 功能： 云台自瞄模式                                                
//* 参数： 无                                                          
//* 返回值：无  
//* 说明：根据从nuc获取的temp1234，自动计算yaw和pitch。当预测的yaw，pitch与当前值相差很大时自动进入follow mode
//**********************************************************************
float NUCstop_pitch;
float NUCstop_yaw;
float v;
extern float IMU_yaw, pre_IMU_yaw;
extern float errotime;

uint8_t yaw_schmidtFlag = 0;

void gimbal_auto_mode()
{
	float pitch_target;
	float pitch_feedback;
	float yaw_target;
	float yaw_feedback;
	uint8_t follow_flag = 0;

	//获取nuc数据的时刻 和 目前时间 超过100时，退出auto模式
	if(HAL_GetTick() - nuc_tick > 100)
	{
		//current_yaw = imu_yaw;
		Auto_Angle_Yaw = 0;
		Auto_Angle_Pitch = 0;
		PITCH_AUTO_OFFSET = 0;
		YAW_AUTO_OFFSET = 0;
		follow_flag = 1;
		nuc_flag=1;
		
	}
	else if ( !nuc_update_flag)
	{
		float ___yaw = imu.yaw - imu_yaw_last;
		Auto_Angle_Yaw = Auto_Angle_Yaw - ___yaw;
		imu_yaw_last = imu.yaw;
	}
	else 
	{
		imu_yaw_last = imu.yaw;
		float XZ_ratio, YZ_ratio; 
		XZ_ratio = temp1 * 1.0f / temp3;
		YZ_ratio = temp2 * 1.0f / temp3;
		Auto_Angle_Yaw = atan(XZ_ratio) * 180 / PI;
		Auto_Angle_Pitch = atan(YZ_ratio) * 180 /PI;
		if(fabs(Auto_Angle_Yaw)>10)
			Auto_Angle_Yaw = (Auto_Angle_Yaw/fabs(Auto_Angle_Yaw))*10;

		//yaw_schmidtFlag yaw的斯密特触发器标志
		if(yaw_schmidtFlag==0 && (fabs(Auto_Angle_Yaw)<6))
		   yaw_schmidtFlag=1;
	  	else if(yaw_schmidtFlag==1 && (fabs(Auto_Angle_Yaw)>=11))
	    	 yaw_schmidtFlag=0;
		
	  	if((yaw_schmidtFlag==0)||((fabs(Auto_Angle_Pitch)>=PITCH_AUTO_WINDOWS)))
			   follow_flag = 1;
	  	else
			   follow_flag = 0;
			   //fabs(Auto_Angle_Pitch) < PITCH_AUTO_WINDOWS  
			   //fabs(Auto_Angle_Yaw) < 6 才能自动跟踪
		
		erro_Auto_Angle_Yaw = (Auto_Angle_Yaw - last_Auto_Angle_Yaw)*1000;
		last_Auto_Angle_Yaw = Auto_Angle_Yaw;
		nuc_update_flag = 0;
	}
	
	
	if(follow_flag==1)
	{
		gimbal_follow_mode();
	}
// current_yaw = imu_yaw;
// gimbal_follow_mode();
	else
	{
		pitch_target = set_auto_pitch_angle();
		pitch_feedback = GMPitchEncoder.real_angle;
		yaw_target = set_auto_yaw_angle();
		yaw_feedback = imu_yaw;
		gimbal_auto_action(&PitchAutoPID, pitch_target, pitch_feedback, &YawAutoPID, yaw_target, yaw_feedback);
		angle_yaw=1000*Auto_Angle_Yaw;
	}

}
//**********************************************************************
//* 功能： 通过PID控制云台                                             *
//* 参数： PitchPID：      Pitch轴PID结构体                            *
//* 参数： pitch_target：  pitch轴目标角度值                           *
//* 参数： pitch_feedback：pitch轴反馈角度值                           *
//* 参数： YawPID：        Yaw轴PID结构体                              *
//* 参数： yaw_target：    yaw轴目标角度值                             *
//* 参数： yaw_feedback：  yaw轴反馈角度值                             *
//* 返回值：无                                                         *
//**********************************************************************
void gimbal_action(PID_Regulator_Double_Loop_t *pitch_pid, float pitch_target,float pitch_feedback, PID_Regulator_Double_Loop_t *yaw_pid,float yaw_target,float yaw_feedback)
{
	//Pitch轴位置环
	pitch_pid->Position.ref = pitch_target;
	pitch_pid->Position.fdb = pitch_feedback;
	PID_Calc(&pitch_pid->Position);
	//Pitch轴速度环
	pitch_pid->Speed.ref = KalmanFilter2(pitch_pid->Position.output, KALMAN_Q, KALMAN_R);//增加卡尔曼滤波
	//pitch_pid->Speed.ref = pitch_pid->Position.output;                                 //无卡尔曼滤波
	pitch_pid->Speed.fdb = imu_data.gx * 0.061037019f;	//gx是原始值，需要乘上分辨率。(2000-(-2000))/(2^16-1)
	PID_Calc(&pitch_pid->Speed);
    //YAW位置环
    yaw_pid->Position.ref = yaw_target;
	yaw_pid->Position.fdb = yaw_feedback;
	PID_Calc(&yaw_pid->Position);
	//速度环
	//yaw_pid->Speed.ref = KalmanFilter2(yaw_pid->Position.output, KALMAN_Q, KALMAN_R); //增加卡尔曼滤波，不增加也比较稳定，所以先去掉了
	yaw_pid->Speed.ref = yaw_pid->Position.output;                                      //无卡尔曼滤波
	yaw_pid->Speed.fdb = -imu_data.gz * 0.061037019f;
	PID_Calc(&yaw_pid->Speed);                                                      //该PID增加过0解决方案，如果PItch存在过零，则也可以调用
	//pitch为can1控制，yaw为can2控制
	if(HAL_GetTick() - rc_tick >= 100)
		Set_Gimbal_Current(&hcan1, 0,0);
	else
		Set_Gimbal_Current(&hcan1, (int16_t)yaw_pid->Speed.output, (int16_t)pitch_pid->Speed.output);
	
	//测试代码
	//Set_Gimbal_Current(&hcan1, 0, (int16_t)pitch_pid->Speed.output);                  //单独调试pitch
	//Set_Gimbal_Current(&hcan1, (int16_t)yaw_pid->Speed.output, 0);                    //单独调试yaw
	//Set_Gimbal_Current(&hcan1, 0, 1000);                                              //pitch电机测试
	//Set_Gimbal_Current(&hcan2, 5000, 0);                                              //yaw电机测试
}



//**********************************************************************
//* 功能： 通过NUC 及 PID控制云台                                             *
//* 参数： PitchPID：      Pitch轴PID结构体                            *
//* 参数： pitch_target：  pitch轴目标角度值                           *
//* 参数： pitch_feedback：pitch轴反馈角度值                           *
//* 参数： YawPID：        Yaw轴PID结构体                              *
//* 参数： yaw_target：    yaw轴目标角度值                             *
//* 参数： yaw_feedback：  yaw轴反馈角度值                             *
//* 返回值：无                                                         *
//**********************************************************************
void gimbal_auto_action(PID_Regulator_Double_Loop_t *pitch_pid, float pitch_target,float pitch_feedback, PID_Regulator_Double_Loop_t *yaw_pid,float yaw_target,float yaw_feedback)
{
	//Pitch轴位置环
	pitch_pid->Position.ref = pitch_target;
	pitch_pid->Position.fdb = pitch_feedback;
	PID_Calc(&pitch_pid->Position);
	//Pitch轴速度环
	//pitch_pid->Speed.ref = KalmanFilter2(pitch_pid->Position.output, KALMAN_Q, KALMAN_R);//增加卡尔曼滤波
    pitch_pid->Speed.ref = pitch_pid->Position.output;                                 //无卡尔曼滤波
	pitch_pid->Speed.fdb = imu_data.gx * 0.061037019f;
	PID_Calc(&pitch_pid->Speed);

	
    //YAW位置环
  yaw_pid->Position.ref = yaw_target;//+v;
	yaw_pid->Position.fdb = yaw_feedback;
	PID_Calc(&yaw_pid->Position);
	//速度环
	//yaw_pid->Speed.ref = KalmanFilter2(yaw_pid->Position.output, KALMAN_Q, KALMAN_R); //增加卡尔曼滤波，不增加也比较稳定，所以先去掉了
	yaw_pid->Speed.ref = yaw_pid->Position.output;//+0.2*my_temp_v;                                      //无卡尔曼滤波
	yaw_pid->Speed.fdb = -imu_data.gz * 0.061037019f;
	PID_Calc(&yaw_pid->Speed);                                                      //该PID增加过0解决方案，如果PItch存在过零，则也可以调用
	//pitch为can1控制，yaw为can2控制
	YAW_SPEED_OUTPUT = yaw_pid->Speed.output;
  float tmp_ratio = 1;
	if ( yaw_pid->Speed.output < 0 ) tmp_ratio = 1.0;
	Set_Gimbal_Current(&hcan1,(int16_t)(tmp_ratio * yaw_pid->Speed.output), (int16_t)pitch_pid->Speed.output);
	//Set_Gimbal_Current(&hcan1, (int16_t)yaw_pid->Speed.output, 0);
	//Set_Gimbal_Current(&hcan1, 0, (int16_t)pitch_pid->Speed.output);
//	//测试代码
//	//Set_Gimbal_Current(&hcan1, 0, (int16_t)pitch_pid->Speed.output);                  //单独调试pitch
//	//Set_Gimbal_Current(&hcan2, (int16_t)yaw_pid->Speed.output, 0);                    //单独调试yaw
//	//Set_Gimbal_Current(&hcan1, 0, 1000);                                              //pitch电机测试
//	//Set_Gimbal_Current(&hcan2, 5000, 0);                                              //yaw电机测试
}







//**********************************************************************
//* 功能： pitch轴角度转换函数                                         *
//* 参数： 无                                                          *
//* 返回值：pitch轴绝对角度值                                          *
//**********************************************************************
float get_pitch_encoder_angle(void)
{
	return GMPitchEncoder.angle_raw_value * 0.0439453125f;
}
//**********************************************************************
//* 功能： yaw轴角度转换函数                                           *
//* 参数： 无                                                          *
//* 返回值：yaw轴绝对角度值                                            *
//**********************************************************************
float get_yaw_encoder_angle(void)
{
	return GMYawEncoder.angle_raw_value * 0.0439453125f;
}
//**********************************************************************
//* 功能： 设定遥控器控制pitch目标位置绝对角度值                         *
//* 参数： 无                                                          *
//* 返回值：current_pitch 遥控器控制pitch轴的绝对角度值                                *
//**********************************************************************
float set_pitch_angle(void)
{
	float k_mouse = 0.0010f;                   //鼠标灵敏度
	float k_controller = 0.5f;                //遥控器灵敏度
	float delta_angle;                         //变化角度
	if (rc.mouse.y == 0)
		delta_angle = k_controller * rc.LV;    //鼠标无操作，控制权为遥控器左摇杆垂直方向
	else
		delta_angle = -k_mouse * rc.mouse.y;   //控制权为鼠标
	current_pitch += delta_angle;              //更新目标绝对角度
	current_pitch = Pitch_Limit(current_pitch);//解决过零和俯仰限制
	return current_pitch;
}
//**********************************************************************
//* 功能： 遥控器控制yaw目标位置绝对角度值                             *
//* 参数： 无                                                          *
//* 返回值：遥控器控制yaw轴绝对角度值                                  *
//**********************************************************************
float set_yaw_angle(float k_mouse,float k_controller)
{
	float delta_angle ;                         //变化角度   
    int16_t rcmouse =  rc.mouse.x;	
	if(rc.mouse.x > 100) 
		  rcmouse=100;
	else if(rc.mouse.x < -100) 
		  rcmouse=-100;
	
	if (rc.mouse.x == 0) 
		delta_angle = -k_controller * rc.LH;   	//是鼠标无操作，控制权为遥控器左摇杆水平方向
	else 
		delta_angle = -k_mouse*rcmouse;    		//控制权为鼠标
    current_yaw += delta_angle;                	//更新目标绝对角度
	//current_yaw = Yaw_Limit(current_yaw);     //解决过零
	return current_yaw;
}


//**********************************************************************
//* 功能： NUC控制pitch目标位置绝对角度值                              *
//* 参数： 无                                                          *
//* 返回值：NUC控制pitch轴绝对角度值                                   *
//**********************************************************************
float set_auto_pitch_angle()
{
	float temp;
	temp=GMPitchEncoder.real_angle-Auto_Angle_Pitch+PITCH_AUTO_OFFSET;
	temp = Pitch_Limit(temp);
	current_pitch=temp;
  return temp;
}

//**********************************************************************
//* 功能： NUC控制yaw目标位置绝对角度值                              *
//* 参数： 无                                                          *
//* 返回值：NUC控制yaw轴绝对角度值                                   *
//**********************************************************************
float shoot_ratio = 0;
float set_auto_yaw_angle()
{
	float temp;
	my_temp_v =shoot_ratio*v;
	V_DETECT = 1000*v;
	

	if(fabs(Auto_Angle_Yaw)>2)
	{
		my_temp_v=0;
	}
	temp=imu_yaw-Auto_Angle_Yaw-YAW_AUTO_OFFSET-my_temp_v;
	current_yaw=temp;
  return temp;
}


//**********************************************************************
//* 功能： pitch轴范围限制和过零                                       *
//* 参数： pitch计算绝对角度值                                         *
//* 返回值：pitch修正绝对角度值                                        *
//**********************************************************************
float Pitch_Limit(float pitch_current)
{
	if (pitch_current > 360)                                     //大于360重新从0开始
		pitch_current = pitch_current - 360;
	else if (pitch_current < 0)                                  //小于0重新从360开始
		pitch_current = pitch_current + 360;
	
	if (pitch_current > PITCH_OFFSET_ANGLE + PITCH_MAX_ANGLE)                         //超过最大限制，取最大值
		pitch_current = PITCH_OFFSET_ANGLE + PITCH_MAX_ANGLE;
	else if (pitch_current < PITCH_OFFSET_ANGLE + PITCH_MIN_ANGLE)                    //超过最小限制，取最小值
		pitch_current = PITCH_OFFSET_ANGLE + PITCH_MIN_ANGLE;
	return pitch_current;
}
//**********************************************************************
//* 功能： yaw轴范围限制和过零                                         *
//* 参数： yaw计算绝对角度值                                           *
//* 返回值：yaw修正绝对角度值                                          *
//* 说明： 注释内容为非360度云台代码
//**********************************************************************
float Yaw_Limit(float yaw_current)
{
//	if (yaw_current > 360)                                     //大于360重新从0开始
//		yaw_current = yaw_current - 360;
//	else if (yaw_current < 0)                                  //小于0重新从360开始
//		yaw_current = yaw_current + 360;
	if (yaw_current > YAW_MAX_ANGLE)                         //超过最大限制，取最大值
		yaw_current = YAW_MAX_ANGLE;
	else if (yaw_current < YAW_MIN_ANGLE)                    //超过最小限制，取最小值
		yaw_current = YAW_MIN_ANGLE;
	return yaw_current;
}
