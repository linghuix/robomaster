#include "gimbal_task.h"
#include "HW_nuc.h"

void set_pitch_follow_Encoder(float* target, float *feedback, int is_custom, float target_custom);
void set_yaw_follow_Encoder(float* target, float *feedback, int is_custom, float target_custom);
void set_yaw_follow_IMU(float* target, float *feedback, int is_custom, float target_custom);


#define NORMAL_K_MOUSE        0.0030f   /*控制输入与实际控制值的控制系数K*/
#define NORMAL_K_CONTROLLER   0.18f
#define SUPPLY_K_MOUSE        0.0015f   /*鼠标*/
#define SUPPLY_K_CONTROLLER   0.12f     /*遥控器*/

#define PITCH_AUTO_WINDOWS    10        /*限制 自动瞄准的角度变化 的窗口*/
#define YAW_AUTO_WINDOWS       7

float PITCH_AUTO_OFFSET;    /*根据弹道的补偿*/      //shoot_task.c fric_wheel_ctrl();
float YAW_AUTO_OFFSET;                              //shoot_task.c fric_wheel_ctrl();
float current_pitch = PITCH_OFFSET_ANGLE;           //pitch轴全局绝对位置量,由于编码器反馈的是绝对值,所以可以初始化
float current_yaw;                                  /*yaw轴,IMU反馈的是相对的角度,所以没必要初始化*/


extern float v;
float last_v = 0;
float my_temp_v = 0;

extern int nuc_flag, nuc_update_flag;
extern short pre_temp1,pre_temp3;
extern short temp1,temp2,temp3;

//int FLAG = 0;
int IMUerro;
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
	imu_yaw_last = imu.yaw;
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
	float pitch_target, pitch_feedback;
	float yaw_target, yaw_feedback;
	
	if(Last_Mode.Gimbal == Gimbal_Follow_Mode)
		gimbal_round_now = gimbal_round;

	set_pitch_follow_Encoder(&pitch_target, &pitch_feedback, 0, 0);
	set_yaw_follow_Encoder(&yaw_target, &yaw_feedback, 1, gimbal_round_now * 360);
	current_yaw = imu_yaw;

	gimbal_action(&PitchPID, pitch_target, pitch_feedback, &YawPID, yaw_target, yaw_feedback);
}


#define  PITCHTIMEOUT 	100
#define  YAWIMEOUT 		100

int16_t  pitch_timeout_ms = PITCHTIMEOUT;//需要與零比較，所以為int類型
int16_t  yaw_timeout_ms = YAWIMEOUT;
uint8_t  timeout_ms = 0;

uint8_t  pitch_error = 2;
uint8_t  yaw_error = 2;
uint8_t  test_step = 1;
uint8_t  testFlag;

void reach_testPitchPosition(uint16_t pos, uint8_t next_testStep);
void reach_testYawPosition(uint16_t pos, uint8_t next_testStep);
//float abs(float num);
void test_passed(void);
void warningERROR(void);
//**********************************************************************
//* 功能： 云台自检模式                                      		     	      *
//* 参数： 无                                                      		      *
//* 返回值：无                                                   		      *
//* 说明： 实现云台全程的运动，检测控制error和控制时间是否在要求范围内                        *
//**********************************************************************

void gimbal_check_mode()
{
	switch(test_step)
	{
		case 1:
			reach_testPitchPosition(PITCH_OFFSET_ANGLE+PITCH_MAX_ANGLE-5, 2);
			break;
		case 2:
			reach_testPitchPosition(PITCH_OFFSET_ANGLE+PITCH_MIN_ANGLE+5, 3);
			break;
		case 3:
			reach_testYawPosition(PITCH_OFFSET_ANGLE+PITCH_MIN_ANGLE-5, 4);
			break;
		case 4:
			reach_testYawPosition(PITCH_OFFSET_ANGLE+PITCH_MIN_ANGLE+5, 1);
			while(test_step == 1){test_passed();}
			break;
		default:
			break;
	}
}




//**********************************************************************
//* 功能： 云台跟随模式                                      		     	      *
//* 参数： 无                                                      		      *
//* 返回值：无                                                   		      *
//* 说明：云台pitch,yaw跟随控制指令运动，yaw的反馈为imu，pitch反馈为编码器。                         *
//**********************************************************************


void gimbal_follow_mode(void)
{
	float pitch_target, pitch_feedback;
	float yaw_target, yaw_feedback;
	
	set_pitch_follow_Encoder(&pitch_target, &pitch_feedback, 0, 0);
	set_yaw_follow_IMU(&yaw_target, &yaw_feedback, 0, 0);
	gimbal_action(&PitchPID, pitch_target, pitch_feedback, &YawPID, yaw_target, yaw_feedback);
}




//**********************************************************************
//* 功能： 云台自瞄模式                                                
//* 参数： 无                                                          
//* 返回值：无  
//* 说明：根据从nuc获取的temp1234，自动计算yaw和pitch。当预测的yaw，pitch与当前值相差很大时自动进入follow mode
//**********************************************************************
/*float NUCstop_pitch;
float NUCstop_yaw;
float v;*/
extern float IMU_yaw, pre_IMU_yaw;
extern float errotime;

uint8_t yaw_schmidtFlag = 0;

int Cal_and_check(void);
void Cal_Auto_Angle(void);
uint8_t schmidt(float angle, float L, float H);
void gimbal_auto_follow(void);
void gimbal_auto_mode()
{
	uint8_t AutoMode_fail;

	AutoMode_fail = Cal_and_check();
	
	if(AutoMode_fail == 1){
		gimbal_follow_mode();
	}
	else{
        gimbal_auto_follow();
	}

}


void gimbal_auto_follow(void){
	float target_custom;
	float pitch_target, pitch_feedback;
	float yaw_target, yaw_feedback;
	
    target_custom = set_auto_pitch_angle();
	set_pitch_follow_Encoder( &pitch_target, &pitch_feedback, 1, target_custom);
	target_custom = set_auto_yaw_angle();
	set_yaw_follow_IMU(&yaw_target, &yaw_feedback, 1, target_custom);
	gimbal_auto_action(&PitchAutoPID, pitch_target, pitch_feedback, &YawAutoPID, yaw_target, yaw_feedback);

    return;
}

int Cal_and_check(void)
{

	uint8_t AutoMode_fail;
	//获取nuc数据的时刻 和 目前时间 超过100时，退出auto模式
	if(HAL_GetTick() - nuc_tick > 100){
		AutoMode_fail = 1;
		Auto_Angle_Yaw = 0;
		Auto_Angle_Pitch = 0;
		PITCH_AUTO_OFFSET = 0;
		YAW_AUTO_OFFSET = 0;
		nuc_flag=1;
	}
	else if ( !nuc_update_flag){
		AutoMode_fail = 0;
		float ___yaw = imu.yaw - imu_yaw_last;
		Auto_Angle_Yaw = Auto_Angle_Yaw - ___yaw;
	}
	else{
		Cal_Auto_Angle();
		yaw_schmidtFlag = schmidt(Auto_Angle_Yaw, 6, 11);
		if((fabs(Auto_Angle_Yaw)>=PITCH_AUTO_WINDOWS)){
			Auto_Angle_Yaw = Auto_Angle_Yaw/(fabs(Auto_Angle_Yaw)*YAW_AUTO_WINDOWS);
		}
	  	if((yaw_schmidtFlag==0)||((fabs(Auto_Angle_Pitch)>=PITCH_AUTO_WINDOWS)))
			   AutoMode_fail = 1;
	  	else
			   AutoMode_fail = 0;
		
		erro_Auto_Angle_Yaw = (Auto_Angle_Yaw - last_Auto_Angle_Yaw)*1000;
		last_Auto_Angle_Yaw = Auto_Angle_Yaw;
		nuc_update_flag = 0;
	}
	return AutoMode_fail;
}

void Cal_Auto_Angle(void){
	float XZ_ratio, YZ_ratio; 
	XZ_ratio = temp1 * 1.0f / temp3;
	YZ_ratio = temp2 * 1.0f / temp3;
	Auto_Angle_Yaw = atan(XZ_ratio) * 180 / PI;
	Auto_Angle_Pitch = atan(YZ_ratio) * 180 /PI;

}


//斯密特触发器
uint8_t schmidt(float angle, float L, float H){
	if(yaw_schmidtFlag==0 && (fabs(angle)<L))
	   	yaw_schmidtFlag=1;
	else if(yaw_schmidtFlag==1 && (fabs(angle)>=H))
		yaw_schmidtFlag=0;
	return yaw_schmidtFlag;
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
	if(HAL_GetTick() - rc_tick >= 100){
		Set_Gimbal_Current(&hcan1, 0, 0);
        return;
    }

	//Pitch轴位置环
	pitch_pid->Position.ref = pitch_target;
	pitch_pid->Position.fdb = pitch_feedback;
	PID_Calc(&pitch_pid->Position);
	//Pitch轴速度环
	pitch_pid->Speed.ref = KalmanFilter2(pitch_pid->Position.output, KALMAN_Q, KALMAN_R);
	pitch_pid->Speed.fdb = imu_data.gx * 0.061037019f;	        //gx是原始值，需要乘上分辨率。(2000-(-2000))/(2^16-1)
	PID_Calc(&pitch_pid->Speed);
    //YAW位置环
    yaw_pid->Position.ref = yaw_target;
	yaw_pid->Position.fdb = yaw_feedback;
	PID_Calc(&yaw_pid->Position);
	//YAW速度环
	//无卡尔曼滤波,不增加也比较稳定，所以先去掉了
	yaw_pid->Speed.ref = yaw_pid->Position.output;      
	yaw_pid->Speed.fdb = -imu_data.gz * 0.061037019f;
	PID_Calc(&yaw_pid->Speed);

	//pitch,yaw为can1控制
	Set_Gimbal_Current(&hcan1, (int16_t)yaw_pid->Speed.output, (int16_t)pitch_pid->Speed.output);
	
	//测试代码
	//Set_Gimbal_Current(&hcan1, 0, (int16_t)pitch_pid->Speed.output);                  //单独调试pitch
	//Set_Gimbal_Current(&hcan1, (int16_t)yaw_pid->Speed.output, 0);                    //单独调试yaw
	//Set_Gimbal_Current(&hcan1, 0, 1000);                                              //pitch电机测试
	//Set_Gimbal_Current(&hcan2, 5000, 0);                                              //yaw电机测试
}

//**********************************************************************
//* 功能： 通过NUC 及 PID控制云台                                      *
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
    pitch_pid->Speed.ref = pitch_pid->Position.output;
	pitch_pid->Speed.fdb = imu_data.gx * 0.061037019f;
	PID_Calc(&pitch_pid->Speed);

    //YAW位置环
    yaw_pid->Position.ref = yaw_target;//+v;
	yaw_pid->Position.fdb = yaw_feedback;
	PID_Calc(&yaw_pid->Position);
	//速度环
	yaw_pid->Speed.ref = yaw_pid->Position.output;//+0.2*my_temp_v;                                      //无卡尔曼滤波
	yaw_pid->Speed.fdb = -imu_data.gz * 0.061037019f;
	PID_Calc(&yaw_pid->Speed);                                                      
	
    //pitch,yaw为can1控制 ????
	YAW_SPEED_OUTPUT = yaw_pid->Speed.output;
    float tmp_ratio = 1;
	if ( yaw_pid->Speed.output < 0 ) 
        tmp_ratio = 1.0;
	Set_Gimbal_Current(&hcan1,(int16_t)(tmp_ratio * yaw_pid->Speed.output), (int16_t)pitch_pid->Speed.output);

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
	current_pitch = GMPitchEncoder.real_angle-Auto_Angle_Pitch+PITCH_AUTO_OFFSET;
	current_pitch = Pitch_Limit(current_pitch);
  return current_pitch;
}

//**********************************************************************
//* 功能： NUC控制yaw目标位置绝对角度值                              *
//* 参数： 无                                                          *
//* 返回值：NUC控制yaw轴绝对角度值                                   *
//**********************************************************************
float shoot_ratio = 0;
float set_auto_yaw_angle()
{
	my_temp_v =shoot_ratio*v;
	V_DETECT = 1000*v;
	
	if(fabs(Auto_Angle_Yaw)>2){
		my_temp_v=0;
	}
	current_yaw = imu_yaw-Auto_Angle_Yaw-YAW_AUTO_OFFSET-my_temp_v;
  return current_yaw;
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

void reach_testPitchPosition(uint16_t pos, uint8_t next_testStep){
	float pitch_target, yaw_target;
	float pitch_feedback, yaw_feedback;


	pitch_target = pos;
	pitch_feedback = GMPitchEncoder.real_angle;
	yaw_target = gimbal_round_now * 360; 			//固定在同一个位置
	yaw_feedback = GMYawEncoder.real_angle;	        //通过码盘反馈当前位置
	gimbal_action(&PitchPID, pitch_target, pitch_feedback, &YawPID, yaw_target, yaw_feedback);

 	if(fabs(pitch_target-pitch_feedback)>pitch_error){
		pitch_timeout_ms--;
	}
	else{
		test_step = next_testStep;
		pitch_timeout_ms = PITCHTIMEOUT;
		yaw_timeout_ms = YAWIMEOUT;
	}
	
	if(pitch_timeout_ms < 0){
		while(1){
			warningERROR();
		}
	}
}

void reach_testYawPosition(uint16_t pos, uint8_t next_testStep){
	float pitch_target, yaw_target;
	float pitch_feedback, yaw_feedback;


	pitch_target = pitch_feedback;
	pitch_feedback = GMPitchEncoder.real_angle;
	yaw_target = pos; 			//固定在同一个位置
	yaw_feedback = imu_yaw;	        //通过码盘反馈当前位置
	gimbal_action(&PitchPID, pitch_target, pitch_feedback, &YawPID, yaw_target, yaw_feedback);

 	if(fabs(yaw_target - yaw_feedback)>yaw_error){
		yaw_timeout_ms--;
	}
	else{
		test_step = next_testStep;
		pitch_timeout_ms = PITCHTIMEOUT;
		yaw_timeout_ms = YAWIMEOUT;
	}
	
	if(yaw_timeout_ms < 0){
		while(1){
			warningERROR();
		}
	}
}

/*float abs(float num){

	//num = num? num:-num;
	//return num;
	return fabs(num);
}
*/

void test_passed(void){
	return;
}

void warningERROR(void){
	return;
}


void testPitch(void){
	
}


void set_pitch_follow_Encoder(float* target, float *feedback, int is_custom, float target_custom){
	if(is_custom == 1){
		*target = target_custom;//自定义目标位置
	}
	else{
		*target = set_pitch_angle();				//通过 外部控制器 设置目标位置
	}
	
	*feedback = GMPitchEncoder.real_angle;
}


void set_yaw_follow_Encoder(float* target, float *feedback, int is_custom, float target_custom){
	if(is_custom == 1){
		*target = target_custom;//自定义目标位置
	}
	else{
		*target = set_yaw_angle(NORMAL_K_MOUSE, NORMAL_K_CONTROLLER);;				//通过 外部控制器 设置目标位置
	}
	
	*feedback = GMYawEncoder.real_angle;
}


void set_yaw_follow_IMU(float* target, float *feedback, int is_custom, float target_custom){
	if(is_custom == 1){
		*target = target_custom;//自定义目标位置
	}
	else{
		*target = set_yaw_angle(NORMAL_K_MOUSE, NORMAL_K_CONTROLLER);;				//通过 外部控制器 设置目标位置
	}

	*feedback = imu_yaw;                         	//通过IMU四元数结算反馈当前位置
}


