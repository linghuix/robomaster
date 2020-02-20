#include "chassis_task.h"
#include "pid.h"
#include "HW_beep.h"
#include "HW_steer.h"
chassis_t chassis;

//控制结果
extern R_or_K_Control_t RK_Control;

//最大调整方向速度设置
#define VX_MAX 3000             //mm/s   正方向：up
#define VY_MAX 3000             //mm/s   正方向：left
#define VW_MAX 550				//deg/s  正方向：逆时针



#define POWER_LIMIT 80.0f		// w
#define W_BUF 60.0f         //缓冲能量
#define Wd    30.0f         //缓冲能量危险值
#define W_BUF 60.0f					// j


/* chassis twist angle (degree)*/
#define TWIST_ANGLE    45
/* twist period time (ms) */
#define TWIST_PERIOD   10000.0


//keyboard speed set

#define CHASSIS_PERIOD 10	/* chassis control period time (ms) */
#define TIME_KEYAC 2000.0f	// ms


//chasis speed set
#define DegreeToRad PI / 180.0f


#define RADIAN_COEF 57.3f
/* the radius of wheel(mm) */
#define RADIUS     76
/* the perimeter of wheel(mm) */
#define PERIMETER  478
/* wheel track distance(mm) */
#define WHEELTRACK 380
/* wheelbase distance(mm) */
#define WHEELBASE  330
/* gimbal offset distance(mm) */
#define GIMBAL_X_OFFSET 0
#define GIMBAL_Y_OFFSET 0
/* chassis motor use 3508 */
/* the deceleration ratio of chassis motor */
#define CHASSIS_DECELE_RATIO (1.0f/19.0f)


chassis_t chassis;
chassis_t last_chassis;
uint32_t twist_count = 0;
extern char send_pwm[5];
extern uint8_t chassisoutput_ratio;
extern RC_Type  rc; 



//自动补弹
//ADC 采集到的数据
extern float distance[2];
#define ADC_datax_offset 23
#define ADC_datay_offset 28

//电容
uint8_t power_chaisispower;
uint8_t isErro;
uint8_t isChange;
uint8_t RemainPower;
uint32_t isStop_tick = 0;
extern uint32_t power_tick;
extern uint32_t judge_tick;
uint8_t isAccelerate;
uint8_t isBoom = 0;
int8_t twist_direction = 1;
float twist_speed = 0.0;



int current_wheel0;             //左前轮全局速度
int current_wheel1;             //右前轮全局速度
int current_wheel2;             //右后轮全局速度
int current_wheel3;             //左后轮全局速度


//LF 左前 0
//RF 右前 1
//LB 左后 3
//RB 右后 4
extern uint32_t wheel_can_rc[4];
uint8_t wheel_can_problem[4];
uint8_t wheel_problem[4];

//**********************************************************************
//* 功能： 底盘模式总控制函数                                          *
//* 参数： 无                                                          *
//* 返回值：无                                                         *
//**********************************************************************
void chassis_task(void)
{
	switch (Mode.Chassis)
	{
		case Chassis_Lock_Mode:  chassis_depart_mode(); break;
		case Chassis_Follow_Mode:chassis_follow_mode(); break;
		case Chassis_Twist_Mode :chassis_twist_mode(); break;
		case Chassis_Start_Mode: chassis_start_mode();break;
		case Chassis_Inspecion_Mode: chassis_inspection_mode();break;
		case Chassis_Hand_Start_Mode:chassis_hand_start_mode();break;
		case Chassis_Check_Mode:chassis_check_mode();break;
		default:                 chassis_depart_mode(); break;
	}
//chassis_depart_mode();
//	chassis_start_mode();
}


//**********************************************************************
//* 功能： 底盘手动取弹模式                                            *
//* 参数： 无                                                          *
//* 返回值：无                                                         *
//* 说明：底盘锁死模式，弹仓盖打开。只能左右上下平动+旋转                                    *
//**********************************************************************
void chassis_hand_start_mode()
{
	chassis_set_speed(0,1.0);      //平动方向控制
	Set_Steer_Duty(STEER_OPEN);
	chassis_action();          //pid及can发送电流
}

//**********************************************************************
//* 功能： 底盘检录模式                                                *
//* 参数： 无                                                          *
//* 返回值：无                                                         *
//* 说明：底盘输出为原来的30% ，只能左右上下平动+旋转                                 *
//**********************************************************************
void chassis_inspection_mode()
{
	chassis_set_speed(0,0.3);      //平动方向控制
	Set_Steer_Duty(STEER_OPEN);
	chassis_action();          //pid及can发送电流
	twist_speed = 0.0;
}


//**********************************************************************
//* 功能： 底盘自检模式                                                *
//* 参数： 无                                                          *
//* 返回值：无                                                         *
//* 说明：底盘输出值数案件为原来的30%。   按照 ID 的顺序来播
//* 	  放提示音，如果某一个轮子 can 线损坏，则会在对应的 ID 处报明亮
//* 	  的蜂鸣声。同时还针对四个轮的安装是否差异较大做了判断，发送电流
//*		  高于其他的电机，如果超过太多，就认为是这个轮子或者电机的安装存在问题。
//**********************************************************************
void chassis_check_mode()
{
	chassis_set_speed(0,1.0);      	//平动方向控制
	Set_Steer_Duty(STEER_ClOSE);
	chassis_action();          		//pid及can发送电流
	//自检
  	chassis_check(chassis.current[0],chassis.current[1],chassis.current[2],chassis.current[3]);
	
}


//**********************************************************************
//* 功能： 底盘自动取弹模式                                            *
//* 参数： 无                                                          *
//* 返回值：无                                                         *
//* 说明：底盘x轴和y轴会受ADC控制，自动定位，但是W分量仍然可以使用               *
//**********************************************************************
void chassis_start_mode()
{
	float kv_x = 20;
	float kv_y = 20;
	chassis_set_speed(0,1.0);      //平动方向控制
	chassis.vx =0;//(kv_x * (distance[0] - ADC_datax_offset));
	chassis.vy = 0;//(kv_y * ( distance[1] - ADC_datay_offset));
	Set_Steer_Duty(STEER_OPEN);	//打开
	chassis_action();          //pid及can发送电流
}


//**********************************************************************
//* 功能： 底盘陀螺模式                                                *
//* 描述： 在速度环的控制上，再加一个旋转角度的位置控制PID
//*		   以yaw轴的编码器角度作为反馈，并保持与yaw轴的角度为30或者60度，
//* 	   由于yaw轴也是跟着底盘运动的，所以反馈值不会随底盘变化而变化，
//*		   所以可以实现持续旋转，相当于输入特定的电压值驱动电机朝一个方向动。
//*
//* 参数： 无                                                          *
//* 返回值：无                                                         *
//**********************************************************************
void chassis_twist_mode()
{
	chassis_set_speed(-(GMYawEncoder.real_angle),1.0);      //平动方向控制
	Chassis_Twist_PID.fdb = GMYawEncoder.real_angle;
    if(isAccelerate)
	{
		Chassis_Twist_PID.ref = GMYawEncoder.real_angle+twist_direction*60;
    	twist_speed = 60.0;		
	}
	else
	{
	  	Chassis_Twist_PID.ref = GMYawEncoder.real_angle+twist_direction*30;	
		twist_speed = 30.0;
	}
	
	PID_Calc(&Chassis_Twist_PID);
	chassis.vw = -Chassis_Twist_PID.output;
	Set_Steer_Duty(STEER_ClOSE);
	chassis_action();          								//pid及can发送电流
}



//**********************************************************************
//* 功能： 底盘分离模式                                                *
//* 参数： 无                                                          *
//* 返回值：无                                                         *
//* 说明：底盘实现前后左右平动 和转动   。遥控器R设定速度，L设定旋转速度                                  *
//**********************************************************************
void chassis_depart_mode()
{
	chassis_set_speed(0,1.0);      //平动方向控制
	Set_Steer_Duty(STEER_ClOSE);
	chassis_action();          //pid及can发送电流
}


//**********************************************************************
//* 功能： 底盘跟随模式                                                *
//* 参数： 无                                                          *
//* 返回值：无                                                         *
//* 说明：底盘会跟随云台yaw运动，yaw生成的反馈角度，限制在[-180,180]，回归到默认角度处。                                 *
//**********************************************************************
void chassis_follow_mode()
{
	chassis_set_speed(-GMYawEncoder.real_angle,1.0);      //平动方向控制
	ChassisVWPID.ref = 0;
	if(GMYawEncoder.real_angle>180)
	{
	  	ChassisVWPID.fdb = GMYawEncoder.real_angle-360;
	}
	else if(GMYawEncoder.real_angle<-180)
	{
		ChassisVWPID.fdb = GMYawEncoder.real_angle+360;
	}
	else
	{
		ChassisVWPID.fdb = GMYawEncoder.real_angle;
	}
	
	PID_Calc(&ChassisVWPID);
	
	if(ChassisVWPID.output<50&&ChassisVWPID.output>-50)
	{
		ChassisVWPID.output = 3*ChassisVWPID.output;
	}
	if(ChassisVWPID.output>300||ChassisVWPID.output<-300)
	{
		ChassisVWPID.output =300+ 0.50*(ChassisVWPID.output-300);
	}
	if(GMYawEncoder.real_angle > 30||GMYawEncoder.real_angle <-30)
	{
		ChassisVWPID.output = 200*ChassisVWPID.output/fabs(ChassisVWPID.output);
	}
	
	chassis.vw = -ChassisVWPID.output;
	twist_speed = 0.0;
	Set_Steer_Duty(STEER_ClOSE);
	chassis_action();          //pid及can发送电流
	
}



float lastRV_WS, RV_WS;//前后移动速度，遥控器中的右侧垂直遥杆
float lastRH_AD, RH_AD;//左右
float lastLH_Mouse_x, LH_Mouse_x;//旋转
void keyboard_set_speed(float*VFront,float*VLeftRight,float*Rotate);
void limit(float *x, float down, float up);

//**********************************************************************
//* 功能： 底盘速度设定                                             *
//* 参数： angle:相对小车方向角，逆时针为正，理想下为0     		     	       *
//* 参数： ratio:速度比例控制，平动和旋转速度都控制                                         *
//* 返回值：无                                                  *
//* 简介： 遥控器R设定速度，L设定旋转速度
//*		   鼠标 - 
//**********************************************************************

/**xlh:原本鼠标与遥控器的控制是独立的，初始值和状态值都是分开记录的.
	   控制对象发生变化时，会出现错误**/
void chassis_set_speed(float angle,float ratio)
{
	//float RV = 0;
	//float RH = 0;
	//float LH = 0;

	angle = angle * DegreeToRad;

	
	if (RK_Control == Remote_Control)  //键盘无操作，则用遥控器赋值
	{
		/** 控制器操作 */
		RV_WS = 0.1*rc.RV+0.9*lastRV_WS;     	//遥控器一阶低通滤波
		RH_AD = 0.1*rc.RH+0.9*lastRH_AD;
		LH_Mouse_x = -(0.1*rc.LH+0.9*lastLH_Mouse_x);
		
		lastRV_WS = rc.RV;
		lastRH_AD = rc.RH;
		lastLH_Mouse_x = rc.LH;
	}
	else{
		/** 电脑键盘操作 */
		keyboard_set_speed(&RV_WS, &RH_AD, &LH_Mouse_x);
	}
	
	// 沿angle方向的速度分量计算
	last_chassis = chassis;
	chassis.vx =  (RV_WS * cos(angle) - RH_AD * sin(angle))*VX_MAX*ratio;
	chassis.vy = -(RV_WS * sin(angle) + RH_AD * cos(angle))*VY_MAX*ratio;
	chassis.vw =  LH_Mouse_x * VW_MAX * ratio;
	chassis.ax = chassis.vx - last_chassis.vx;
	chassis.ay = chassis.vy - last_chassis.vy;
	
//	if(Mode.Chassis!=Chassis_Twist_Mode)
//	{
//	  if((chassis.ax>0)&&(chassis.ay==0))
//		{
//			ChassisPID[0].outputMax = 3000;
//			ChassisPID[1].outputMax = 3000;
//			ChassisPID[2].outputMax = 13000;
//			ChassisPID[3].outputMax = 13000;
//		}
//		else if((chassis.ax<0)&&(chassis.ay==0))
//		{
//			ChassisPID[0].outputMax = 13000;
//			ChassisPID[1].outputMax = 13000;
//			ChassisPID[2].outputMax = 3000;
//			ChassisPID[3].outputMax = 3000;
//		}
//		else if((chassis.ay>0)&&(chassis.ax==0))
//		{
//			ChassisPID[0].outputMax = 3000;
//			ChassisPID[1].outputMax = 13000;
//			ChassisPID[2].outputMax = 13000;
//			ChassisPID[3].outputMax = 3000;
//		}
//		else if((chassis.ay<0)&&(chassis.ax==0))
//		{
//			ChassisPID[0].outputMax = 13000;
//			ChassisPID[1].outputMax = 3000;
//			ChassisPID[2].outputMax = 3000;
//			ChassisPID[3].outputMax = 13000;
//		}
//	}

}

/** 键盘WASD，设定速度 **/
void keyboard_set_speed(float*VFront,float*VLeftRight,float*Rotate)
{

	//static float ramp[3] = { 0 };
	//static float lastramp[3] = { 0 };
	float k_mouse = 6.0f;                    //鼠标灵敏度
	float dramp = CHASSIS_PERIOD / TIME_KEYAC;//阻尼


	//W S A D 误操作，减速
	if (!rc.kb.bit.W && !rc.kb.bit.S && !rc.kb.bit.A && !rc.kb.bit.D)
		dramp = 2;
	
	//按下SHIFT，加速和减速
	if (rc.kb.bit.SHIFT)
		dramp = 2;

	if (rc.kb.bit.W && !rc.kb.bit.S)     	 //按下W不按S
		RH_AD += dramp;
	else if (rc.kb.bit.S && !rc.kb.bit.W)    //按下S不按W
		RH_AD -= dramp;
	else                                     //未操作，缓慢减速
	{
		if (RH_AD > 0)
		{
			RH_AD -= dramp;
			if (RH_AD < 0)
			{
				RH_AD = 0;
			}
		}
		else if (RH_AD < 0)
		{
			RH_AD += dramp;
			if (RH_AD > 0)
			{
				RH_AD = 0;
			}
		}
	}

	if (rc.kb.bit.A && !rc.kb.bit.D)         //按下A不按D
		RV_WS -= dramp;
	else if (rc.kb.bit.D && !rc.kb.bit.A)    //按下D不按A
		RV_WS += dramp;
	else                                     //归零操作，未操作，缓慢减速
	{
		if (RV_WS > 0)
		{
			RV_WS -= dramp;
			if (RV_WS < 0)
			{
				RV_WS = 0;
			}
		}
		else if (RV_WS < 0)
		{
			RV_WS += dramp;
			if (RV_WS > 0)
			{
				RV_WS = 0;
			}
		}		
	}
	//范围限制
	limit(&RV_WS, -1, 1);
	limit(&RH_AD, -1, 1);
	limit(&LH_Mouse_x, -1, 1);

	//键盘一阶低通滤波
	*VFront = 0.1*RV_WS+0.9*lastRV_WS;
	*VLeftRight = 0.1*RH_AD+0.9*lastRH_AD;
	*Rotate = -k_mouse * rc.mouse.x / VW_MAX;

	lastRH_AD = RH_AD;
	lastRV_WS = RV_WS;
}


void limit(float *x, float down, float up)
{
	if(*x > up)
		*x = up;
	if(*x < down)
		*x = down;
}


//**********************************************************************
//* 功能： 底盘速度分解                                                *
//* 参数： vx：x方向速度                                               *
//* 参数： vy：y方向速度                                               *
//* 参数： vw：w方向速度                                               *
//* 参数： speed[]：分解结束后四个轮子的速度                           *
//* 返回值：无                                                         *
//**********************************************************************
void mecanum_calc(float vx, float vy, float vw, int16_t speed[]) {
	static float rotate_ratio_fr;
	static float rotate_ratio_fl;
	static float rotate_ratio_bl;
	static float rotate_ratio_br;
	static float wheel_rpm_ratio;
  
	chassis.rotate_x_offset = GIMBAL_X_OFFSET;
	chassis.rotate_y_offset = GIMBAL_Y_OFFSET;

	rotate_ratio_fr = ((WHEELBASE + WHEELTRACK) / 2.0f - chassis.rotate_x_offset + chassis.rotate_y_offset) / RADIAN_COEF;
	rotate_ratio_fl = ((WHEELBASE + WHEELTRACK) / 2.0f - chassis.rotate_x_offset - chassis.rotate_y_offset) / RADIAN_COEF;
	rotate_ratio_bl = ((WHEELBASE + WHEELTRACK) / 2.0f + chassis.rotate_x_offset - chassis.rotate_y_offset) / RADIAN_COEF;
	rotate_ratio_br = ((WHEELBASE + WHEELTRACK) / 2.0f + chassis.rotate_x_offset + chassis.rotate_y_offset) / RADIAN_COEF;

	wheel_rpm_ratio = 60.0f / (PERIMETER*CHASSIS_DECELE_RATIO);

	VAL_LIMIT(vx, -VX_MAX, VX_MAX);  //mm/s
	VAL_LIMIT(vy, -VY_MAX, VY_MAX);  //mm/s
	VAL_LIMIT(vw, -VW_MAX, VW_MAX);  //deg/s

	int16_t wheel_rpm[4];
	wheel_rpm[FR] = (-vx - vy - vw * rotate_ratio_fr) * wheel_rpm_ratio;
	wheel_rpm[FL] = (vx - vy - vw * rotate_ratio_fl) * wheel_rpm_ratio;
	wheel_rpm[BL] = (vx + vy - vw * rotate_ratio_bl) * wheel_rpm_ratio;
	wheel_rpm[BR] = (-vx + vy - vw * rotate_ratio_br) * wheel_rpm_ratio;

	memcpy(speed, wheel_rpm, 4 * sizeof(int16_t));
}
uint8_t isAccellerate[4];
uint8_t isaaaaaa;
//**********************************************************************
//* 功能： 底盘PID双环计算以及发送can电流控制指令                                          *
//* 参数： 无                                                          *
//* 返回值：无                                                         *
//**********************************************************************
void chassis_action(void) {
	int i;
//	static int16_t lastwheelcurrent[4];
//	static uint16_t all_current;
//	static uint16_t last_all_current;
	mecanum_calc(chassis.vx, chassis.vy, chassis.vw, chassis.wheel_spd_ref);
	for (i = 0; i<4; i++) 
	{
		chassis.wheel_spd_fdb[i] = CMEncoder[i].velocity;
		ChassisPID[i].fdb = chassis.wheel_spd_fdb[i];
		ChassisPID[i].ref = chassis.wheel_spd_ref[i];
		PID_Calc(&ChassisPID[i]);
		chassis.current[i] = ChassisPID[i].output;
	}
	//因为速度和电流没有明确对应，这里面只是扩大一定的倍数
	current_wheel0 = chassis.wheel_spd_fdb[0] * 1000;
	current_wheel1 = chassis.wheel_spd_fdb[1] * 1000;
	current_wheel2 = chassis.wheel_spd_fdb[2] * 1000;
	current_wheel3 = chassis.wheel_spd_fdb[3] * 1000;
//	//死区控制
//	for(int i=0;i<4;++i)
//	{
//		if((chassis.current[i]<1100)&&(chassis.current[i]>-1100))
//		{
//			chassis.current[i] = 0;
//		}
//	}
//	if(chassisoutput_ratio == 0)
//	{
//		chassisoutput_ratio = 40;
//	}
	
//	all_current = fabs(chassis.wheel_spd_fdb[0])+fabs(chassis.wheel_spd_fdb[1])+fabs(chassis.wheel_spd_fdb[2])+fabs(chassis.wheel_spd_fdb[3]);
//	if((all_current - last_all_current)>2500)
//	{
//		isAccellerate[3]=0;
//	}
//	else
//	{
//		isAccellerate[3]=1;
//	}
//	isAccellerate[2] = chassispowerbuffer;
//	isAccellerate[1] = 0xbb;
//	isAccellerate[0] = 0xaa;
//	isaaaaaa = isAccellerate[3];
//	last_all_current = all_current;
	//HAL_UART_Transmit(&huart2, isAccellerate, 4, 1000);
	chassisoutput_ratio = 100;
	 
	 //power_limit();
	 if((HAL_GetTick() - rc_tick >= 100)||(HAL_GetTick() -isStop_tick<=100))
		Set_Chassis_Current(&hcan2, 0,0,0,0);
	else
		Set_Chassis_Current(&hcan2,chassisoutput_ratio*(chassis.current[0])/100,chassisoutput_ratio*(chassis.current[1])/100,chassisoutput_ratio*(chassis.current[2])/100,chassisoutput_ratio*(chassis.current[3])/100);
	//Set_Chassis_Current(&hcan2, 1000, current_wheel1, current_wheel2, current_wheel3);

}


//**********************************************************************
//* 功能： 随动模式下摆动方向设置                                      *
//* 参数： k_vw：摆动的灵敏度，及偏差乘以该数字                        *
//* 返回值：实际计算的w大小和方向                                      *
//*说明：该函数主要用来将当前的0到360的码盘值与缺省码盘值比较，包括解决*
//*      过0问题，从而计算w方向和大小。                                *
//*注意：该函数是以w为正数按照正方向的逆时针旋转                       *
//*warning：！！！！！！！！测试时把底盘离地测试！！！！切记           *
//         调整确定正方向没问题才能放在地上测试                        *
//**********************************************************************
//float set_chassis_vw(float k_vw)
//{
//	//获取0到360码盘值
//	float encoder_angle = GMYawEncoder.angle_raw_value * 0.0439453125f;
//	//返回的偏差值
//	//float erro_value = CHASSIS_FOLLOW_ENCODER_OFFSET - encoder_angle;
//	//过零处理
//	if (erro_value < -180)
//		erro_value = erro_value + 360;
//	else if (erro_value > 180)
//		erro_value = erro_value - 360;
//	return erro_value;
//}




//待测试部分
void power_limit()
{
	uint16_t my_chassispowerbuffer;
	uint8_t TxMessage[3];
	float Pmax;
	int i;
	float current = 0;
  isAccelerate = rc.kb.bit.SHIFT;
	TxMessage[0] = (uint8_t)chassispowerbuffer;
	TxMessage[1] = isAccelerate;
	TxMessage[2] = isBoom;

	CAN_Send_Msg(&hcan2, TxMessage, 0x111, 3);
	isBoom = 0;
	if((isErro == 0)||(HAL_GetTick()- power_tick>500))
	{
		my_chassispowerbuffer = chassispowerbuffer;
	}
	else
	{
		my_chassispowerbuffer = power_chaisispower;
	}
	if(HAL_GetTick() - judge_tick > 300)
	{
		my_chassispowerbuffer = 0;
	}
	if(my_chassispowerbuffer>Wd)
	{
		Pmax = ((my_chassispowerbuffer - Wd)/0.02);
		if(Pmax < POWER_LIMIT)
		{
			Pmax = POWER_LIMIT;
		}
	}
	else
	{
		Pmax = POWER_LIMIT;
	}
	for(i = 0;i<4;i++)
	{
		current +=fabs(20 * chassis.current[i]/16384.0f);
	}
	if(current*24>=Pmax)
	{
		for(i = 0;i<4;i++)
		{
			chassis.current[i] *=Pmax*1.0f/(24.0f*current);
		}
	}
}

//**********************************************************************
//* 功能： 底盘问题检测函数                                            *
//* 参数： 输入控制电流  LF 左前 0  RF 右前 1  LB 左后 2  RB 右后 3                  *
//* 返回值：无                                                         *
//**********************************************************************
void chassis_check(int16_t LFcurrent,int16_t RFcurrent,int16_t LBcurrent,int16_t RBcurrent)
{
	uint8_t i;
	
	uint16_t abscurrent[4];//电流绝对值
	uint16_t mincurrent;
	float percent = 0.6;	
	abscurrent[0] = fabs(LFcurrent);
	abscurrent[1] = fabs(RFcurrent);
	abscurrent[2] = fabs(LBcurrent);
	abscurrent[3] = fabs(RBcurrent);
	
	//寻找最小电流
	mincurrent = abscurrent[0];
	for(i = 1;i < 4;i++)
	{
		if(abscurrent[i] < mincurrent)
			mincurrent = abscurrent[i];
	}
	
	for(i = 0;i<4;++i)
	{
		//如果我们某一个轮子的安装存在问题，那么势必会导致我们在实现统一转速的情况下这一个电机的电流要高于其他的电机
		wheel_problem[i] = (abscurrent[i]>(percent*mincurrent)&&(abscurrent[i]>2000))?1:0;

		if((HAL_GetTick()-wheel_can_rc[i])>100)//can线接收超时
			wheel_can_problem[i] = 1;
		else
			wheel_can_problem[i] = 0;
	}
	
	chassis_beep(wheel_problem[0],wheel_problem[1],wheel_problem[2],wheel_problem[3],wheel_can_problem[0],wheel_can_problem[1],wheel_can_problem[2],wheel_can_problem[3]);
	
}


//xlh:检查底盘四个轮子是否安装在同一个圆上，检查要求：小车需要进行旋转操作
void chassis_Mount_check(void)
{
	//寻找最大，最小轮子速度，四个轮子的平均速度
	int16_t errorDeviation = 0.1;//
	int16_t minSpeed = chassis.wheel_spd_fdb[0];
	int16_t maxSpeed = chassis.wheel_spd_fdb[0];
	int16_t average=0;
	
	for(int i = 0;i < 4;i++)
	{
		average += chassis.wheel_spd_fdb[i];
		if(chassis.wheel_spd_fdb[i] < minSpeed){
			minSpeed = chassis.wheel_spd_fdb[i];
		}
		else if(chassis.wheel_spd_fdb[i] > maxSpeed){
			maxSpeed = chassis.wheel_spd_fdb[i];
		}
	}

	//如果四个轮子不在同一个圆上，那么在进行旋转运动的时候，必然会出现较大的偏差
	//
	if((maxSpeed-minSpeed)>errorDeviation*maxSpeed){
		if((maxSpeed-average)>errorDeviation*maxSpeed/2){
			warning(1, Do1M);
		}
		else if((average-minSpeed)>errorDeviation*maxSpeed/2){
			warning(2, Do1M);
		}
		else
			warning(3, Do1M);
	}
	
}




