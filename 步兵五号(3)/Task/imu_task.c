#include "imu_task.h"
#include "gimbal_task.h"
#include "HW_imu.h"
#include "pid.h"
#include "math.h"
//#include "arm_math.h"


//相关计算使用的结构体
imu_data_t     imu;
int IMU_YAW;
int IMU_YAW_SPEED = 0;
/* imu task static parameter */
static volatile float q0 = 1.0f;
static volatile float q1 = 0.0f;
static volatile float q2 = 0.0f;
static volatile float q3 = 0.0f;
static volatile uint32_t last_update, now_update;
static volatile float exInt, eyInt, ezInt;
static volatile float gx, gy, gz, ax, ay, az, mx, my, mz;
//imu计算返回的yaw相对位置
float imu_yaw = 0;
int16_t temp_inu_yaw = 0;
uint32_t imu_init_tick;
static float imu_yaw_init;
static float imu_yaw_offset;


/**
* @brief     Fast inverse square-root, to calculate 1/Sqrt(x)
* @param[in] input:x
* @retval    1/Sqrt(x)
*/
float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//#define BOARD_DOWN 1   //

static void init_quaternion(void)
{
	int16_t hx, hy;
	float temp;

	hx = imu.mx;
	hy = imu.my;

	if (hy != 0)
		temp = hx / hy;
	else
		return;

#ifdef BOARD_DOWN
	if (hx<0 && hy <0)   //OK
	{
		if (fabs(temp) >= 1)
		{
			q0 = -0.005f;
			q1 = -0.199f;
			q2 = 0.979f;
			q3 = -0.0089f;
		}
		else
		{
			q0 = -0.008f;
			q1 = -0.555f;
			q2 = 0.83f;
			q3 = -0.002f;
		}

	}
	else if (hx<0 && hy > 0) //OK
	{
		if (fabs(temp) >= 1)
		{
			q0 = 0.005f;
			q1 = -0.199f;
			q2 = -0.978f;
			q3 = 0.012f;
		}
		else
		{
			q0 = 0.005f;
			q1 = -0.553f;
			q2 = -0.83f;
			q3 = -0.0023f;
		}

	}
	else if (hx > 0 && hy > 0)   //OK
	{
		if (fabs(temp) >= 1)
		{
			q0 = 0.0012f;
			q1 = -0.978f;
			q2 = -0.199f;
			q3 = -0.005f;
		}
		else
		{
			q0 = 0.0023f;
			q1 = -0.83f;
			q2 = -0.553f;
			q3 = 0.0023f;
		}

	}
	else if (hx > 0 && hy < 0)     //OK
	{
		if (fabs(temp) >= 1)
		{
			q0 = 0.0025f;
			q1 = 0.978f;
			q2 = -0.199f;
			q3 = 0.008f;
		}
		else
		{
			q0 = 0.0025f;
			q1 = 0.83f;
			q2 = -0.56f;
			q3 = 0.0045f;
		}
	}
#else
	if (hx<0 && hy <0)
	{
		if (fabs(temp) >= 1)
		{
			q0 = 0.195f;
			q1 = -0.015f;
			q2 = 0.0043f;
			q3 = 0.979f;
		}
		else
		{
			q0 = 0.555f;
			q1 = -0.015f;
			q2 = 0.006f;
			q3 = 0.829f;
		}

	}
	else if (hx<0 && hy > 0)
	{
		if (fabs(temp) >= 1)
		{
			q0 = -0.193f;
			q1 = -0.009f;
			q2 = -0.006f;
			q3 = 0.979f;
		}
		else
		{
			q0 = -0.552f;
			q1 = -0.0048f;
			q2 = -0.0115f;
			q3 = 0.8313f;
		}

	}
	else if (hx>0 && hy > 0)
	{
		if (fabs(temp) >= 1)
		{
			q0 = -0.9785f;
			q1 = 0.008f;
			q2 = -0.02f;
			q3 = 0.195f;
		}
		else
		{
			q0 = -0.9828f;
			q1 = 0.002f;
			q2 = -0.0167f;
			q3 = 0.5557f;
		}

	}
	else if (hx > 0 && hy < 0)
	{
		if (fabs(temp) >= 1)
		{
			q0 = -0.979f;
			q1 = 0.0116f;
			q2 = -0.0167f;
			q3 = -0.195f;
		}
		else
		{
			q0 = -0.83f;
			q1 = 0.014f;
			q2 = -0.012f;
			q3 = -0.556f;
		}
	}
#endif

}

//**********************************************************************
//* 功能： 初始化开机imu的yaw计算值                                    *
//* 参数：无                                                           *
//* 返回值：无                                                         *
//**********************************************************************
void imu_param_init(void)
{
	init_quaternion();
	imu_yaw = 0;
	imu_yaw_init = 0;
}
//**********************************************************************
//* 功能： imu任务处理                                                 *
//* 参数：无                                                           *
//* 返回值：无                                                         *
//**********************************************************************
void imu_task()
{
	IMU_Get_Data();                        	                         //获取数据
	if(imu_init_tick < IMU_INIT_TIME)                                //花2秒计算出yaw轴的温漂
	{
		imu_yaw_init = imu_yaw_init - imu_data.gz * 0.061037019f * 0.001f;
		imu_init_tick++;
	}
	if(imu_init_tick == IMU_INIT_TIME)                               //计算出平均每毫秒的温漂
	{
		imu_yaw_offset = imu_yaw_init / IMU_INIT_TIME;
		//imu_yaw_offset = 0;测试温漂抑制效果
		imu_init_tick++;
	}
	if(Last_Mode.Gimbal == Gimbal_Lock_Mode && (Mode.Gimbal == Gimbal_Follow_Mode || Mode.Gimbal == Gimbal_Auto_Mode))
	{
		imu_yaw = current_yaw;                                     //模式切换积分清零
	}
	else
	{
		imu_yaw = imu_yaw - imu_data.gz * 0.061037019f * 0.001f - imu_yaw_offset;        //每次计算减去计算所得温漂
	}
}



