#ifndef __IMU_TASK_H__
#define __IMU_TASK_H__

#include "stm32f4xx_hal.h"
#include "HW_imu.h"

#define IMU_TASK_PERIOD 1        //imu任务处理周期，单位为毫秒
#define IMU_INIT_TIME 2000	    //计算yaw轴offset所需的时间

typedef struct
{
	//前一次的姿态量
	float last_roll;
	float last_pitch;
	float last_yaw;
    //当前姿态量
	float roll;
	float pitch;
	float yaw;
} imu_attitude_t;


extern imu_data_t     imu;
extern float imu_yaw;
extern float imu_yaw_buf[256];
extern uint8_t imu_yaw_buf_pos;
extern uint32_t imu_init_tick;

void imu_task(void);
void imu_param_init(void);



#endif
