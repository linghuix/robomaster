#ifndef _CONTROL_TASK_H_
#define _CONTROL_TASK_H_

#include "HW_beep.h"
#include "mode_update.h"
#include "shoot_task.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "judge_task.h"
#include "imu_task.h"
#include "pid.h"

void Control_Task(void);
extern uint32_t time_1ms; 
#endif
