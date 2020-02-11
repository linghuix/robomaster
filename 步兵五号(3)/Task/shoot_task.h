#ifndef _SHOOT_TASK_H_
#define _SHOOT_TASK_H_

#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "string.h"
#include "tim.h"
#include "HW_can.h"
#include "pid.h"
#include "mode_update.h"
#include "judge_task.h"
#include "HW_nuc.h"

//宏定义给英雄用，步兵需要修改
#define SINGLE_BULLET_HEAT  100     //单发射击热量
#define ONE_SHOOT_ANGLE     45      //一次射击拨弹轮旋转的角度

#define FEED  0
#define FRIC1 1
#define FRIC2 2
#define FRIC3 3

typedef enum
{
    Single_Shoot,                  //单发
    Continuous_Shoot,              //连续射击
}Trigger_Mode_t;

extern int16_t feed_current;
extern int16_t fric_current[3];

void shoot_task(void);
void shoot_stop(void);
void shoot_signal_process(void);
static void _shoot_normal_handle(void);
static void _shoot_auto_handle(void);
static void fric_wheel_ctrl(void);
#endif
