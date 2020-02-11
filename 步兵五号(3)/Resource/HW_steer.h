#ifndef _HW_STEER_H_
#define _HW_STEER_H_

#include "stdint.h"
#include "math.h"
#include "gpio.h"
#define STEER_ClOSE 17	//弹仓关闭控制指令
#define STEER_OPEN  4	//弹仓开启控制指令
void steer_task(void);
void Set_Steer_Duty(uint8_t duty);
void Set_steer_high(void);
void Set_steer_low(void);
void Steer_Init(void);

#endif
