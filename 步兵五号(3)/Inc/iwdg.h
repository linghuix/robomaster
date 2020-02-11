#ifndef _IWDG_H
#define _IWDG_H

#include "stm32f4xx_hal.h"
#include "main.h"
#include "stm32f4xx_hal_iwdg.h"



void IWDG_Init(uint8_t prer,uint16_t rlr);
void IWDG_Feed(void) ;

#endif
