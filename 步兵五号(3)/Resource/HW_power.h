#ifndef _HW_POWER_
#define _HW_POWER_
#include "stdint.h"
#include "usart.h"
#include "math.h"

extern uint8_t PowerRxBuf[9];
extern uint8_t chassisoutput_ratio;
void POWER_USART_IRQHandler(void);
void ADC_process(void);


#endif
