#ifndef _BEEP_H_
#define _BEEP_H_

#include "stm32f4xx.h"
#include "stdint.h"
#include "tim.h"

#define Startup_Success_music_len 40
#define Chassis_music_len 10

#define BEEP_ON     (TIM4->CRR1 = 599)
#define BEEP_OFF    (TIM4->CCR1 = 0)

#define BEEP_ARR    (TIM4->ARR)
#define BEEP_CH     (TIM4->CCR1)


typedef enum{

  Do1L = 0, ///*261.63Hz*/    3822us
  Re2L,     ///*293.66Hz*/    3405us
  Mi3L,     ///*329.63Hz*/    3034us
  Fa4L,     ///*349.23Hz*/    2863us
  So5L,     ///*392.00Hz*/    2551us
  La6L,     ///*440.00Hz*/    2272us
  Si7L,     ///*493.88Hz*/    2052us

  Do1M,     ///*523.25Hz*/    1911us
  Re2M,     ///*587.33Hz*/    1703us
  Mi3M,     ///*659.26Hz*/    1517us
  Fa4M,     ///*698.46Hz*/    1432us
  So5M,     ///*784.00Hz*/    1276us
  La6M,     ///*880.00Hz*/    1136us
  Si7M,     ///*987.77Hz*/    1012us

  Do1H,     ///*1046.50Hz*/   956us
  Re2H,     ///*1174.66Hz*/   851us
  Mi3H,     ///*1318.51Hz*/   758us
  Fa4H,     ///*1396.91Hz*/   716us
  So5H,     ///*1567.98Hz*/   638us
  La6H,     ///*1760.00Hz*/   568us
  Si7H,     ///*1975.53Hz*/   506us

  Silent,
}Sound_tone_e;

extern uint32_t Startup_Success_music_index;
void Beep_Init(void);
void Sing(Sound_tone_e tone);
void Sing_Chassis_music(uint32_t index);
void Sing_Startup_music(uint32_t index);
void chassis_beep(uint8_t LFw,uint8_t RFw,uint8_t LBw,uint8_t RBw,uint8_t LFc,uint8_t RFc,uint8_t LBc,uint8_t RBc);
#endif
