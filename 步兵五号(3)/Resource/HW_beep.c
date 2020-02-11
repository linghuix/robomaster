/*************************************
* @file       HW_beep.c
* @author     只有pwm初始化是wwy写的
* @version    season_2018
* @date       2019.1.29
* @brief      发出歌声~
**************************************
* @attention  
使用定时器4通道1，IO口为PD12，这些初始化在tim.c里完成，PWM的初始化在Beep_Init()完成
**************************************
*/
/* Include -------------------------*/
#include "HW_beep.h"
#include "control_task.h"
/* Global Variable -----------------------------*/






uint32_t Startup_Success_music_index = 0;
uint32_t Chassis_music_index = 0;
const uint16_t tone_tab[] = 
{
  3822,  3405, 3033, 2863, 2551, 2272, 2024,	//bass 1~7
  1911,  1702, 1526, 1431, 1275, 1136, 1012,	//mid 1~7
  955,   851,  758,  715,   637, 568,   506,	//treble 1~7
};

const Sound_tone_e Mavic_Startup_music[Startup_Success_music_len] = 
{
	Do1M,  Do1M,  Silent,  Do1M,  Do1M,
	Silent,Mi3M,  Mi3M,    Silent,Silent,
	Do1M,  Silent,Do1M,    Do1M,  Silent,
	Mi3M,  Mi3M,  Mi3M,    Silent,Silent,
	La6M,  La6M,  Silent,  La6M,  La6M,
	Silent,La6M,  So5M,    Silent,La6M,
	La6M,  Silent,So5M,    Silent,Do1M,
	Do1M,  Silent,Mi3M,    Mi3M,  Silent //最后一个音是silent，所以唱完后会停止蜂鸣器
};

Sound_tone_e chassis_check_music[Chassis_music_len] = 
{
	Do1M,  Silent,
	Do1M,  Silent,
	Do1M,  Silent,
	Do1M,  Silent,
	Silent,Silent
};
/*Do*/

/**
* @brief  蜂鸣器pwm初始化
* @param  none
* @retval none
* @note
**/
void Beep_Init(void)
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
}

void Sing(Sound_tone_e tone)
{
  if(Silent == tone)
    BEEP_CH = 0;
  else 
  {
    BEEP_ARR = tone_tab[tone];
    BEEP_CH = tone_tab[tone] / 2;
  }
}

void Sing_Startup_music(uint32_t index)
{
  if(index < Startup_Success_music_len)
    Sing(Mavic_Startup_music[index]);
}
void Sing_Chassis_music(uint32_t index)
{
	if(index < Chassis_music_len)
		Sing(chassis_check_music[index]);
}



void chassis_beep(uint8_t LFw,uint8_t RFw,uint8_t LBw,uint8_t RBw,uint8_t LFc,uint8_t RFc,uint8_t LBc,uint8_t RBc)
{
	chassis_check_music[0] = ((LFw==1)||(LFc==1))?Do1H:Do1M;
	chassis_check_music[1] = (LFc==1)?Re2H:Silent;
	
	chassis_check_music[2] = ((RFw==1)||(RFc==1))?Do1H:Do1M;
	chassis_check_music[3] = (RFc==1)?Re2H:Silent;
	
	chassis_check_music[4] = ((LBw==1)||(LBc==1))?Do1H:Do1M;
	chassis_check_music[5] = (LBc==1)?Re2H:Silent;

	chassis_check_music[6] = ((RBw==1)||(RBc==1))?Do1H:Do1M;
	chassis_check_music[7] = (RBc==1)?Re2H:Silent;
	
	if(time_1ms % 80==0)
	{
		if(Chassis_music_index < Chassis_music_len)
		{
			Sing_Chassis_music(Chassis_music_index);
		  	Chassis_music_index++;
		}
		else
		{
			Chassis_music_index=0;
		}
	}
}
