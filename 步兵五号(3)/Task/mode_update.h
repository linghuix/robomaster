#ifndef _MODE_UPDATE_H_
#define _MODE_UPDATE_H_

#include "HW_dbus.h"
#include "usart.h"

//NUC发送数据的模式
typedef enum
{
    Red,
    Blue,
	  Close,
	  CHECK_OPEN,
	  CHECK_CLOSE,
}NUC_Send_Mode;
	
typedef enum
{ 
	  HERO,
    INFANTRY,
    GUARD,
    BASE,
    DEFAULT,  
}NUC_Send_Type;
	  
//云台模式
typedef enum
{
    Gimbal_Lock_Mode,
    Gimbal_Follow_Mode,
    Gimbal_Auto_Mode
}Gimbal_Mode_t;               

//射击模式
typedef enum
{
    Shoot_Normal_Mode,
    Shoot_Auto_Mode,
}Shoot_Mode_t;

//底盘模式
typedef enum
{
	Chassis_Lock_Mode,
	Chassis_Follow_Mode,
	Chassis_Twist_Mode,
	Chassis_Auto_Mode,
	Chassis_Start_Mode,
	Chassis_Hand_Start_Mode,
	Chassis_Inspecion_Mode,
	Chassis_Check_Mode,
}Chassis_Mode_t;

//模式
typedef struct
{
    Gimbal_Mode_t  Gimbal;
    Chassis_Mode_t Chassis;
    Shoot_Mode_t   Shoot;
    uint8_t Shoot_Signal;
}Mode_t;

//模式由遥控器或者键盘设定
typedef enum
{
    Remote_Control,
    Keyboard_Control,
}R_or_K_Control_t;

extern Mode_t Mode;
extern Mode_t Last_Mode;


void RK_Control_Init(void);
void Control_Change(void);
void Remote_Mode_Refresh(void);
void Keyboard_Mode_Refresh(void);
void NUC_Send_ID_Process(void);
void NUC_Send_Mode_Process(NUC_Send_Mode Mode);
void NUC_Send_Type_Process(NUC_Send_Type Mode);


#endif
