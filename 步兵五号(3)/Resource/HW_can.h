#ifndef _HW_CAN_H_
#define _HW_CAN_H_

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "can.h"

/*底盘4个3508回传ID*/
#define CHASSIS1_RxID    0x201
#define CHASSIS2_RxID    0x202
#define CHASSIS3_RxID    0x203
#define CHASSIS4_RxID    0x204

/*飞盘电容主控版*/
#define POWER_ID         0x205

/*云台两个轴回传ID*/
#define GIMBAL_YAWRxID   0x205
#define GIMBAL_PITCHRxID 0x206

/*关于拨弹的几个电机回传ID，步兵去掉第三个摩擦轮*/
#define FEED_RxID        0x201
#define FRIC1_RxID       0x202
#define FRIC2_RxID       0x203 
#define FRIC3_RxID       0x204


#ifndef PI
#define PI 3.1415926f
#endif

typedef struct
{
    float angular_velocity;
	float real_angle;                        //映射到0-360的值
    int16_t rotate_speed;
    int16_t angle_raw_value;            //码盘原始值
    int16_t last_angle_raw_value;
} GMEncoder_t;

typedef struct
{
    int16_t velocity;
    int16_t current;
} CMEncoder_t;

typedef struct
{
    float angular_velocity;
    int16_t rotate_speed;
    int16_t angle_raw_value;
    int16_t last_angle_raw_value;
    float real_angle;
} SHOOTEncoder_t;

extern GMEncoder_t GMYawEncoder;
extern GMEncoder_t GMPitchEncoder;
extern CMEncoder_t CMEncoder[4];
extern SHOOTEncoder_t SHOOTEncoder[4];
extern int32_t gimbal_round,gimbal_round_now;

void CanFilter_Init(CAN_HandleTypeDef* hcan);
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan);
void CAN_Send_Msg(CAN_HandleTypeDef* hcan, uint8_t* msg, uint32_t id, uint8_t len);
void Set_Gimbal_Current(CAN_HandleTypeDef* hcan, int16_t yaw_current, int16_t gimbal_current);
void Set_Chassis_Current(CAN_HandleTypeDef* hcan, int16_t fl_current, int16_t fr_current, int16_t br_current, int16_t bl_current);
void Set_Shoot_Current(CAN_HandleTypeDef* hcan, int16_t feed_current, int16_t* fric_current);

#endif
