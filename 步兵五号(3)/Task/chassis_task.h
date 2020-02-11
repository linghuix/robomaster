#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#include <math.h>
#include "stdint.h"
#include "string.h"
#include "gimbal_task.h"
#include "mode_update.h"
#include "filter.h"
#include "HW_dbus.h"
#include "HW_can.h"
#include "imu_task.h"

#ifndef PI
#define PI 3.1415926f
#endif
#define FL 0
#define FR 1
#define BR 2
#define BL 3

#define WS 0
#define AD 1
#define QE 2

#define VAL_LIMIT(val, min, max) \
do {\
if((val) <= (min))\
{\
  (val) = (min);\
}\
else if((val) >= (max))\
{\
  (val) = (max);\
}\
} while(0)\



typedef struct
{
	float           vx; // forward/back
	float           vy; // left/right
	float           vw;
	float           ax;
	float           ay;
	float           aw;
	int16_t         rotate_x_offset;
	int16_t         rotate_y_offset;

	float           gyro_angle;
	float           gyro_palstance;

	int16_t         wheel_spd_fdb[4];
	int16_t         wheel_spd_ref[4];
	int16_t         current[4];

	int16_t         position_ref;
	uint8_t         follow_gimbal;
} chassis_t;


void chassis_task(void);
void chassis_inspection_mode(void);
void chassis_start_mode(void);
void chassis_twist_mode(void);
void chassis_depart_mode(void);
void chassis_follow_mode(void);
void chassis_hand_start_mode(void);
void chassis_check_mode(void);
// chassis_parameter_init(void);
void chassis_set_speed(float angle,float ratio);
void mecanum_calc(float vx, float vy, float vw, int16_t speed[]);
void chassis_action(void);
float set_chassis_vw(float k_vw);
void power_limit(void);
void chassis_check(int16_t LFcurrent,int16_t RFcurrent,int16_t LBcurrent,int16_t RBcurrent);

#endif // !_CHASSIS_H_
