#ifndef _GIMBAL_TASK_H_
#define _GIMBAL_TASK_H_

#include "stdint.h"
#include "pid.h"
#include "filter.h"
#include "control_task.h"
#include "HW_imu.h"
#include "HW_can.h"
#include "imu_task.h"
#include "HW_nuc.h"
#include "HW_laser.h"

#define PITCH_MAX_ANGLE 25
#define PITCH_MIN_ANGLE -30
#define PITCH_OFFSET_ANGLE 300//130
#define YAW_OFFSET_ANGLE 62
#define YAW_MAX_ANGLE 20
#define YAW_MIN_ANGLE -20
extern float imu_pitch;
extern float imu_yaw;
extern float current_yaw;
extern float current_pitch;


void gimbal_task(void);
void gimbal_lock_mode(void);
void gimbal_follow_mode(void);
void gimbal_auto_mode(void);
void gimbal_auto_action(PID_Regulator_Double_Loop_t *pitch_pid, float pitch_target,float pitch_feedback, PID_Regulator_Double_Loop_t *yaw_pid,float yaw_target,float yaw_feedback);
void gimbal_action(PID_Regulator_Double_Loop_t *pitch_pid, float pitch_target, float pitch_feedback, PID_Regulator_Double_Loop_t *yaw_pid, float yaw_target, float yaw_feedback);
float get_pitch_encoder_angle(void);
float get_yaw_encoder_angle(void);


float set_auto_pitch_angle(void);
float set_auto_yaw_angle(void);
float set_pitch_angle(void);
float set_yaw_angle(float k_mouse,float controller);
float Pitch_Limit(float pitch_current);
float Yaw_Limit(float yaw_current);

#endif // !_GIMBAL_TASH_H_
