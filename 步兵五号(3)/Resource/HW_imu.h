#ifndef __BSP_IMU_H__
#define __BSP_IMU_H__

#include "stm32f4xx_hal.h"

#define MPU6500_NSS_Low() HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET)
#define MPU6500_NSS_High() HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET)

typedef struct
{
    int16_t ax;//加速度
    int16_t ay;
    int16_t az;

    int16_t mx;//
    int16_t my;
    int16_t mz;

    int16_t temp;

    int16_t gx;//角速度 degree peer second
    int16_t gy;
    int16_t gz;
    
    int16_t ax_offset;
    int16_t ay_offset;
    int16_t az_offset;
  
    int16_t gx_offset;
    int16_t gy_offset;
    int16_t gz_offset;
    
} mpu_data_t;

typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t mx;
    int16_t my;
    int16_t mz;

    float temp;
    float temp_ref;
  
    float wx;//角速度 rad/s
    float wy;
    float wz;

    float vx;//速度 m/s
    float vy;
    float vz;
  
    float rol;
    float pit;
    float yaw;
} imu_data_t;

extern uint8_t MPU_id;
extern mpu_data_t imu_data;
uint8_t MPU6500_Init(void);
uint8_t MPU6500_Write_Reg(uint8_t const reg, uint8_t const data);
uint8_t MPU6500_Read_Reg(uint8_t const reg);
uint8_t MPU6500_Read_Regs(uint8_t const regAddr, uint8_t *pData, uint8_t len);
void IMU_Get_Data(void);
uint8_t IST8310_Init(void);

#endif
