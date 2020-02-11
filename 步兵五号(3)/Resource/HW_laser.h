#ifndef _LASER_H_
#define _LASER_H_

#include "gpio.h"
#define LASER_GPIO_PORT GPIOA
#define LASER_PIN       GPIO_PIN_0

void Laser_Init(void);
void Laser_Turn_ON(void);
void Laser_Turn_OFF(void);
#endif
