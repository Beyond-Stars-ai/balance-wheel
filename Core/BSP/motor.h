#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include <stdint.h>

#define MOTOR_MAX_PWM 119

// PWM区间是 0-120

void Motor_SetPWM(int16_t left, int16_t right);

#endif