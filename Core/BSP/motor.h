#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include <stdint.h>

#define MOTOR_MAX_PWM 100

#define PI 3.14159265							//PI圆周率  PI π
#define Control_Frequency  200.0	//编码器读取频率  Encoder reading frequency
#define Diameter  65.0 				//轮子直径65mm   Wheel diameter 65mm
#define EncoderMultiples   4.0 		//编码器倍频数  Encoder multiples
#define Encoder_precision  11.0 	//编码器精度 11线  Encoder precision 11 lines
#define Reduction_Ratio  30.0			//减速比30  Reduction ratio 30
#define Perimeter  204.2035 			//周长，单位mm Perimeter, unit mm

// #define Position_KP (2.2)
// #define Position_KI (0.013)
// #define Position_KD (0.05)

#define Position_KP (0.8)
#define Position_KI (0.001)
#define Position_KD (0.5)

#define Incremental_KP (20)
#define Incremental_KI (8)
#define Incremental_KD (2)

// #define Incremental_KP (20)
// #define Incremental_KI (1)
// #define Incremental_KD (2)

#define Target_Velocity (10)

void Motor_SetPWM(int16_t left, int16_t right);

int Position_PID (int position,int target);

int Incremental_PI (int Encoder,int Target);

#endif