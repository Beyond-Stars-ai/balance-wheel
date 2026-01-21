#ifndef __PID__H
#define __PID__H

#include "main.h"

int Balance_PD(float Angle,float Gyro);
int Velocity_PI(int encoder_left,int encoder_right);
int Turn_PD(float gyro);

#endif