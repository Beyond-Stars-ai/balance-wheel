#ifndef __PID__H
#define __PID__H

#include "main.h"

int Balance_PD(float Angle,float Gyro);
int Velocity_PI(int encoder_left,int encoder_right);
int Turn_PD(float gyro);

// 下面的pid为了好调,都放到100倍了

// 直立环PD控制参数
#define Balance_Kp  10200 // 范围0-288 Range 0-288
#define Balance_Kd  78     // 范围0-2 Range 0-2

// 速度环PI控制参数
#define Velocity_Kp  7000 // 范围0-72 Range 0-72 6000
#define Velocity_Ki  35    // kp/200

// 转向环PD控制参数
#define Turn_Kp  1400 // 这个根据自己的需求调，只是平衡可以不调 This can be adjusted according to one's own needs, but the balance can be left unadjusted, depending on the rotation speed
#define Turn_Kd  10   // 范围 0-2 Range 0-2

#endif