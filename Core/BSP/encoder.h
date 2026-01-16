#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

typedef enum {
    MOTOR_ID_ML,
    MOTOR_ID_MR,
    MAX_MOTOR
} Motor_ID;

int Read_Encoder(Motor_ID MYTIMX);

int Read_Position(Motor_ID MYTIMX);

#endif