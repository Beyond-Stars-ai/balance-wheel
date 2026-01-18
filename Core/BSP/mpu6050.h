#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"

#define MPU6050_ADDR 0xD0
#define PWR_MGMT_1 0x6B
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define INT_ENABLE 0x38
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43

void MPU6050_Init(I2C_HandleTypeDef *hi2c);
void MPU6050_ReadAccel(I2C_HandleTypeDef *hi2c, int16_t *ax, int16_t *ay, int16_t *az);
void MPU6050_ReadGyro(I2C_HandleTypeDef *hi2c, int16_t *gx, int16_t *gy, int16_t *gz);

#endif