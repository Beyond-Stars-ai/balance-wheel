#include "mpu6050.h"

void MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t data;

    // 唤醒MPU6050
    data = 0;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

    // 设置采样率
    data = 0x07;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, SMPLRT_DIV, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

    // 配置陀螺仪和加速度计
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, CONFIG, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

    // 配置陀螺仪量程
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

    // 配置加速度计量程
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

    // 使能中断
    data = 0x01;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, INT_ENABLE, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

void MPU6050_ReadAccel(I2C_HandleTypeDef *hi2c, int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buffer[6];

    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, buffer, 6, 100);

    *ax = (int16_t)((buffer[0] << 8) | buffer[1]);
    *ay = (int16_t)((buffer[2] << 8) | buffer[3]);
    *az = (int16_t)((buffer[4] << 8) | buffer[5]);
}

void MPU6050_ReadGyro(I2C_HandleTypeDef *hi2c, int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buffer[6];

    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, buffer, 6, 100);

    *gx = (int16_t)((buffer[0] << 8) | buffer[1]);
    *gy = (int16_t)((buffer[2] << 8) | buffer[3]);
    *gz = (int16_t)((buffer[4] << 8) | buffer[5]);
}