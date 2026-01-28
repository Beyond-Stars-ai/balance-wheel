/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "encoder.h"
#include "motor.h"
// #include "mpu6050.h"
// #include "WT61C.h"
#include "oled.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t receiveData[32];

int16_t Encoder_Left = 0, Encoder_Right = 0;              // 速度编码
float Angle_Balance = 0, Gyro_Balance = 0, Gyro_Turn = 0; // mpu6050参数

// MPU6050_t MPU6050;
// uint8_t RxBuff[11]; // 接收缓冲区
uint8_t RxBuff[33]; // 接收缓冲区

float Angle_X, Angle_Y, Angle_Z;             // 角度数据
float Velocity_Angle_X, Velocity_Angle_Y;    // 角速度数据
float Acc_Angle_X, Acc_Angle_Y, Acc_Angle_Z; // 加速度
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for DebugTask */
osThreadId_t DebugTaskHandle;
const osThreadAttr_t DebugTask_attributes = {
    .name = "DebugTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for OLEDTask */
osThreadId_t OLEDTaskHandle;
const osThreadAttr_t OLEDTask_attributes = {
    .name = "OLEDTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
    .name = "MotorTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDebugTask(void *argument);
void StartOLEDTask(void *argument);
void StartMotorTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of DebugTask */
    DebugTaskHandle = osThreadNew(StartDebugTask, NULL, &DebugTask_attributes);

    /* creation of OLEDTask */
    OLEDTaskHandle = osThreadNew(StartOLEDTask, NULL, &OLEDTask_attributes);

    /* creation of MotorTask */
    MotorTaskHandle = osThreadNew(StartMotorTask, NULL, &MotorTask_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDebugTask */
/**
 * @brief  Function implementing the DebugTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDebugTask */
void StartDebugTask(void *argument)
{
    /* USER CODE BEGIN StartDebugTask */

    // 启动UART2的DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxBuff, sizeof(RxBuff));

    // 启动TIM8基础定时器
    HAL_TIM_Base_Start(&htim8);

    // 启动TIM8的四个PWM通道
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

    // 初始化PWM
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0); // Motor1 正转 PWM
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0); // Motor1 反转 PWM
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0); // Motor2 正转 PWM
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0); // Motor2 反转 PWM

    // 启动编码定时器
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    /* Infinite loop */
    for (;;)
    {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        sprintf((char *)receiveData,
                "Lv:%d\t Rv:%d\r\n", Encoder_Left, Encoder_Right);
        HAL_UART_Transmit(&huart1, receiveData, sizeof(receiveData), 100);
        memset(receiveData, 0, sizeof(receiveData));
        // HAL_GPIO_TogglePin(Beep_GPIO_Port, Beep_Pin);
        osDelay(1000);
    }
    /* USER CODE END StartDebugTask */
}

/* USER CODE BEGIN Header_StartOLEDTask */
/**
 * @brief Function implementing the OLEDTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartOLEDTask */
void StartOLEDTask(void *argument)
{
    /* USER CODE BEGIN StartOLEDTask */
    /* Infinite loop */
    for (;;)
    {
        // MPU6050_Read_All(&hi2c2, &MPU6050);
        // Angle_Balance = MPU6050.KalmanAngleY;
        // Gyro_Balance = MPU6050.Gy;
        // Gyro_Turn = MPU6050.Gz;

        OLED_NewFrame();
        char strBuffer[32];
        sprintf(strBuffer, "X:%d", (int)(Acc_Angle_X * 100));
        OLED_PrintString(0, 0, strBuffer, &font16x16, OLED_COLOR_NORMAL);
        sprintf(strBuffer, "Y:%d", (int)(Acc_Angle_Y * 100));
        OLED_PrintString(0, 16, strBuffer, &font16x16, OLED_COLOR_NORMAL);
        sprintf(strBuffer, "Z:%d", (int)(Acc_Angle_Z * 100));
        OLED_PrintString(0, 32, strBuffer, &font16x16, OLED_COLOR_NORMAL);

        // sprintf(strBuffer, "L:%d",Acc_Angle_Y);
        // OLED_PrintString(0, 0, strBuffer, &font16x16, OLED_COLOR_NORMAL);
        // sprintf(strBuffer, "R:%d", Acc_Angle_Y);
        // OLED_PrintString(0, 16, strBuffer, &font16x16, OLED_COLOR_NORMAL);

        OLED_ShowFrame();
        osDelay(100);
    }
    /* USER CODE END StartOLEDTask */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
 * @brief Function implementing the MotorTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void *argument)
{
    /* USER CODE BEGIN StartMotorTask */
    int motor_L, motor_R;

    int Balance_Pwm, Velocity_Pwm, Turn_Pwm;
    /* Infinite loop */
    for (;;)
    {

        Encoder_Left = Read_Encoder(MOTOR_ID_ML);
        Encoder_Right = -Read_Encoder(MOTOR_ID_MR);

        // Position_L +=Encoder_Left;
        // Position_R +=Encoder_Right;

        Balance_Pwm = Balance_PD(Angle_Balance, Gyro_Balance);   // 平衡PID控制
        Velocity_Pwm = Velocity_PI(Encoder_Left, Encoder_Right); // 速度环PID控制
        Turn_Pwm = Turn_PD(Gyro_Turn);                           // 转向环PID控制

        motor_L = Balance_Pwm + Velocity_Pwm + Turn_Pwm; // 计算左轮电机最终PWM Calculate the final PWM of the left wheel motor
        motor_R = Balance_Pwm + Velocity_Pwm - Turn_Pwm; // 计算右轮电机最终PWM Calculate the final PWM of the right wheel motor

        // Motor_SetPWM(motor_L,motor_R);
        osDelay(10);
    }
    /* USER CODE END StartMotorTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
// int _write(int fd, char *ptr, int len)
// {
//     HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
//     return len;
// }
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == KEY_Pin)
    {
        osDelay(10);
        if (GPIO_Pin == KEY_Pin)
        {
            HAL_GPIO_TogglePin(Beep_GPIO_Port, Beep_Pin);
        }
    }
}

// void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
// {
//     if (huart->Instance == USART2)
//     {
//         // 检查接收到的数据长度是否正确
//         if (RxBuff[0] == 0x55 && Size == 33)
//         {
//             switch (RxBuff[1])
//             {
//             case 0x53:
//                 Angle_X = ((short)(RxBuff[3] << 8 | RxBuff[2])) / 32768.0 * 180;
//                 Angle_Y = ((short)(RxBuff[5] << 8 | RxBuff[4])) / 32768.0 * 180;
//                 Angle_Z = ((short)(RxBuff[7] << 8 | RxBuff[6])) / 32768.0 * 180;
//                 break;
//             case 0x52:
//                 Velocity_Angle_X = ((short)(RxBuff[3] << 8 | RxBuff[2])) / 32768.0 * 2000;
//                 Velocity_Angle_Y = ((short)(RxBuff[5] << 8 | RxBuff[4])) / 32768.0 * 2000;
//                 break;
//             case 0x51:
//                 Acc_Angle_X = ((short)(RxBuff[3] << 8 | RxBuff[2])) / 32768.0 * 16 * 9.8;
//                 Acc_Angle_Y = ((short)(RxBuff[5] << 8 | RxBuff[4])) / 32768.0 * 16 * 9.8;
//                 Acc_Angle_Z = ((short)(RxBuff[7] << 8 | RxBuff[6])) / 32768.0 * 16 * 9.8;
//                 break;
//             default:
//                 break;
//             }
//         }

//         // 清空接收缓冲区
//         memset(RxBuff, 0, sizeof(RxBuff));

//         // 清除 IDLE 中断标志
//         __HAL_UART_CLEAR_IDLEFLAG(huart);

//         // 重新启动DMA接收
//         HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxBuff, sizeof(RxBuff));

//         // 禁止半传送中断
//         __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
//     }
// }

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2) {

    uint16_t i = 0;
    
    // 在接收到的数据中找所有完整的 WT61C 数据包
    while (i + 11 <= Size) {
        // 找包头 0x55
        if (RxBuff[i] == 0x55) {
            // 校验和检查
            uint8_t sum = 0;
            for (int j = 0; j < 10; j++) sum += RxBuff[i + j];
            
            if (sum == RxBuff[i + 10]) {
                // 校验通过，解析这 11 字节
                int16_t temp;
                uint8_t type = RxBuff[i + 1];
                
                switch (type) {
                    case 0x51: // 加速度包
                        temp = (RxBuff[i+3] << 8) | RxBuff[i+2];
                        Acc_Angle_X = temp / 32768.0f * 16.0f * 9.8f;
                        temp = (RxBuff[i+5] << 8) | RxBuff[i+4];
                        Acc_Angle_Y = temp / 32768.0f * 16.0f * 9.8f;
                        temp = (RxBuff[i+7] << 8) | RxBuff[i+6];
                        Acc_Angle_Z = temp / 32768.0f * 16.0f * 9.8f;
                        break;
                        
                    case 0x52: // 角速度包
                        temp = (RxBuff[i+3] << 8) | RxBuff[i+2];
                        Velocity_Angle_X = temp / 32768.0f * 2000.0f;
                        temp = (RxBuff[i+5] << 8) | RxBuff[i+4];
                        Velocity_Angle_Y = temp / 32768.0f * 2000.0f;
                        break;
                        
                    case 0x53: // 角度包
                        temp = (RxBuff[i+3] << 8) | RxBuff[i+2];
                        Angle_X = temp / 32768.0f * 180.0f;
                        temp = (RxBuff[i+5] << 8) | RxBuff[i+4];
                        Angle_Y = temp / 32768.0f * 180.0f;
                        temp = (RxBuff[i+7] << 8) | RxBuff[i+6];
                        Angle_Z = temp / 32768.0f * 180.0f;
                        break;
                }
                i += 11;  // 跳到下一个包
                continue;
            }
        }
        i++;  // 不是包头或校验失败，滑动一个字节
    }
  }

    // 清除 IDLE 标志并重启 DMA
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxBuff, sizeof(RxBuff));
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}
/* USER CODE END Application */
