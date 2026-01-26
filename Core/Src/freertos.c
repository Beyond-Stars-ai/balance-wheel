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
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "oled.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t receiveData[128];

int16_t Encoder_Left =0 ,Encoder_Right=0;   //速度编码
float Angle_Balance =0 ,Gyro_Balance =0 ,Gyro_Turn =0; //mpu6050参数

// MPU6050_t MPU6050; 

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
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BTTask */
osThreadId_t BTTaskHandle;
const osThreadAttr_t BTTask_attributes = {
  .name = "BTTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartBTTask(void *argument);
void StartMotorTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
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
  DebugTaskHandle = osThreadNew(StartDefaultTask, NULL, &DebugTask_attributes);

  /* creation of BTTask */
  BTTaskHandle = osThreadNew(StartBTTask, NULL, &BTTask_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(StartMotorTask, NULL, &MotorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the DebugTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  // // 启动UART1的DMA接收
  // HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receiveData, sizeof(receiveData));

  // 启动TIM8基础定时器
  HAL_TIM_Base_Start(&htim8);

  // 启动TIM8的四个PWM通道
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

  // 初始化PWM
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);  // Motor1 正转 PWM
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);  // Motor1 反转 PWM
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);  // Motor2 正转 PWM
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);  // Motor2 反转 PWM

  // 启动编码定时器 
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  // 初始化MPU6050
  // MPU6050_Init(&hi2c2);

  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    sprintf((char *)receiveData,
    "Lv:%d\t Rv:%d\r\n", Encoder_Left, Encoder_Right);       					
    HAL_UART_Transmit(&huart1, receiveData, sizeof(receiveData), 100);
    memset(receiveData, 0, sizeof(receiveData));
    // HAL_GPIO_TogglePin(Beep_GPIO_Port, Beep_Pin);
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartBTTask */
/**
* @brief Function implementing the BTTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBTTask */
void StartBTTask(void *argument)
{
  /* USER CODE BEGIN StartBTTask */
  // int n = 0;
  /* Infinite loop */
  
  
  for(;;)
  {
    // MPU6050_Read_All(&hi2c2, &MPU6050);
    // Angle_Balance = MPU6050.KalmanAngleY;
    // Gyro_Balance = MPU6050.Gy;
    // Gyro_Turn = MPU6050.Gz;

    // n++;

    // if(n>=50)
    // {
    //   printf("Ax:%dg\t Ay:%dg\t Az:%dg\r\nGx:%d°/s\t Gy:%d°/s\t Gz:%d°/s\r\n",
    //   (int)MPU6050.Ax, (int)MPU6050.Ay, (int)MPU6050.Az, (int)MPU6050.Gx, (int)MPU6050.Gy, (int)MPU6050.Gz);
    //   n=0;
    //   // printf("Ax:%.2fg\t Ay:%.2fg\t Az:%.2fg\r\nGx:%.2f°/s\t Gy:%.2f°/s\t Gz:%.2f°/s\r\n",
    //   // MPU6050.Ax, MPU6050.Ay, MPU6050.Az, MPU6050.Gx, MPU6050.Gy, MPU6050.Gz);
    //   // printf("Angle:%.2f°\t Gyro.Gy:%.2f°/s \t Gyro.Gz:%.2f°/s \t temperature:%.2f°C\r\n", Angle_Balance, Gyro_Balance, Gyro_Turn, MPU6050.Temperature);
    // }
    for (uint8_t i = 0; i < 256; i++)
    {
      OLED_NewFrame();
      OLED_DrawImage((128 - (bilibiliImg.w)) / 2, 0, &bilibiliImg, OLED_COLOR_NORMAL);
      OLED_PrintString(128 - i, 64 - 16, "波特律动hello", &font16x16, OLED_COLOR_NORMAL);
      OLED_ShowFrame();
      osDelay(10);
    }
    osDelay(1);

  }
  /* USER CODE END StartBTTask */
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
  int motor_L,motor_R;

  int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
  /* Infinite loop */
  for(;;)
  {    
    
		Encoder_Left = Read_Encoder(MOTOR_ID_ML);          					
		Encoder_Right = -Read_Encoder(MOTOR_ID_MR);   
		
		// Position_L +=Encoder_Left;
		// Position_R +=Encoder_Right;

    Balance_Pwm = Balance_PD(Angle_Balance,Gyro_Balance);    //平衡PID控制 
		Velocity_Pwm = Velocity_PI(Encoder_Left,Encoder_Right);  //速度环PID控制		
		Turn_Pwm = Turn_PD(Gyro_Turn);														//转向环PID控制  

    motor_L = Balance_Pwm + Velocity_Pwm + Turn_Pwm;       //计算左轮电机最终PWM Calculate the final PWM of the left wheel motor
		motor_R = Balance_Pwm + Velocity_Pwm - Turn_Pwm;      //计算右轮电机最终PWM Calculate the final PWM of the right wheel motor

    // Motor_SetPWM(motor_L,motor_R);
    osDelay(10);

  }
  /* USER CODE END StartMotorTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
int _write(int fd, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}
/* USER CODE END Application */

