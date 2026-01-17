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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t receiveData[64];

int16_t Position_L=0,Position_R=0;          //位置编码
int16_t Encoder_Left =0 ,Encoder_Right=0;   //速度编码

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

  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
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
  /* Infinite loop */
  for(;;)
  {
    sprintf((char *)receiveData,
    "Lv:%d\t Rv:%d\r\nLp:%d\t Rp:%d\t\r\n", Encoder_Left, Encoder_Right, Position_L, Position_R);       					
    HAL_UART_Transmit(&huart1, receiveData, sizeof(receiveData), 100);
    osDelay(100);
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
  int Position_Motor_L,Position_Motor_R;
 
  int16_t Target_Position = 0;
  /* Infinite loop */
  for(;;)
  {    
    
		Encoder_Left = Read_Encoder(MOTOR_ID_ML);          					
		Encoder_Right = -Read_Encoder(MOTOR_ID_MR);   
		
		Position_L +=Encoder_Left;
		Position_R +=Encoder_Right;
		
		Position_Motor_L = Position_PID(Position_L,Target_Position);
		Position_Motor_R = Position_PID(Position_R,Target_Position);
		
		motor_L = Incremental_PI(Encoder_Left,Position_Motor_L);
		motor_R = Incremental_PI(Encoder_Right,Position_Motor_R);

    Motor_SetPWM(motor_L,motor_R);
    osDelay(10);

  }
  /* USER CODE END StartMotorTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
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

// //不定长定时DMA中断接收
// void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
// {
//   HAL_UART_Transmit_DMA(&huart1, receiveData, sizeof(receiveData));
//   osDelay(1);
//   HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receiveData, sizeof(receiveData));
// }

// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// {
//   if(htim->Instance == TIM3)
//   {
//     // 处理TIM3溢出
//   }
//   else if(htim->Instance == TIM4)
//   {
//     // 处理TIM4溢出
//   }
// }


