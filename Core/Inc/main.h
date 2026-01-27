/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

// extern DMA_HandleTypeDef hdma_usart1_tx;
// extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Motor1_EncA_Pin GPIO_PIN_6
#define Motor1_EncA_GPIO_Port GPIOA
#define Motor1_EncB_Pin GPIO_PIN_7
#define Motor1_EncB_GPIO_Port GPIOA
#define Motor1_PWM_A_Pin GPIO_PIN_6
#define Motor1_PWM_A_GPIO_Port GPIOC
#define Motor1_PWM_B_Pin GPIO_PIN_7
#define Motor1_PWM_B_GPIO_Port GPIOC
#define Motor2_PWM_A_Pin GPIO_PIN_8
#define Motor2_PWM_A_GPIO_Port GPIOC
#define Motor2_PWM_B_Pin GPIO_PIN_9
#define Motor2_PWM_B_GPIO_Port GPIOC
#define KEY_Pin GPIO_PIN_8
#define KEY_GPIO_Port GPIOA
#define KEY_EXTI_IRQn EXTI9_5_IRQn
#define Beep_Pin GPIO_PIN_11
#define Beep_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_3
#define LED_GPIO_Port GPIOB
#define Motor2_EncA_Pin GPIO_PIN_6
#define Motor2_EncA_GPIO_Port GPIOB
#define Motor2_EncB_Pin GPIO_PIN_7
#define Motor2_EncB_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
