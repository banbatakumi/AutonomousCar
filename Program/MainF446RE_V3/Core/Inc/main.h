/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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
#define CURRENT_P_Pin GPIO_PIN_0
#define CURRENT_P_GPIO_Port GPIOC
#define CURRENT_S_Pin GPIO_PIN_1
#define CURRENT_S_GPIO_Port GPIOC
#define VOLTAGE_S_Pin GPIO_PIN_2
#define VOLTAGE_S_GPIO_Port GPIOC
#define VOLTAGE_P_Pin GPIO_PIN_3
#define VOLTAGE_P_GPIO_Port GPIOC
#define ENCODER_RIGHT_Pin GPIO_PIN_4
#define ENCODER_RIGHT_GPIO_Port GPIOA
#define ENCODER_LEFT_Pin GPIO_PIN_5
#define ENCODER_LEFT_GPIO_Port GPIOA
#define LED4_Pin GPIO_PIN_6
#define LED4_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_7
#define LED3_GPIO_Port GPIOA
#define LIDAR_5V_SIG_Pin GPIO_PIN_4
#define LIDAR_5V_SIG_GPIO_Port GPIOC
#define INT_Pin GPIO_PIN_5
#define INT_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_0
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_10
#define BUZZER_GPIO_Port GPIOB
#define RAS_SIG_Pin GPIO_PIN_12
#define RAS_SIG_GPIO_Port GPIOB
#define BUTTON2_Pin GPIO_PIN_13
#define BUTTON2_GPIO_Port GPIOB
#define BUTTON1_Pin GPIO_PIN_14
#define BUTTON1_GPIO_Port GPIOB
#define POWER_SIG_Pin GPIO_PIN_15
#define POWER_SIG_GPIO_Port GPIOB
#define TRIG2_Pin GPIO_PIN_8
#define TRIG2_GPIO_Port GPIOC
#define ECHO2_Pin GPIO_PIN_9
#define ECHO2_GPIO_Port GPIOC
#define LIDAR_OUT_Pin GPIO_PIN_8
#define LIDAR_OUT_GPIO_Port GPIOA
#define ECHO2A11_Pin GPIO_PIN_11
#define ECHO2A11_GPIO_Port GPIOA
#define TRIG2A12_Pin GPIO_PIN_12
#define TRIG2A12_GPIO_Port GPIOA
#define USER_LED1_Pin GPIO_PIN_4
#define USER_LED1_GPIO_Port GPIOB
#define USER_LED2_Pin GPIO_PIN_5
#define USER_LED2_GPIO_Port GPIOB
#define USER_LED3_Pin GPIO_PIN_6
#define USER_LED3_GPIO_Port GPIOB
#define USER_LED4_Pin GPIO_PIN_7
#define USER_LED4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
