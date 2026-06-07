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
#define LED4_Pin GPIO_PIN_6
#define LED4_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_7
#define LED3_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_0
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOB
#define BTN2_Pin GPIO_PIN_13
#define BTN2_GPIO_Port GPIOB
#define BTN1_Pin GPIO_PIN_14
#define BTN1_GPIO_Port GPIOB
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
