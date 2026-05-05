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
#define BUTTON2_Pin GPIO_PIN_13
#define BUTTON2_GPIO_Port GPIOC
#define BUTTON1_Pin GPIO_PIN_14
#define BUTTON1_GPIO_Port GPIOC
#define VOLTAGE_P_Pin GPIO_PIN_2
#define VOLTAGE_P_GPIO_Port GPIOC
#define VOLTAGE_S_Pin GPIO_PIN_3
#define VOLTAGE_S_GPIO_Port GPIOC
#define ADC3_Pin GPIO_PIN_4
#define ADC3_GPIO_Port GPIOA
#define ADC2_Pin GPIO_PIN_5
#define ADC2_GPIO_Port GPIOA
#define LED4_Pin GPIO_PIN_6
#define LED4_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_7
#define LED3_GPIO_Port GPIOA
#define ADC1_Pin GPIO_PIN_4
#define ADC1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_0
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOB
#define TRIG5_Pin GPIO_PIN_12
#define TRIG5_GPIO_Port GPIOB
#define ECHO5_Pin GPIO_PIN_13
#define ECHO5_GPIO_Port GPIOB
#define TRIG4_Pin GPIO_PIN_14
#define TRIG4_GPIO_Port GPIOB
#define ECHO4_Pin GPIO_PIN_15
#define ECHO4_GPIO_Port GPIOB
#define TRIG3_Pin GPIO_PIN_8
#define TRIG3_GPIO_Port GPIOC
#define ECHO3_Pin GPIO_PIN_9
#define ECHO3_GPIO_Port GPIOC
#define ECHO2_Pin GPIO_PIN_8
#define ECHO2_GPIO_Port GPIOA
#define TRIG2_Pin GPIO_PIN_11
#define TRIG2_GPIO_Port GPIOA
#define ECHO1_Pin GPIO_PIN_12
#define ECHO1_GPIO_Port GPIOA
#define TRIG1_Pin GPIO_PIN_15
#define TRIG1_GPIO_Port GPIOA
#define LIDAR_Pin GPIO_PIN_10
#define LIDAR_GPIO_Port GPIOC
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
