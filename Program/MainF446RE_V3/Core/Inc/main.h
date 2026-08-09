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
#define FRONT_LED_Pin GPIO_PIN_6
#define FRONT_LED_GPIO_Port GPIOA
#define LW_LED_Pin GPIO_PIN_7
#define LW_LED_GPIO_Port GPIOA
#define LIDAR_POWER_Pin GPIO_PIN_4
#define LIDAR_POWER_GPIO_Port GPIOC
#define INT_Pin GPIO_PIN_5
#define INT_GPIO_Port GPIOC
#define INT_EXTI_IRQn EXTI9_5_IRQn
#define REAR_LED_Pin GPIO_PIN_0
#define REAR_LED_GPIO_Port GPIOB
#define RW_LED_Pin GPIO_PIN_1
#define RW_LED_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_10
#define BUZZER_GPIO_Port GPIOB
#define RAS_SIG_Pin GPIO_PIN_12
#define RAS_SIG_GPIO_Port GPIOB
#define BUTTON2_Pin GPIO_PIN_13
#define BUTTON2_GPIO_Port GPIOB
#define BUTTON1_Pin GPIO_PIN_14
#define BUTTON1_GPIO_Port GPIOB
#define DRIVE_POWER_Pin GPIO_PIN_15
#define DRIVE_POWER_GPIO_Port GPIOB
#define TRIG_REAR_Pin GPIO_PIN_8
#define TRIG_REAR_GPIO_Port GPIOC
#define ECHO_REAR_Pin GPIO_PIN_9
#define ECHO_REAR_GPIO_Port GPIOC
#define ECHO_REAR_EXTI_IRQn EXTI9_5_IRQn
#define LIDAR_OUT_Pin GPIO_PIN_8
#define LIDAR_OUT_GPIO_Port GPIOA
#define TRIG_FRONT_Pin GPIO_PIN_11
#define TRIG_FRONT_GPIO_Port GPIOA
#define ECHO_FRONT_Pin GPIO_PIN_12
#define ECHO_FRONT_GPIO_Port GPIOA
#define ECHO_FRONT_EXTI_IRQn EXTI15_10_IRQn
#define LED1_Pin GPIO_PIN_4
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_5
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_6
#define LED3_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_7
#define LED4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
