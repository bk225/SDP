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
#include "stm32g4xx_hal.h"

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
#define STE_PWM1_Pin GPIO_PIN_0
#define STE_PWM1_GPIO_Port GPIOA
#define STE_PWM2_Pin GPIO_PIN_1
#define STE_PWM2_GPIO_Port GPIOA
#define SER_PWM5_Pin GPIO_PIN_2
#define SER_PWM5_GPIO_Port GPIOA
#define SER_PWM6_Pin GPIO_PIN_3
#define SER_PWM6_GPIO_Port GPIOA
#define ACC_EN1_Pin GPIO_PIN_4
#define ACC_EN1_GPIO_Port GPIOA
#define ACC_EN2_Pin GPIO_PIN_5
#define ACC_EN2_GPIO_Port GPIOA
#define SER_PWM1_Pin GPIO_PIN_6
#define SER_PWM1_GPIO_Port GPIOA
#define SER_PWM2_Pin GPIO_PIN_7
#define SER_PWM2_GPIO_Port GPIOA
#define SER_PWM3_Pin GPIO_PIN_0
#define SER_PWM3_GPIO_Port GPIOB
#define SER_PWM4_Pin GPIO_PIN_1
#define SER_PWM4_GPIO_Port GPIOB
#define PI_RX_Pin GPIO_PIN_10
#define PI_RX_GPIO_Port GPIOB
#define PI_TX_Pin GPIO_PIN_11
#define PI_TX_GPIO_Port GPIOB
#define BRA_EN1_Pin GPIO_PIN_12
#define BRA_EN1_GPIO_Port GPIOB
#define BRA_EN2_Pin GPIO_PIN_13
#define BRA_EN2_GPIO_Port GPIOB
#define STE_EN1_Pin GPIO_PIN_14
#define STE_EN1_GPIO_Port GPIOB
#define STE_EN2_Pin GPIO_PIN_15
#define STE_EN2_GPIO_Port GPIOB
#define ACC_PWM1_Pin GPIO_PIN_8
#define ACC_PWM1_GPIO_Port GPIOA
#define ACC_PWM2_Pin GPIO_PIN_9
#define ACC_PWM2_GPIO_Port GPIOA
#define BRA_PWM1_Pin GPIO_PIN_10
#define BRA_PWM1_GPIO_Port GPIOA
#define BRA_PWM2_Pin GPIO_PIN_11
#define BRA_PWM2_GPIO_Port GPIOA
#define Solenoid_Valve_GPIO_Pin GPIO_PIN_12
#define Solenoid_Valve_GPIO_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
