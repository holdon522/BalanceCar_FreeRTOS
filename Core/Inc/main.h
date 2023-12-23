/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOC
#define OLED_SCL_Pin GPIO_PIN_4
#define OLED_SCL_GPIO_Port GPIOA
#define OLED_SDA_Pin GPIO_PIN_5
#define OLED_SDA_GPIO_Port GPIOA
#define speed1A_Pin GPIO_PIN_6
#define speed1A_GPIO_Port GPIOA
#define speed1B_Pin GPIO_PIN_7
#define speed1B_GPIO_Port GPIOA
#define TRIG_Pin GPIO_PIN_0
#define TRIG_GPIO_Port GPIOB
#define ECHO_Pin GPIO_PIN_1
#define ECHO_GPIO_Port GPIOB
#define ECHO_EXTI_IRQn EXTI1_IRQn
#define PWMA_Pin GPIO_PIN_10
#define PWMA_GPIO_Port GPIOB
#define MOTOR_B_1_Pin GPIO_PIN_12
#define MOTOR_B_1_GPIO_Port GPIOB
#define MOTOR_B_2_Pin GPIO_PIN_13
#define MOTOR_B_2_GPIO_Port GPIOB
#define MOTOR_A_1_Pin GPIO_PIN_14
#define MOTOR_A_1_GPIO_Port GPIOB
#define MOTOR_A_2_Pin GPIO_PIN_15
#define MOTOR_A_2_GPIO_Port GPIOB
#define IMU_INT_Pin GPIO_PIN_8
#define IMU_INT_GPIO_Port GPIOA
#define speed2A_Pin GPIO_PIN_6
#define speed2A_GPIO_Port GPIOB
#define speed2B_Pin GPIO_PIN_7
#define speed2B_GPIO_Port GPIOB
#define SCL_Pin GPIO_PIN_8
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_9
#define SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
