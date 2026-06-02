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
#include "stm32h7xx_hal.h"

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
#define Power_OUT2_EN_Pin GPIO_PIN_13
#define Power_OUT2_EN_GPIO_Port GPIOC
#define Power_OUT1_EN_Pin GPIO_PIN_14
#define Power_OUT1_EN_GPIO_Port GPIOC
#define Power_5V_EN_Pin GPIO_PIN_15
#define Power_5V_EN_GPIO_Port GPIOC
#define ACCL_CS_Pin GPIO_PIN_0
#define ACCL_CS_GPIO_Port GPIOC
#define GYRO_CS_Pin GPIO_PIN_3
#define GYRO_CS_GPIO_Port GPIOC
#define photo4_Pin GPIO_PIN_0
#define photo4_GPIO_Port GPIOA
#define photo3_Pin GPIO_PIN_2
#define photo3_GPIO_Port GPIOA
#define IMU_HEAT_Pin GPIO_PIN_1
#define IMU_HEAT_GPIO_Port GPIOB
#define photo2_Pin GPIO_PIN_9
#define photo2_GPIO_Port GPIOE
#define ACCL_INT_Pin GPIO_PIN_10
#define ACCL_INT_GPIO_Port GPIOE
#define ACCL_INT_EXTI_IRQn EXTI15_10_IRQn
#define GYRO_INT_Pin GPIO_PIN_12
#define GYRO_INT_GPIO_Port GPIOE
#define GYRO_INT_EXTI_IRQn EXTI15_10_IRQn
#define checkphoto_spear_Pin GPIO_PIN_14
#define checkphoto_spear_GPIO_Port GPIOE
#define USER_KEY_Pin GPIO_PIN_15
#define USER_KEY_GPIO_Port GPIOA
#define USER_KEY_EXTI_IRQn EXTI15_10_IRQn
#define valve_arm_Pin GPIO_PIN_10
#define valve_arm_GPIO_Port GPIOC
#define valve_rod_Pin GPIO_PIN_11
#define valve_rod_GPIO_Port GPIOC
#define checkphoto_orelow_Pin GPIO_PIN_0
#define checkphoto_orelow_GPIO_Port GPIOE
#define checkphoto_orehigh_Pin GPIO_PIN_1
#define checkphoto_orehigh_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
