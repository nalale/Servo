/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "tools.h"
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
#define SYSLED_R_Pin GPIO_PIN_13
#define SYSLED_R_GPIO_Port GPIOC
#define CLOCK_ENABLE_Pin GPIO_PIN_12
#define CLOCK_ENABLE_GPIO_Port GPIOB
#define SHIFT_CLK_Pin GPIO_PIN_13
#define SHIFT_CLK_GPIO_Port GPIOB
#define SHIFT_DATA_IN_Pin GPIO_PIN_14
#define SHIFT_DATA_IN_GPIO_Port GPIOB
#define LATCH_DATA_Pin GPIO_PIN_8
#define LATCH_DATA_GPIO_Port GPIOA
#define RS485_RTS_Pin GPIO_PIN_11
#define RS485_RTS_GPIO_Port GPIOA
#define SYSLED_G_Pin GPIO_PIN_8
#define SYSLED_G_GPIO_Port GPIOB
#define SYSLED_B_Pin GPIO_PIN_9
#define SYSLED_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define USER_SWITCH_SENS_NUMBER		50
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
