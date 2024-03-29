/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define SL_Pin GPIO_PIN_4
#define SL_GPIO_Port GPIOA
#define CLK_Pin GPIO_PIN_5
#define CLK_GPIO_Port GPIOA
#define DAT_Pin GPIO_PIN_6
#define DAT_GPIO_Port GPIOA
#define CAPS_LOCK_LED_Pin GPIO_PIN_0
#define CAPS_LOCK_LED_GPIO_Port GPIOB
#define SCROLL_LOCK_LED_Pin GPIO_PIN_1
#define SCROLL_LOCK_LED_GPIO_Port GPIOB
#define NUM_LOCK_LED_Pin GPIO_PIN_10
#define NUM_LOCK_LED_GPIO_Port GPIOB
#define CS_LINE_KEY_Pin GPIO_PIN_13
#define CS_LINE_KEY_GPIO_Port GPIOB
#define LINE_KEY0_Pin GPIO_PIN_14
#define LINE_KEY0_GPIO_Port GPIOB
#define LINE_KEY1_Pin GPIO_PIN_15
#define LINE_KEY1_GPIO_Port GPIOB
#define LINE_KEY2_Pin GPIO_PIN_8
#define LINE_KEY2_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
