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
#define OLED_RESET_Pin GPIO_PIN_14
#define OLED_RESET_GPIO_Port GPIOC
#define MCLK_EN_Pin GPIO_PIN_0
#define MCLK_EN_GPIO_Port GPIOA
#define CANCEL_LED_Pin GPIO_PIN_3
#define CANCEL_LED_GPIO_Port GPIOA
#define N_LEFT_Pin GPIO_PIN_0
#define N_LEFT_GPIO_Port GPIOB
#define N_LEFT_EXTI_IRQn EXTI0_IRQn
#define N_RIGHT_Pin GPIO_PIN_1
#define N_RIGHT_GPIO_Port GPIOB
#define N_RIGHT_EXTI_IRQn EXTI1_IRQn
#define N_SELECT_Pin GPIO_PIN_2
#define N_SELECT_GPIO_Port GPIOB
#define N_SELECT_EXTI_IRQn EXTI2_IRQn
#define ENC_L_B_Pin GPIO_PIN_13
#define ENC_L_B_GPIO_Port GPIOB
#define RIGHT_LED_Pin GPIO_PIN_8
#define RIGHT_LED_GPIO_Port GPIOA
#define LEFT_LED_Pin GPIO_PIN_9
#define LEFT_LED_GPIO_Port GPIOA
#define N_FLASH_RESET_Pin GPIO_PIN_10
#define N_FLASH_RESET_GPIO_Port GPIOA
#define SELECT_LED_Pin GPIO_PIN_13
#define SELECT_LED_GPIO_Port GPIOA
#define TEMPO_LED_Pin GPIO_PIN_14
#define TEMPO_LED_GPIO_Port GPIOA
#define N_AMP_SHDN_Pin GPIO_PIN_15
#define N_AMP_SHDN_GPIO_Port GPIOA
#define ENC_L_A_Pin GPIO_PIN_3
#define ENC_L_A_GPIO_Port GPIOB
#define ENC_L_A_EXTI_IRQn EXTI3_IRQn
#define ENC_R_A_Pin GPIO_PIN_4
#define ENC_R_A_GPIO_Port GPIOB
#define ENC_R_A_EXTI_IRQn EXTI4_IRQn
#define ENC_R_B_Pin GPIO_PIN_5
#define ENC_R_B_GPIO_Port GPIOB
#define ENC_R_B_EXTI_IRQn EXTI9_5_IRQn
#define N_CANCEL_Pin GPIO_PIN_8
#define N_CANCEL_GPIO_Port GPIOB
#define N_CANCEL_EXTI_IRQn EXTI9_5_IRQn
#define N_CODEC_IRQ_Pin GPIO_PIN_9
#define N_CODEC_IRQ_GPIO_Port GPIOB
#define N_CODEC_IRQ_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
