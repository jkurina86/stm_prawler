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
#include "stm32l4xx_hal.h"

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
void SystemClock_Config(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define PB0_USART3_EN_Pin GPIO_PIN_0
#define PB0_USART3_EN_GPIO_Port GPIOB
#define PB1_USART2_EN_Pin GPIO_PIN_1
#define PB1_USART2_EN_GPIO_Port GPIOB
#define SD_PWR_Pin GPIO_PIN_2
#define SD_PWR_GPIO_Port GPIOB
#define RTC_INTod_Pin GPIO_PIN_10
#define RTC_INTod_GPIO_Port GPIOB
#define RTC_INTod_EXTI_IRQn EXTI15_10_IRQn
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define CLK_OE_Pin GPIO_PIN_6
#define CLK_OE_GPIO_Port GPIOC
#define PB4_AUX_SEL_A0_Pin GPIO_PIN_4
#define PB4_AUX_SEL_A0_GPIO_Port GPIOB
#define PB5_AUX_SEL_A1_Pin GPIO_PIN_5
#define PB5_AUX_SEL_A1_GPIO_Port GPIOB
#define RECORD_TRIGGER_Pin GPIO_PIN_8
#define RECORD_TRIGGER_GPIO_Port GPIOB
#define RECORD_TRIGGER_EXTI_IRQn EXTI9_5_IRQn
#define PB9_TRUCK_INT_OUT_Pin GPIO_PIN_9
#define PB9_TRUCK_INT_OUT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
