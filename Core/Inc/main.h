/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32g0xx_hal.h"

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
#define TIM_1_8_PERIOD_CLOCKS 4000
#define TIM_1_8_DEADTIME_CLOCKS 50
#define DIN1_Pin GPIO_PIN_1
#define DIN1_GPIO_Port GPIOC
#define DIN2_Pin GPIO_PIN_2
#define DIN2_GPIO_Port GPIOC
#define DIN3_Pin GPIO_PIN_3
#define DIN3_GPIO_Port GPIOC
#define ENC_Z_Pin GPIO_PIN_0
#define ENC_Z_GPIO_Port GPIOB
#define RS485_EN_Pin GPIO_PIN_1
#define RS485_EN_GPIO_Port GPIOB
#define TAMAGAWA_TX_EN_Pin GPIO_PIN_12
#define TAMAGAWA_TX_EN_GPIO_Port GPIOB
#define LED_1_Pin GPIO_PIN_9
#define LED_1_GPIO_Port GPIOC
#define LED_2_Pin GPIO_PIN_0
#define LED_2_GPIO_Port GPIOD
#define KEY1_Pin GPIO_PIN_1
#define KEY1_GPIO_Port GPIOD
#define KEY2_Pin GPIO_PIN_2
#define KEY2_GPIO_Port GPIOD
#define KEY3_Pin GPIO_PIN_3
#define KEY3_GPIO_Port GPIOD
#define SPI1_CS_Pin GPIO_PIN_6
#define SPI1_CS_GPIO_Port GPIOB
#define OLED_RES_Pin GPIO_PIN_9
#define OLED_RES_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
