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

typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;  
typedef const int16_t sc16;  
typedef const int8_t sc8;  

typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

typedef __I int32_t vsc32;  
typedef __I int16_t vsc16; 
typedef __I int8_t vsc8;   

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;  
typedef const uint16_t uc16;  
typedef const uint8_t uc8; 

typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

typedef __I uint32_t vuc32;  
typedef __I uint16_t vuc16; 
typedef __I uint8_t vuc8; 

union error_uint32_t
{
	uint32_t all;
	struct
	{
		uint8_t drv_fault:1;						 //0
		uint8_t ADC_error:1;						//1
		uint8_t :1;											//2
		uint8_t ENC_error:1;						//3
		uint8_t hall_state_error:1;			//4
		uint8_t commutation_error:1;		//5
		uint8_t following_error:1;			//6
		uint8_t :1;											//7
		uint8_t voltage_low:1;					//8
		uint8_t voltage_high:1;					//9
		uint8_t over_temperature:1;			//10
		uint8_t over_current:1;					//11
		uint8_t over_load:1;						//12
		uint8_t :1;											//13
		uint8_t :1;											//14
		uint8_t :1;											//15
	}bits;
};


union Status_uint16_t
{
	uint16_t all;
	struct
	{
		uint8_t driver_ready:1;						 //0
		uint8_t motor_on:1;						//1
		uint8_t operation_enable:1;											//2
		uint8_t error:1;											//3
		uint8_t voltage_enable:1;			//4
		uint8_t quick_stop:1;		//5
		uint8_t switch_on_disable:1;			//6
		uint8_t warning:1;											//7
		uint8_t manufacture0:1;					//8
		uint8_t remote:1;					//9
		uint8_t target_reach:1;			//10
		uint8_t internal_limit_active:1;					//11
		uint8_t set_point_ack:1;											//12
		uint8_t following_error:1;											//13
		uint8_t commutation_found:1;											//14
		uint8_t reference_found:1;											//15
	}bits;
};

union Control_uint16_t
{
	uint16_t all;
	struct
	{
		uint8_t Switch_on:1;						 //0
		uint8_t Enable_voltage:1;						//1
		uint8_t Quick_stop:1;											//2
		uint8_t Enable_operation:1;											//3
		uint8_t New_set_point:1;			//4
		uint8_t Change_set_immediately:1;		//5
		uint8_t Abs_relative:1;			//6
		uint8_t Fault_reset:1;											//7
		uint8_t Halt:1;					//8
		uint8_t :1;					//9
		uint8_t :1;			//10
		uint8_t :1;					//11
		uint8_t :1;											//12
		uint8_t :1;											//13
		uint8_t :1;											//14
		uint8_t :1;											//15
	}bits;
};


union can_int32_t
{
	int32_t all;
	struct
	{
		int8_t byte_0;
		int8_t byte_1;
		int8_t byte_2;
		int8_t byte_3;
	}byte;
	struct
	{
		int16_t word_0;
		int16_t word_1;
	}word;		
};
union can_int16_t
{
	int16_t all;
	
	struct
	{
		int8_t byte_0;
		int8_t byte_1;
	}byte;		
};

typedef struct {
	uint8_t cmd;
	uint16_t identifier;
    union can_int32_t od;
    union can_int32_t data;
	struct 
	{
		int8_t len;
		int8_t data_0;
		int8_t data_1;
		int8_t data_2;
		int8_t data_3;
		int8_t data_4;
		int8_t data_5;
		int8_t data_6;
		int8_t data_7;
	}pdo;
} rtx_task_action_t;

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
