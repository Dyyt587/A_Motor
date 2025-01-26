/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g0xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mcpwm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
int t1,t2,t3,t4,t5,t6;
int tt1,tt2,tt3,tt4,tt5,tt6;
int ttt1,ttt2,ttt3=0,ttt4=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim6;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	LED_Process();
	OLED_count++;
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel 1 interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

t2=SysTick->VAL;
	t3=t1-t2;
t2=t1;
	if(t3>t4)
		t4=t3;
  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel 2 and channel 3 interrupts.
  */
void DMA1_Channel2_3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 0 */

  /* USER CODE END DMA1_Channel2_3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 1 */

  /* USER CODE END DMA1_Channel2_3_IRQn 1 */
}

/**
  * @brief This function handles ADC1 interrupt.
  */
void ADC1_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_IRQn 0 */

  /* USER CODE END ADC1_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC1_IRQn 1 */

  /* USER CODE END ADC1_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break, update, trigger and commutation interrupts.
  */
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_UP_TRG_COM_IRQn 0 */

  /* USER CODE END TIM1_BRK_UP_TRG_COM_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_BRK_UP_TRG_COM_IRQn 1 */

  /* USER CODE END TIM1_BRK_UP_TRG_COM_IRQn 1 */
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */
t1=SysTick->VAL;
			ttt1=SysTick->VAL;
			ttt3=ttt2-ttt1;
			ttt2=ttt1;
			if(ttt3>ttt4)
				ttt4=ttt3;
	ADCValue[0]=ADC_Value[0];
	ADCValue[1]=ADC_Value[1];
	ADCValue[2]=ADC_Value[2];
	ADCValue[3]=ADC_Value[3];
	ADCValue[4]=ADC_Value[4];
	ADCValue[5]=ADC_Value[5];
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Value, 6);
	
	vbus_voltage=(ADCValue[3]*19)/106;

	NTC_R_Value=1000*ADCValue[2]/(4096-ADCValue[2]);
	device_temperature=Get_NTC_Temperature(NTC_R_Value);
	
	if(Driver_Ready)
	{
		loop_counter_c++;
		loop_counter_v++;
		loop_counter_p++;
		
		update_motor(&motor);
		//if(motor_on)
			//if(loop_counter_c>1)
		{
			current_loop_ready=1;
			Current_loop(&motor, Id_demand, Iq_demand);
			loop_counter_c=0;
		}
		if(loop_counter_v>3)
		{
			//HAL_GPIO_TogglePin(ERR_GPIO_Port, ERR_Pin);
			velocity_loop_ready=1;
			//HAL_GPIO_WritePin(ERR_GPIO_Port, ERR_Pin, GPIO_PIN_SET);
			switch(feedback_type)
			{
				case 4:
					// take about 25us
					if(set_tamagawa_zero==0)
					{
		
						Tamagawa_TX_BUFF[0]=0x02;
						Tamagawa_Read_Cmd(Tamagawa_TX_BUFF,1);
						
						if(Tamagawa_First==10)
						{
							Tamagawa_count_temp++;
							if(Tamagawa_count_temp>1)
							{
								Tamagawa_lost++;
							}
							if(Tamagawa_count_temp>20)
							{
								if(set_tamagawa_zero==0)
									Error_State.bits.ENC_error=1;
							}
						}
					}
					else if(set_tamagawa_zero==1)
					{
		
						Tamagawa_TX_BUFF[0]=0xAA;
						Tamagawa_Read_Cmd(Tamagawa_TX_BUFF,1);
						set_tamagawa_zero=0;
					}
				
					break;
				case 8:
					//spi read take about 15us
				break;
				default:
					break;
			}
			// take baout 6us
			Velocity_loop(&motor,speed_demand);
			//HAL_GPIO_WritePin(ERR_GPIO_Port, ERR_Pin, GPIO_PIN_RESET);
			loop_counter_v=0;
		}

		if(loop_counter_p>7)
		{
			//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
			position_loop_ready=1;
			Position_Loop(&motor,position_demand);
			loop_counter_p=0;
		}
	}
	
	if(Scop_Start)
		Process_Scop_Data();
	
	tt2=SysTick->VAL;
	//tt3=tt2-tt1;
	tt3=t1-tt2;
	if(tt3>tt4)
		tt4=tt3;
	//ttt3=t1-ttt1;
  /* USER CODE END TIM1_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt.
  */
void TIM6_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_IRQn 0 */

		if(position_loop_ready==1)
		{
			//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
			if(motor_on)
				Motion_process();
			position_loop_ready=0;
			
			//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
		}
  /* USER CODE END TIM6_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_IRQn 1 */

  /* USER CODE END TIM6_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

	u32 timeout=0;
	
	timeout=0;
    while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)//|?§???ì??ì??y???ìaD??
	{
	 timeout++;////3???ìo??§?è??|??ì?¨¨??ìa
     if(timeout>HAL_MAX_DELAY) break;		
	
	}
     
	timeout=0;
	while(HAL_UART_Receive_IT(&huart1, (u8 *)USART1RxBuffer, RXBUFFERSIZE) != HAL_OK)//??ì????????|??ì?¨¨??ìa??ìa??ìo3??ì|??o??ì???§o???D??a???D??2??§|??ì|??ì??ì??RxXferCount?a1
	{
	 timeout++; //3???ìo??§?è??|??ì?¨¨??ìa
	 if(timeout>HAL_MAX_DELAY) break;	
	}
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

	u32 timeout=0;
	
	timeout=0;
    while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)//|?§???ì??ì??y???ìaD??
	{
	 timeout++;////3???ìo??§?è??|??ì?¨¨??ìa
     if(timeout>HAL_MAX_DELAY) break;		
	
	}
     
	timeout=0;
	while(HAL_UART_Receive_IT(&huart2, (u8 *)USART2RxBuffer, RXBUFFERSIZE) != HAL_OK)//??ì????????|??ì?¨¨??ìa??ìa??ìo3??ì|??o??ì???§o???D??a???D??2??§|??ì|??ì??ì??RxXferCount?a1
	{
	 timeout++; //3???ìo??§?è??|??ì?¨¨??ìa
	 if(timeout>HAL_MAX_DELAY) break;	
	}
  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
