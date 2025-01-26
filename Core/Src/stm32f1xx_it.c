/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f1xx_it.h"
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
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern CAN_HandleTypeDef hcan;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

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
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
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
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles RCC global interrupt.
  */
void RCC_IRQHandler(void)
{
  /* USER CODE BEGIN RCC_IRQn 0 */

  /* USER CODE END RCC_IRQn 0 */
  /* USER CODE BEGIN RCC_IRQn 1 */

  /* USER CODE END RCC_IRQn 1 */
}

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
	int i;
/* EXTI line interrupt detected */
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) != RESET)
  {
		ENC_Z_Count_C=motor.encoder_timer->Instance->CNT;
		//for(i=0;i<1;i++);
		if(HAL_GPIO_ReadPin(ENC_Z_GPIO_Port,ENC_Z_Pin)) //to avoid the noise
		{
			ENC_Z_Trig=1;
			ENC_Z_Phase=motor.phase;
			ENC_Z_Phase_Err=ENC_Z_Phase_B+M_PI/2-ENC_Z_Phase;
			encoder_offset_diff=(ENC_Z_Phase_Err*feedback_resolution)/(M_PI*poles_num*2);
			ENC_Z_Count_B=ENC_Z_Count;
			ENC_Z_Count=ENC_Z_Count_C;
			ENC_Z_Diff=ENC_Z_Count-ENC_Z_Count_B;
			ENC_Z_Pos_B=ENC_Z_Pos;
			ENC_Z_Pos=motor.encoder_state;
			
		}
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
    HAL_GPIO_EXTI_Callback(GPIO_PIN_0);
  }

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles ADC1 and ADC2 global interrupts.
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  HAL_ADC_IRQHandler(&hadc2);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
  * @brief This function handles USB low priority or CAN RX0 interrupts.
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 0 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan);
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 1 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_2);
	t2=SysTick->VAL;
	
	ADCValue[0]=HAL_ADCEx_InjectedGetValue(&hadc1, 1);
	ADCValue[1]=HAL_ADCEx_InjectedGetValue(&hadc2, 1);
	ADCValue[2]=HAL_ADC_GetValue(&hadc2);
	ADCValue[3]=HAL_ADC_GetValue(&hadc1);
	
	vbus_voltage=(ADCValue[3]*19)/100;

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
						set_tamagawa_zero=4;
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

		if(loop_counter_p>15)
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
	tt3=t2-tt2;
	if(tt3>tt4)
		tt4=tt3;
	t3=t1-t2;
	t1=t2;
	if(t3>t4)
		t4=t3;
	//ttt3=t1-ttt1;
  /* USER CODE END TIM1_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

		if(position_loop_ready==1)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
			ttt1=SysTick->VAL;
			ttt3=ttt2-ttt1;
			ttt2=ttt1;
			if(ttt3>ttt4)
				ttt4=ttt3;
			if(motor_on)
				Motion_process();
			position_loop_ready=0;
			
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
		}
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

	u32 timeout=0;
	
	timeout=0;
    while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)//|?′???━??━??y???━aD??
	{
	 timeout++;////3???━o??′?┬??|??━?：：??━a
     if(timeout>100) 
		 {
			 huart2.RxState=HAL_UART_STATE_READY;
			 __HAL_UNLOCK(&huart2);
			 break;
		 }		
	
	}
     
	timeout=0;
	while(HAL_UART_Receive_IT(&huart2, (u8 *)USART2RxBuffer, RXBUFFERSIZE) != HAL_OK)//??━????????|??━?：：??━a??━a??━o3??━|??o??━???′o???D??a???D??2??′|??━|??━??━??RxXferCount?a1
	{
	 timeout++; //3???━o??′?┬??|??━?：：??━a
	 if(timeout>100) break;	
	}
	__HAL_UART_CLEAR_OREFLAG(&huart2);
  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

	u32 timeout=0;
	
	timeout=0;
    while (HAL_UART_GetState(&huart3) != HAL_UART_STATE_READY)//|?′???━??━??y???━aD??
	{
	 timeout++;////3???━o??′?┬??|??━?：：??━a
     if(timeout>100) 
		 {
			 huart3.RxState=HAL_UART_STATE_READY;
			 __HAL_UNLOCK(&huart3);
			 break;
		 }		
	
	}
     
	timeout=0;
	while(HAL_UART_Receive_IT(&huart3, (u8 *)USART3RxBuffer, RXBUFFERSIZE) != HAL_OK)//??━????????|??━?：：??━a??━a??━o3??━|??o??━???′o???D??a???D??2??′|??━|??━??━??RxXferCount?a1
	{
	 timeout++; //3???━o??′?┬??|??━?：：??━a
	 if(timeout>100) break;	
	}
	__HAL_UART_CLEAR_OREFLAG(&huart3);
  /* USER CODE END USART3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
