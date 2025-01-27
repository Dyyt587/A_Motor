#include <stm32g0xx_hal.h> //Sets up the correct chip specifc defines required by arm_math
//#define ARM_MATH_CM4
//#include <arm_math.h>

#include <mcpwm.h>

#include <stdlib.h>
#include <math.h>
//#include <cmsis_os.h>

#include <main.h>
#include <adc.h>
#include <tim.h>
#include <spi.h>
#include <utils.h>
#include "delay.h"
#include "perf_counter.h"

short operation_mode=1,operation_mode_buff=0;
union Status_uint16_t status_word;
union Control_uint16_t control_word,control_word_b;
u16 motor_on=0;

void DS402_process(void)
{
	if(control_word.all!=control_word_b.all)
	{
		switch(control_word.all)
		{
			case 0x0f:
				if(motor.motion.Error_State.all==0)
				{
					//Scop_Start=1;
					V_current_control_integral_d=0;
					V_current_control_integral_q=0;
					Iq_demand=0;
					speed_demand=0;
					position_demand=0;
					switch(operation_mode)
					{
						case 14:
						case 12:
						case 11:
						case 17:
							phase_dir=1;
							motor.motion.commutation_founded=1;
							//kci=0;	
						case 0:
							if(motor.motion.Error_State.all==0)
							{
								start_pwm(&htim1);
								motor_on=1;
							}
							break;
						case 1:
						case 3:
							target_speed_now=real_speed_filter*1000;
							target_pos_now=pos_actual;	
							speed_demand=real_speed_filter;
							position_demand=pos_actual;
						case 4:
						case 2:
						case 5:
						case 7:	
							if(motor.motion.commutation_founded==0)
							{
								find_commutation();
								delay_ms(1);
							}
							if(motor.motion.commutation_founded==1)
							{
								if(motor.motion.Error_State.all==0)
								{
									start_pwm(&htim1);
									motor_on=1;
								}
							}
							break;
						default:
							break;
					}
				}
				break;
			case 0x06:
				stop_pwm(&htim1);
				motor_on=0;
				break;
			case 0x86:
				stop_pwm(&htim1);
				motor_on=0;
				//drv8301_error=0;
				motor.motion.Error_State.all=0;
				enc_z.counting_error=0;
				break;
			default:
				break;
		}
		control_word_b=control_word;
	}
	if(motor.motion.Error_State.all)
	{
		stop_pwm(&htim1);
		motor_on=0;	
	}

}

