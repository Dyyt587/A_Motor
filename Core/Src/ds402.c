#include <stm32g0xx_hal.h> //Sets up the correct chip specifc defines required by arm_math
// #define ARM_MATH_CM4
// #include <arm_math.h>

#include <mcpwm.h>

#include <stdlib.h>
#include <math.h>
// #include <cmsis_os.h>

#include <main.h>
#include <adc.h>
#include <tim.h>
#include <spi.h>
#include <utils.h>
#include "delay.h"
#include "perf_counter.h"
#include "amotor_port.h"

short operation_mode = 1, operation_mode_buff = 0;
union Status_uint16_t status_word;
union Control_uint16_t control_word, control_word_b;
// u16 motor_on=0;

void DS402_process(void)
{
	if (control_word.all != control_word_b.all)
	{
		switch (control_word.all)
		{
		case 0x0f:
			if (motor.motion.Error_State.all == 0)
			{
				// Scop_Start=1;
				// V_current_control_integral_d=0;
				// V_current_control_integral_q=0;
				Iq_demand = 0;
				speed_demand = 0;
				position_demand = 0;

				if (motor.wkc.lic_aprove.bits.commutation_founded == 0)
				{
					//find_commutation();
					delay_ms(1);
				}
				if (motor.wkc.lic_aprove.bits.commutation_founded == 1)
				{
					if (motor.motion.Error_State.all == 0)
					{
						pwm_start(&motor);
						motor.wkc.lic_aprove.bits.motor_on = 1;
					}
				}
				if (motor.motion.work_mode == Default_Mode)
				{
					if (motor.motion.Error_State.all == 0)
					{
						pwm_start(&motor);
						motor.wkc.lic_aprove.bits.motor_on = 1;
					}
				}
				if (motor.motion.work_mode == Position_Mode)
				{
					target_speed_now = real_speed_filter * 1000;
					target_pos_now = pos_actual;
					speed_demand = real_speed_filter;
					position_demand = pos_actual;
				}
			}
			break;
		case 0x06:
			pwm_stop(&motor);
			motor.wkc.lic_aprove.bits.motor_on = 0;
			break;
		case 0x86:
			pwm_stop(&motor);
			motor.wkc.lic_aprove.bits.motor_on = 0;
			// drv8301_error=0;
			motor.motion.Error_State.all = 0;
			// enc_z.counting_error=0;
			break;
		default:
			break;
		}
		control_word_b = control_word;
	}
	// if (motor.motion.Error_State.all)
	// {
	// 	pwm_stop(&htim1);
	// 	motor.wkc.lic_aprove.bits.motor_on = 0;
	// }
}
