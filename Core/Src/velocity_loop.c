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

int32_t encoder_state_b;
int real_speed, real_speed_filter, speed_err, kvi_sum = 0, kvi_sum_limit = 3000;
short kvp = 253, kvi = 8;
int real_speed_buff[32];
int buff_sum = 0;
int real_speed_buff_point;
short real_speed_filter_num = 20, low_pass_filter_on = 1;
short speed_in_lpf_a = 1000, speed_out_lpf_a = 1000;
int Iq_temp = 0, Ilim = 5000;

int check_vel_overshot_p = 0, check_vel_overshot_n = 0;
int hall_speed_loop_count = 0, hall_speed_update = 0;
int display_speed_loop_count = 0, display_speed_update = 0, display_speed = 0, display_encoder_state_b = 0;
void Velocity_loop(Motor_t *motors, int target_vel)
{
	//Update_Speed(motors);

	if (operation_mode != 4)
	{
		// if (motor.wkc.lic_aprove.bits.commutation_founded )
		// {

			APID_Set_Target(&motors->apidv, target_vel);
 
			APID_Set_Present(&motors->apidv, real_speed);
			//APID_Set_Present(&motors->apidv, real_speed_filter);
 			APID_Hander(&motors->apidv, 1);

			Iq_demand = Low_pass_filter_1(speed_out_lpf_a, -motors->apidv.parameter.out/1000, Iq_demand);
			// Iq_demand=Iq_temp;
		// }
	}
}
void Update_Speed(Motor_t *motors)
{
	real_speed = (motors->encoder_state - encoder_state_b) * 1000 * 4000 / motor.motion.feedback_resolution;
	encoder_state_b = motors->encoder_state;
	real_speed_filter = Low_pass_filter_1(speed_in_lpf_a, real_speed, real_speed_filter);
	display_speed_loop_count++;
	if (display_speed_loop_count > 399) // 100ms
	{
		display_speed_loop_count = 0;
		display_speed = (motors->encoder_state - display_encoder_state_b) * 600 / motor.motion.feedback_resolution;
		display_encoder_state_b = motors->encoder_state;
	}
}

int Low_pass_filter(int *Buffer, int X, int n)
{
	int i;
	int temp;
	buff_sum = 0.0f;
	real_speed_buff_point++;
	if (real_speed_buff_point >= n)
	{
		real_speed_buff_point = 0;
	}
	Buffer[real_speed_buff_point] = X;
	for (i = 0; i < n; i++)
	{
		buff_sum += Buffer[i];
	}
	return buff_sum / n;
}

// Y=A*X+(1-A)*Y0 -> Y=Y0+((X-Y0)*A)/1000; 0<A<1000
int Low_pass_filter_1(int A, int X, int Y)
{
	return Y + ((X - Y) * A) / 1000;
}
