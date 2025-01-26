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

int32_t encoder_state_b;
int real_speed,real_speed_filter,speed_err,kvi_sum=0,kvi_sum_limit=3000;
short kvp=253,kvi=8;
int real_speed_buff[32];
int buff_sum=0;
int real_speed_buff_point;
short real_speed_filter_num=20,low_pass_filter_on=1;
short speed_in_lpf_a=1000,speed_out_lpf_a=1000;
int Iq_temp=0,Ilim=5000;
int Step_phase=0;
int check_vel_overshot_p=0,check_vel_overshot_n=0;
int hall_speed_loop_count=0,hall_speed_update=0;
int display_speed_loop_count=0,display_speed_update=0,display_speed=0,display_encoder_state_b=0;
void Velocity_loop(Motor_t* motors, int target_vel)
{
	Update_Speed(motors);
	if(motor_on)
		switch(operation_mode)
		{
			case 1:
			case 3:
			case 2:
			case 5:
			case 7:	
				if(commutation_founded)
				{
					if(low_pass_filter_on)
						speed_err=target_vel-real_speed_filter;
					else
						speed_err=target_vel-real_speed;
					Iq_temp = vel_dir*(kvp * speed_err+kvi_sum);
					kvi_sum=kvi_sum+kvi*speed_err;
					if(kvi_sum > kvi_sum_limit*1000)kvi_sum = kvi_sum_limit*1000;
					if(kvi_sum < -kvi_sum_limit*1000)kvi_sum = -kvi_sum_limit*1000;
					Iq_temp=Iq_temp/1000;
					if (Iq_temp > Ilim) Iq_temp = Ilim;
					if (Iq_temp < -Ilim) Iq_temp = -Ilim;
					Iq_demand=Low_pass_filter_1(speed_out_lpf_a,Iq_temp,Iq_demand);
					//Iq_demand=Iq_temp;
				}
				if(target_vel>0)
				{
						if(speed_err<check_vel_overshot_p)
							check_vel_overshot_p=speed_err;
				}
				if(target_vel<0)
				{
						if(speed_err>check_vel_overshot_n)
							check_vel_overshot_n=speed_err;
				}
				break;
			case 12:
			case 13:
			case 11:
			case 17:
				//if((target_vel>1000)||(target_vel<-1000))
				{
					if(hall_speed_update==1)
					{
						hall_speed_update=0;
						if(commutation_founded)
						{
							if(low_pass_filter_on)
								speed_err=target_vel-real_speed_filter;
							else
								speed_err=target_vel-real_speed;
							Iq_temp = (kvp * speed_err+kvi_sum);
							kvi_sum=kvi_sum+kvi*speed_err;
							if(kvi_sum > kvi_sum_limit*1000)kvi_sum = kvi_sum_limit*1000;
							if(kvi_sum < -kvi_sum_limit*1000)kvi_sum = -kvi_sum_limit*1000;
							Iq_temp=Iq_temp/1000;
							if (Iq_temp > Ilim) Iq_temp = Ilim;
							if (Iq_temp < -Ilim) Iq_temp = -Ilim;
							Iq_demand=Low_pass_filter_1(speed_out_lpf_a,Iq_temp,Iq_demand);
							//Iq_demand=Iq_temp;
						}
						if(target_vel>0)
						{
								if(speed_err<check_vel_overshot_p)
									check_vel_overshot_p=speed_err;
						}
						if(target_vel<0)
						{
								if(speed_err>check_vel_overshot_n)
									check_vel_overshot_n=speed_err;
						}
					}	
				}
				break;
			case 0:
				commutation_founded=0;
				Step_phase+=target_vel;
			
				Step_phase = Step_phase%(2*M_PI);
				if(Step_phase<0)
					Step_phase+=2*M_PI;
				motors->phase=Step_phase;
				break;
		default:
			break;
	}
	
}
void Update_Speed(Motor_t* motors)
{
	switch(operation_mode)
	{
		case 1:
		case 3:
		case 4:
		case 2:
		case 5:
		case 7:	
			real_speed=(motors->encoder_state-encoder_state_b)*1000*4000/feedback_resolution;
			encoder_state_b=motors->encoder_state;
			real_speed_filter=Low_pass_filter_1(speed_in_lpf_a,real_speed,real_speed_filter);
			display_speed_loop_count++;
			if(display_speed_loop_count>399) //100ms
			{	
				display_speed_loop_count=0;
				display_speed=(motors->encoder_state-display_encoder_state_b)*600/feedback_resolution;
				display_encoder_state_b=motors->encoder_state;
			}
			//real_speed_filter=Low_pass_filter(real_speed_buff,real_speed,real_speed_filter_num);
			break;
		case 0:	
			real_speed=speed_demand*637/(poles_num);
			display_speed=real_speed/20;
			break;
		case 14:
		case 12:
		case 13:
		case 11:
		case 17:
			hall_speed_loop_count++;
			display_speed_loop_count++;
			if(hall_speed_loop_count>99)
			{
				hall_speed_loop_count=0;
				hall_speed_update=1;
				real_speed=((hall_position-hall_position_b)*6667)/poles_num;
				hall_position_b=hall_position;
				//real_speed_filter=Low_pass_filter(real_speed_buff,real_speed,real_speed_filter_num);
				real_speed_filter=Low_pass_filter_1(speed_in_lpf_a,real_speed,real_speed_filter);
			}
			if(display_speed_loop_count>399) //100ms
			{	
				display_speed_loop_count=0;
				display_speed=((hall_position-display_encoder_state_b)*1000)/poles_num;
				display_encoder_state_b=hall_position;
			}
			break;
		
			
	}
}

int Low_pass_filter(int* Buffer,int X,int n)
{
	int i;
	int temp;
	buff_sum=0.0f;
	real_speed_buff_point++;
	if(real_speed_buff_point>=n)
	{
		real_speed_buff_point=0;
	}
	Buffer[real_speed_buff_point]=X;
	for(i=0;i<n;i++)
	{
		buff_sum+=Buffer[i];
	}
	return buff_sum/n;
}

// Y=A*X+(1-A)*Y0 -> Y=Y0+((X-Y0)*A)/1000; 0<A<1000
int Low_pass_filter_1(int A,int X,int Y)
{
	return Y+((X-Y)*A)/1000;
}


