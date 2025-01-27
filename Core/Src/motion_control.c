//#include <stm32f4xx_hal.h> //Sets up the correct chip specifc defines required by arm_math
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
#include "oled.h"
#include "apid.h"
#include "perf_counter.h"

s32 profile_acce=5000,profile_dece=5000,profile_speed=5000,end_speed=0,profile_speed_b,target_speed_now=0,direction=1;
s32 profile_target_position,profile_target_position_b;
s32 home_offest=0,homing_speed=0,homing_acce=0;
s8 homing_method=0;
s32 acce_diatance,dece_diatance,distance_diff=0,decelerating_position=0,target_pos_now=0,searching_speed=0;
int64_t target_pos_diff;
u8 accelerating=0,decelerating=0,positionning=0,motion_state=0;
int64_t remain_dst=0,step_dst=0;
s32 auto_p_pos=5000,auto_n_pos=-5000;
u32 auto_reverse_p_time=0,auto_reverse_n_time=0,auto_reverse_time=0,auto_reverse_status;
u16 auto_switch_on=0;
int pluse_num=0,pluse_num_r=0,pul_dir=0,gear_factor_a=100,gear_factor_b=10,pluse_num_beforegear=0;
u16 pluse_temp=0,pluse_temp_b=0,delta_pulse=0,delta_pulse_r=0;
short motion_out_lpf_a=1000;
char display_buff[40],display_buff1[40],display_buff2[40],display_buff3[40],display_buff4[40];
short OLED_count=0,OLED_Period=200;

int EN_state=0,EN_state_b=0,DIN1_state=0,DIN1_state_b=0,DIN2_state=0,DIN2_state_b=0,DIN3_state=0,DIN3_state_b=0,DOUT1_state=0,DOUT1_state_b=0;


extern apid_t apidd;
extern apid_t apidq;
s32 Acce_distance_cal(int acce,int speed)
{
	int64_t v,a,r,d;
	v=speed;
	a=acce;
	r=feedback_resolution;
	d=(v*v*r)/(2000*a);
	return d;
}
void Motion_process(void)
{
	
	int temp32a,temp32b;
	int64_t temp64a,temp64b,temp64c,temp64d;
	switch(operation_mode)
	{
		case 1:
		case 11:
			switch(motion_state)
			{
				case 1://accelerating
					remain_dst=direction*(profile_target_position_b-target_pos_now);
					dece_diatance=Acce_distance_cal(profile_dece,(target_speed_now/1000));
					if(remain_dst<dece_diatance)
					{
						decelerating=1;
						accelerating=0;
						motion_state=3;
					}
					target_speed_now+=profile_acce;
					if(target_speed_now>(profile_speed*1000))
					{
						target_speed_now=profile_speed*1000;
						accelerating=0;
						motion_state=2;
					}
					temp64a=target_speed_now;
					temp64b=feedback_resolution;
					step_dst=(temp64a*temp64b)/1000000000;
					target_pos_now=target_pos_now+direction*step_dst;
					break;
				case 2://const speed
					remain_dst=direction*(profile_target_position_b-target_pos_now);
					dece_diatance=Acce_distance_cal(profile_dece,(target_speed_now/1000));
					if(remain_dst<dece_diatance)
					{
						decelerating=1;
						accelerating=0;
						motion_state=3;
					}
					temp64a=target_speed_now;
					temp64b=feedback_resolution;
					step_dst=(temp64a*temp64b)/1000000000;
					target_pos_now=target_pos_now+direction*step_dst;
					break;
				case 3://decelerating
					remain_dst=direction*(profile_target_position_b-target_pos_now);
					//dece_diatance=Acce_distance_cal(profile_dece,(target_speed_now/1000));
					temp64a=(2*profile_dece*remain_dst*1000)/feedback_resolution;
					temp64b= sqrt(temp64a);
					target_speed_now=1000*temp64b;
				
					if(target_speed_now<(searching_speed*1000))
					{
						target_speed_now=searching_speed*1000;
						decelerating=0;
						motion_state=4;
					}
				
					temp64a=target_speed_now;
					temp64b=feedback_resolution;
					step_dst=(temp64a*temp64b)/1000000000;
					target_pos_now=target_pos_now+direction*step_dst;
					if(direction*target_pos_now>direction*profile_target_position_b)
					{
						target_pos_now=profile_target_position_b;
						decelerating=0;
						positionning=0;
						motion_state=0;
					}
					break;
				case 4://searching
					temp64a=target_speed_now;
					temp64b=feedback_resolution;
					step_dst=(temp64a*temp64b)/1000000000;
					target_pos_now=target_pos_now+direction*step_dst;
					if(direction*target_pos_now>=direction*profile_target_position_b)
					{
						target_pos_now=profile_target_position_b;
						decelerating=0;
						positionning=0;
						motion_state=0;
					}
					break;
				default:
					break;
			}
			
			//position_demand=Low_pass_filter_1(motion_out_lpf_a,target_pos_now,position_demand);
			position_demand=target_pos_now;
			
			if((target_position!=profile_target_position_b)&&(motion_state==0))
			{
				profile_target_position_b=target_position;
				positionning=1;
				distance_diff=target_position-pos_actual;
				//target_speed_now=real_speed_filter*1000;
				//target_pos_now=pos_actual;
				if(distance_diff>=0)
				{
					direction=1;
					acce_diatance=Acce_distance_cal(profile_acce,profile_speed);// 计算加减速距离
					dece_diatance=Acce_distance_cal(profile_dece,profile_speed);
					accelerating=1;
					decelerating=0;
					motion_state=1;
				}
				if(distance_diff<0)
				{
					direction=-1;
					acce_diatance=Acce_distance_cal(profile_acce,profile_speed);
					dece_diatance=Acce_distance_cal(profile_dece,profile_speed);
					accelerating=1;
					decelerating=0;
					motion_state=1;
				}
			}
			
			break;
		case 3:
		case 13:
			if(target_speed!=profile_speed_b)
			{
				profile_speed_b=target_speed;
				//target_speed_now=real_speed_filter*1000;
				//target_pos_now=pos_actual;		
			}
			if(profile_speed_b>=0)
			{
				if(target_speed_now<profile_speed_b*1000)
				{
					target_speed_now+=profile_acce;
					if(target_speed_now>profile_speed_b*1000)
						target_speed_now=profile_speed_b*1000;
				}
				if(target_speed_now>profile_speed_b*1000)
				{
					target_speed_now-=profile_dece;
					if(target_speed_now<profile_speed_b*1000)
						target_speed_now=profile_speed_b*1000;
				}
			}
			if(profile_speed_b<0)
			{
				if(target_speed_now>profile_speed_b*1000)
				{
					target_speed_now-=profile_acce;
					if(target_speed_now<profile_speed_b*1000)
						target_speed_now=profile_speed_b*1000;
				}
				if(target_speed_now<profile_speed_b*1000)
				{
					target_speed_now+=profile_dece;
					if(target_speed_now>profile_speed_b*1000)
						target_speed_now=profile_speed_b*1000;
				}
			}
			
			temp64a=target_speed_now;
			temp64b=feedback_resolution;
			step_dst=(temp64a*temp64b)/1000000000;
			target_pos_now=target_pos_now+step_dst;
			
			position_demand=target_pos_now;
			break;
		case 7:
		case 17:
			position_demand=target_position;
			break;
		case 2:
		case 12:
			speed_demand=target_speed;
			break;
		case 0:
			speed_demand=target_speed;
			Iq_demand=target_Iq;
			break;
		case 4:
		case 14:
			Iq_demand=target_Iq;
			break;
		case 5:	
			if(commutation_founded)
			{
				//pul_dir=HAL_GPIO_ReadPin(DIR_GPIO_Port,DIR_Pin);
				//pluse_temp=__HAL_TIM_GET_COUNTER(&htim2);
				delta_pulse=pluse_temp-pluse_temp_b;
				pluse_temp_b=pluse_temp;
				pluse_num=delta_pulse;
				if(pul_dir)
				{
					//pluse_num+=delta_pulse;
					pluse_num_beforegear+=pluse_num;
					position_demand+=(pluse_num*gear_factor_a+pluse_num_r)/gear_factor_b;
					pluse_num_r=(pluse_num*gear_factor_a+pluse_num_r)%gear_factor_b;
				}
				else
				{
					pluse_num_beforegear-=pluse_num;
					position_demand-=(pluse_num*gear_factor_a+pluse_num_r)/gear_factor_b;
					pluse_num_r=(delta_pulse*gear_factor_a+pluse_num_r)%gear_factor_b;
					//pluse_num-=delta_pulse;
				}
			}	
			else
			{
				//__HAL_TIM_SET_COUNTER(&htim2,0);
				pluse_temp=0;
				pluse_temp_b=0;
			}
			break;
		default:
			break;
	}
	
}

void Auto_reserve_process(void)
{
	if(auto_switch_on)
	{
		if(motor_on==0)
			if(Error_State.all==0)
				if(control_word.all==0x06)
					control_word.all=0x0f;
	}
	
	if(motor_on)	
	if(auto_reverse_n_time||auto_reverse_p_time)
	{
		switch(auto_reverse_status)
		{
			case 0:
				auto_reverse_time=HAL_GetTick();
				auto_reverse_status=1;
				break;
			case 1:
				if((HAL_GetTick()-auto_reverse_time)>auto_reverse_p_time)
				{
					auto_reverse_time=HAL_GetTick();
					switch(operation_mode)
					{
						case 2:
						case 3:
						case 12:
						case 13:
							target_speed=auto_p_pos;
						break;
						case 1:
						case 7:
						case 11:
						case 17:
							target_position=auto_p_pos;
						break;
						case 4:
						case 14:
						//case 0:
							target_Iq=auto_p_pos;
						break;
					}	
					auto_reverse_status=2;
				}
				break;
			case 2:
				if((HAL_GetTick()-auto_reverse_time)>auto_reverse_n_time)
				{
					auto_reverse_time=HAL_GetTick();
					switch(operation_mode)
					{
						case 2:
						case 3:
						case 12:
						case 13:
							target_speed=auto_n_pos;
						break;
						case 1:
						case 7:
						case 11:
						case 17:
							target_position=auto_n_pos;
						break;
						case 4:
						case 14:
						//case 0:
							target_Iq=auto_n_pos;
						break;
					}
					auto_reverse_status=1;
				}
				break;
			default:
				break;
		}
	}
	if((auto_reverse_n_time==0)&&(auto_reverse_p_time==0))
	{
		auto_reverse_status=0;
	}

}

void IO_Process(void)
{
	//DIN1_state=HAL_GPIO_ReadPin(DIN1_GPIO_Port,DIN1_Pin);
	//DIN2_state=HAL_GPIO_ReadPin(DIN2_GPIO_Port,DIN2_Pin);
	//DIN3_state=HAL_GPIO_ReadPin(DIN3_GPIO_Port,DIN3_Pin);
	
	if(EN_state>EN_state_b)
	{
		control_word.all=0x0f;
	}
	else if(EN_state<EN_state_b)
	{
		control_word.all=0x06;
	}
	
	EN_state_b=EN_state;
	DIN1_state_b=DIN1_state;
	DIN2_state_b=DIN2_state;
	DIN3_state_b=DIN3_state;
}

void LED_Process(void)
{
		led_blink_counter++;
		if(Error_State.all==0)
		{
			if(motor_on)
			{
				HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
				led_blink_period=500-abs(real_speed_filter)/50;
				if(led_blink_period<1)
				{
					led_blink_period=1;
				}
				if(led_blink_counter>led_blink_period)
					led_blink_counter=0;
				if(led_blink_counter>(led_blink_period/2))
				{
					HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
				}
				else
				{
					HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
				}
			}
			else
			{
				HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
				led_blink_period=1000;
				if(led_blink_counter>led_blink_period)
					led_blink_counter=0;
				if(led_blink_counter>(led_blink_period/2))
				{
					HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
				}
				else
				{
					HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
				}
			}
		}
		else
		{
			HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
			led_blink_period=200;
			if(led_blink_counter>led_blink_period)
				led_blink_counter=0;
			if(led_blink_counter>(led_blink_period/2))
			{
				HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
			}
			
		}
}


void OLED_Process(void)
{
	float temp;
		switch(operation_mode)
		{
			case 2:
			case 3:
			case 12:
			case 13:
				sprintf(display_buff,"Velocity Mode   ");
			break;
			case 1:
			case 7:
			case 11:
			case 17:
				sprintf(display_buff,"Position Mode   ");
			break;
			case 4:
			case 14:
				sprintf(display_buff,"Torque Mode   ");
			break;
			case 0:
				sprintf(display_buff,"OpenSpeed Mode  ");
			break;
			case 5:
				sprintf(display_buff,"PUL/DIR mode   ");
			break;
		}
		//OLED_Clear();
		//Lcd_Clear(BLACK);
		//Lcd_Fill(0,0,160,20,BLACK);
		if(Error_State.all)
		{		
			sprintf(display_buff4,"ERROR");
			switch(Error_State.all)
			{
					case 2:
						sprintf(display_buff1,"ADC_error     ");
					break;
					case 8:
						sprintf(display_buff1,"ENC_error     ");
					break;
					case 16:
						sprintf(display_buff1,"hall_state_error     ");
					break;
					case 32:
						sprintf(display_buff1,"commutation_error     ");
					break;
					case 64:
						sprintf(display_buff1,"following_error     ");
					break;
					case 256:
						sprintf(display_buff1,"voltage_low     ");
					break;
					case 512:
						sprintf(display_buff1,"voltage_high     ");
					break;
					case 1024:
						sprintf(display_buff1,"over_temperature     ");
					break;
					case 2048:
						sprintf(display_buff1,"over_current     ");
					break;
					case 4096:
						sprintf(display_buff1,"over_load     ");
					break;
				
			}
			
		}
		else
		{
			if(motor_on)
			{
				sprintf(display_buff4," ON  ");
			}
			else
			{
				sprintf(display_buff4," OFF ");
			}
			switch(operation_mode)
			{
				case 2:
				case 3:
				case 12:
				case 13:
				case 0:	
					temp=Iq_real/1000.0;
					sprintf(display_buff,"%s%.4f%s","Current:",temp,"      ");
					sprintf(display_buff1,"%s%d%s","Speed:",(display_speed),"       ");
				break;
				case 1:
				case 7:
				case 11:
				case 17:
					sprintf(display_buff1,"%s%d%s","Pos  :",pos_actual,"       ");
				break;
				case 4:
				case 14:
					temp=Iq_real/1000.0;
					sprintf(display_buff1,"%s%.1f%s","Current:",temp,"      ");
				break;
				case 5:
					sprintf(display_buff1,"%s%d%s","Pos  :",pos_actual,"      ");
				break;
			}
			
			 
			/*
			else
			{
				sprintf(display_buff1,"ready on!    ");
			}
			*/
		}
		//temp=vbus_voltage/10.0;
		temp=apidd.parameter.target - apidd.parameter.present;
		sprintf(display_buff2,"%s%.1f%s","e:",temp,"");
		extern int Vd;
		extern uint64_t nCycleUsed;

		temp=apidd.parameter.present;
		sprintf(display_buff3,"%s%lld%s","p:",perfc_convert_ticks_to_us(nCycleUsed),"us");
		
	  OLED_ShowString(0,2,(u8*)display_buff,12);
	  OLED_ShowString(0,3,(u8*)display_buff1,12);
	  OLED_ShowString(0,0,(u8*)display_buff4,16);
	  OLED_ShowString(50,0,(u8*)display_buff2,12);
	  OLED_ShowString(50,1,(u8*)display_buff3,12);
		
}

int key1_state,key2_state,key3_state,key1_state_b,key2_state_b,key3_state_b,key12_count=0,key3_count=0;
void KEY_Process(void)
{

	//Ilim=(motor_peak_current*ADCValue[4])/4096; //电位计调节限制的电流即扭矩
	key1_state=HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin);
	key2_state=HAL_GPIO_ReadPin(KEY2_GPIO_Port,KEY2_Pin);
	key3_state=HAL_GPIO_ReadPin(KEY3_GPIO_Port,KEY3_Pin);
	
	DIN1_state=HAL_GPIO_ReadPin(DIN1_GPIO_Port,DIN1_Pin);
	DIN2_state=HAL_GPIO_ReadPin(DIN2_GPIO_Port,DIN2_Pin);
	DIN3_state=HAL_GPIO_ReadPin(DIN3_GPIO_Port,DIN3_Pin);
	switch(operation_mode)
	{
		case 0:
			//target_Iq=Ilim/2;
			if(key1_state<key1_state_b)
				target_speed+=2;
			if(key2_state<key2_state_b)
				target_speed-=2;
			if(target_speed>100)
				target_speed=100;
			if(target_speed<-100)
				target_speed=-100;
			break;
		case 2:
			if(key1_state<key1_state_b)
				target_speed+=1000;
			if(key2_state<key2_state_b)
				target_speed-=1000;
			if(target_speed>100000)
				target_speed=100000;
			if(target_speed<-100000)
				target_speed=-100000;
			break;
		case 1:
			if(key1_state<key1_state_b)
				target_position+=16384;
			if(key2_state<key2_state_b)
				target_position-=16384;
			break;	
		case 4:
			if(key1_state<key1_state_b)
				target_Iq+=200;
			if(key2_state<key2_state_b)
				target_Iq-=200;
			break;	
	}
	

	if(key3_state<key3_state_b)
	{
		if((control_word.all==0x06)||(control_word.all==0x86))
			control_word.all=0x0f;
		else if(control_word.all==0x0f)
			control_word.all=0x06;
	}	
			
	key1_state_b=key1_state;
	key2_state_b=key2_state;
	key3_state_b=key3_state;
	
	DIN1_state_b=DIN1_state;
	DIN2_state_b=DIN2_state;
	DIN3_state_b=DIN3_state;

}

