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

int vel_lim = 50000;
short kpp = 10, kpi = 1;
s32 pos_actual, pos_offest = 0;
int pos_err, vel_des;
int kpi_sum = 0, kpi_sum_limit = 3000;
short position_in_lpf_a = 1000, position_out_lpf_a = 1000;

int check_pos_overshot_p = 0, check_pos_overshot_n = 0;
int Position_loop_first = 0;
void Position_Loop(Motor_t *motors, int target_pos)
{
	// if(operation_mode<11)
	// {
	// 	switch(motors->feedback_type)
	// 	{
	// 		case Default:
	// 		case Tamagawa:
	// 			pos_actual=motors->encoder_state-pos_offest;
	// 		break;
	// 		default:
	// 		break;

	// 	}
	// }
	// else if(operation_mode>10)
	// 	pos_actual=hall_position;
		APID_Set_Target(&motors->apidp, target_pos);

		APID_Set_Present(&motors->apidp, pos_actual);
		APID_Hander(&motors->apidp, 1);

		 motors->motion.speed_demand = Low_pass_filter_1(position_out_lpf_a, motors->apidp.parameter.out, motors->motion.speed_demand);
	// ENC_Z_Check();
//	switch (operation_mode)
//	{
//	case 1:
//	case 3:
//	case 7:
//		// if(motor.wkc.lic_aprove.bits.commutation_founded &&motor_on)
//		//  if(motor.wkc.lic_aprove.bits.motor_on)
//		//  {


//		// speed_demand=vel_des;
//		// }
//		// if(target_pos>0)
//		// {
//		// 		if(pos_err<check_pos_overshot_p)
//		// 			check_pos_overshot_p=pos_err;
//		// }
//		// if(target_pos<0)
//		// {
//		// 		if(pos_err>check_pos_overshot_n)
//		// 			check_pos_overshot_n=pos_err;
//		// }
//		break;
//	// case 11:
//	// case 13:
//	// case 17:
//	// 	if(motor.wkc.lic_aprove.bits.commutation_founded &&motor.wkc.lic_aprove.bits.motor_on)
//	// 	{
//	// 		pos_err = target_pos - pos_actual;
//	// 		/*
//	// 		if((pos_err<3)&&(pos_err<-3))
//	// 		{
//	// 			pos_err=0;
//	// 			kpi_sum=0;
//	// 		}*/
//	// 		vel_des = kpp * pos_err+kpi_sum;
//	// 		kpi_sum=kpi_sum+kpi*pos_err;

//	// 		if(kpi_sum > kpi_sum_limit)kpi_sum = kpi_sum_limit;
//	// 		if(kpi_sum < -kpi_sum_limit)kpi_sum = -kpi_sum_limit;
//	// 		if (vel_des >  vel_lim) vel_des =  vel_lim;
//	// 		if (vel_des < -vel_lim) vel_des = -vel_lim;

//	// 		speed_demand=Low_pass_filter_1(position_out_lpf_a,vel_des,speed_demand);
//	// 		//speed_demand=vel_des;
//	// 	}
//	// 	if(target_pos>0)
//	// 	{
//	// 		if(pos_err<check_pos_overshot_p)
//	// 			check_pos_overshot_p=pos_err;
//	// 	}
//	// 	if(target_pos<0)
//	// 	{
//	// 		if(pos_err>check_pos_overshot_n)
//	// 			check_pos_overshot_n=pos_err;
//	// 	}
//	// 	break;
//	default:
//		break;
//	}
	//	Check_DCBus();
	//	Check_Temperature();
	// Check_IIt();
	// Check_drv8301();
}

// void ENC_Z_Check(void)
//{
//	if(enc_z.trig)
//	{
//		enc_z.trig=0;
//		enc_z.pos_offset=enc_z.pos%motor.motion.feedback_resolution;
//		enc_z.pos_diff=enc_z.pos-enc_z.pos_back;
//		#if 0
//		if(enc_z.diff>=0)
//		{
//			if(((enc_z.diff % motor.motion.feedback_resolution)>ENC_Z_DIFF_ERROR)&&((enc_z.diff % motor.motion.feedback_resolution)<(motor.motion.feedback_resolution-ENC_Z_DIFF_ERROR)))
//			{
//				enc_z.diff_back=enc_z.diff;
//				enc_z.counting_error++;
//			}
//		}
//		else
//		{
//			if(((enc_z.diff % motor.motion.feedback_resolution)<-ENC_Z_DIFF_ERROR)&&((enc_z.diff % motor.motion.feedback_resolution)>(ENC_Z_DIFF_ERROR-motor.motion.feedback_resolution)))
//			{
//				enc_z.diff_back=enc_z.diff;
//				enc_z.counting_error++;
//			}
//		}
//		#else
//		if(enc_z.pos_diff>=0)
//		{
//			if(((enc_z.pos_diff % motor.motion.feedback_resolution)>ENC_Z_DIFF_ERROR)&&((enc_z.pos_diff % motor.motion.feedback_resolution)<(motor.motion.feedback_resolution-ENC_Z_DIFF_ERROR)))
//			{
//				enc_z.pos_diff_back=enc_z.pos_diff;
//				enc_z.counting_error++;
//			}
//		}
//		else
//		{
//			if(((enc_z.pos_diff % motor.motion.feedback_resolution)<-ENC_Z_DIFF_ERROR)&&((enc_z.pos_diff % motor.motion.feedback_resolution)>(ENC_Z_DIFF_ERROR-motor.motion.feedback_resolution)))
//			{
//				enc_z.pos_diff_back=enc_z.pos_diff;
//				enc_z.counting_error++;
//			}
//		}
//		#endif
//		if((motor.motion.commutation_founded==1)&&(motor.motion.commutation_mode==1))
//			if(enc_z.first==0)
//			{
//				motor.encoder_offset-=encoder_offset_diff;
//				enc_z.first=1;
//			}
//	}
// }

// void Check_DCBus(void)
//{
//	if(vbus_voltage>over_voltage)
//		motor.motion.Error_State.bits.voltage_high=1;
//	if(vbus_voltage<under_voltage)
//		motor.motion.Error_State.bits.voltage_low=1;
//
//	//if(vbus_voltage>chop_voltage)
//	//	HAL_GPIO_WritePin(BRAKE_GPIO_Port, BRAKE_Pin, GPIO_PIN_RESET);
//	//if(vbus_voltage<(chop_voltage-10))
//	//	HAL_GPIO_WritePin(BRAKE_GPIO_Port, BRAKE_Pin, GPIO_PIN_SET);
// }

// void Check_Temperature(void)
//{
//	if(device_temperature>over_temperature)
//		motor.motion.Error_State.bits.over_temperature=1;
// }

// void Check_IIt(void)
//{
//	Driver_IIt_Real=IIt_filter(Driver_IIt_Filter,Iq_real,Driver_IIt_Real);
//	if(Driver_IIt_Real>Driver_IIt_Current)
//		motor.motion.Error_State.bits.over_load=1;
//
//	Driver_IIt_Real_DC=IIt_DC_filter(Driver_IIt_Filter_DC,Iq_real,Driver_IIt_Real_DC);
//	if(Driver_IIt_Real_DC>Driver_IIt_Current_DC)
//		motor.motion.Error_State.bits.over_load=1;
// }

void Error_process(void)
{
	if (motor.motion.Error_State.all)
	{
		//status_word.bits.operation_enable = 0;
		//status_word.bits.error = 1;
	}
}
