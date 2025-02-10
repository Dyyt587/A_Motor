/* Includes ------------------------------------------------------------------*/

#include <mcpwm.h>

#include <stdlib.h>
#include <math.h>
#include <park.h>
// #include <cmsis_os.h>

#include <main.h>
#include <adc.h>
#include <tim.h>
#include <utils.h>
#include "delay.h"
#include "perf_counter.h"
#include "workchain.h"
#include "amotor_port.h"

/* Private defines -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
short vbus_voltage = 120, device_temperature = 250;
int ADCValue[6], ADC_Offset[6], ADC_Value[6];
// int Id, Iq, Iq_real, Id_real;
//  short phase_dir = 1;
//   short phase_dir_B = 1, hall_phase_dir = 1,
short vel_dir = 1;
int Iq_demand = 0, Id_demand = 0;
int speed_demand = 0, position_demand;
int commutation_current = 2000, motor_rated_current = 2000, motor_peak_current = 2000, motor_overload_time = 1000;

short over_voltage, under_voltage, chop_voltage, over_temperature;
short tamagawa_offset = 0, tamagawa_dir = 1;

// ENC_Z enc_z = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// Hall_t hall = {0, 0, 0, 0, 0, 0}; // 霍尔传感器结构体
//  svpwm_t motors->svpwm = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int encoder_direction_temp = 0, encoder_direction_temp_b = 0;
short hall_phase[8], ENC_Z_Offset = 2680, hall_phase_offset = 0, ENC_Z_Phase = 0, ENC_Z_Phase_B = 0, ENC_Z_Phase_Err = 0;
int hall_position = 0, hall_position_b = 0;
short encoder_offset_diff = 0, hall_phase_offset_diff = 0;

int led_blink_counter = 0, led_blink_period = 1000;

u16 store_parameter = 0;

/* Private constant data -----------------------------------------------------*/
static const int one_by_sqrt3 = 577;
static const int sqrt3_by_2 = 866;

/* Function implementations --------------------------------------------------*/

// Initalises the low level motor control and then starts the motor control threads
void init_motor_control(Motor_t* motors)
{
	//Motor_t* motors = &motor;
// #define kp 600
// #define ki 30
	APID_Init(&motors->apidd, PID_INCREMENT, motors->param.pid_id.kp, motors->param.pid_id.ki, motors->param.pid_id.kd);
	APID_Set_Integral_Limit(&motors->apidd, motors->param.pid_id.integral_limit);
	APID_Set_Out_Limit(&motors->apidd, motors->param.pid_id.out_limit);

	APID_Init(&motors->apidq, PID_INCREMENT, motors->param.pid_iq.kp, motors->param.pid_iq.ki, motors->param.pid_iq.kd);
	APID_Set_Integral_Limit(&motors->apidq, motors->param.pid_iq.integral_limit);
	APID_Set_Out_Limit(&motors->apidq, motors->param.pid_iq.out_limit);

	APID_Init(&motors->apidv, PID_INCREMENT, motors->param.pid_vel.kp, motors->param.pid_vel.ki, motors->param.pid_vel.kd);
	// APID_Set_Integral_Limit(&motors->apidv, kvi_sum_limit * 10);
	APID_Set_Out_Limit(&motors->apidv, motors->param.pid_vel.out_limit);

	APID_Init(&motors->apidp, PID_POSITION, motors->param.pid_pos.kp, motors->param.pid_pos.ki, motors->param.pid_pos.kd);
	APID_Set_Integral_Limit(&motors->apidp, motors->param.pid_pos.integral_limit);
	APID_Set_Out_Limit(&motors->apidp, motors->param.pid_pos.out_limit);

	extern void works_init(void);
	works_init();
	delay_ms(10);
	// if(vbus_voltage<140)
	// motors->motion.Error_State=motors->motion.Error_State|0x0010;

	// if(motors->motion.Error_State*0x10==0)
	start_adc();

	Calibrate_ADC_Offset();

	// Start Encoders
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

	motors->wkc.lic_aprove.bits.drv_init = 1;
	motors->wkc.lic_aprove.bits.drv_ready = 1;
	motors->wkc.lic_aprove.bits.svm_apply = 1;

	motors->wkc.lic_aprove.bits.torque_mode = 1;
	motors->wkc.lic_aprove.bits.velocity_mode = 1;
	motors->wkc.lic_aprove.bits.position_mode = 1;

	delay_ms(20);
	if (motors->feedback_type == Tamagawa)
	{
		if (Tamagawa_First < 10)
		{
			motors->motion.Error_State.bits.ENC_error = 1;
		}
		else
		{
			motors->angle_b = motors->angle = tamagawa_angle;
			pos_offest = tamagawa_angle;
			motors->encoder_state = pos_offest;
		}
	}
	delay_ms(10);
}

//@TODO make available from anywhere
void safe_assert(int arg)
{
	if (!arg)
	{
		htim1.Instance->BDTR &= ~(TIM_BDTR_MOE);
		for (;;)
			;
	}
}

void start_adc(void)
{
	Motor_t* motors = &motor;

	/* Run the ADC calibration */
	if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
	{
		/* Calibration Error */
		Error_Handler();
	}
	htim1.Instance->CCR4 = 500;
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);
}

/**
 * @brief 计算电流
 *
 * @param ADCValue
 * @return int
 */
int phase_current_from_adcval(uint32_t ADCValue)
{
		Motor_t* motors = &motor;

	int amp_gain = AMP_GAIN;

	int amp_out_volt = ONE_ADC_VOLTAGE * ADCValue;				   // adc value to voltage /unit uv
	int shunt_volt = amp_out_volt / amp_gain;					   // 实际电阻两端电压/unit uv
	int current_ma = (shunt_volt * 100) / motors->shunt_conductance; // unit mA=uv/mohm
	return current_ma;
}

void Calibrate_ADC_Offset(void)
{	Motor_t* motors = &motor;

	delay_ms(200);
	ADC_Offset[0] = ADCValue[0];
	ADC_Offset[1] = ADCValue[1];
	if ((ADC_Offset[0] < 1800) || (ADC_Offset[0] > 2200))
		motors->motion.Error_State.bits.ADC_error = 1;
	if ((ADC_Offset[1] < 1800) || (ADC_Offset[1] > 2200))
		motors->motion.Error_State.bits.ADC_error = 1;
}


int rad_of_round = 2 * PI * 2;
void update_motor(Motor_t *motors, uint16_t angle)
{
	//////////////////////TODO:临界区域///////////////////////////////////
	motors->encoder_time_us_b = motors->encoder_time_us;
	motors->encoder_time_us = get_time_us();
//////////////////////////////////////////////////////////
	int16_t delta_enc;
	//@TODO stick parameter into struct
	rad_of_round = 2 * PI * motors->param.poles_num;

	motors->angle = angle;

	delta_enc = motors->angle - motors->angle_b;
	if (delta_enc < (-motors->motion.feedback_resolution / 2))
		delta_enc += motors->motion.feedback_resolution;
	if (delta_enc > (motors->motion.feedback_resolution / 2))
		delta_enc -= motors->motion.feedback_resolution;
	motors->angle_b = motors->angle;
	motors->encoder_state += (int32_t)delta_enc;

	int ph;
	// 计算电角度
	int32_t enc_state_mod = motors->encoder_state % motors->motion.feedback_resolution;
	if (motors->param.phase_dir == 1)
		ph = PI / 2 + (rad_of_round * (enc_state_mod - motors->encoder_offset)) / motors->motion.feedback_resolution;
	else
		ph = PI / 2 + (rad_of_round * ((motors->motion.feedback_resolution - enc_state_mod) - motors->encoder_offset)) / motors->motion.feedback_resolution;

	// ph = fmod(ph, 2 * PI);
	ph = ph % (2 * PI);

	if (motors->wkc.lic_aprove.bits.commutation_founded)
		motors->phase = ph;
}

u32 calibrate_timechk;
void calibrate_tamagawa_encoder(void)
{
		Motor_t* motors = &motor;

	if (set_tamagawa_zero == 2)
	{
		auto_reverse_p_time = 0;
		auto_reverse_n_time = 0;
		motors->control.target_Iq = commutation_current;
		motors->control.target_speed = 0;
		motors->param.phase_dir = 1;
		//operation_mode = 0;
		motors->wkc.lic_aprove.bits.motor_on = 1;
		//control_word.all = 0x0f;
		calibrate_timechk = HAL_GetTick();
		set_tamagawa_zero = 3;
	}
	if (set_tamagawa_zero == 3)
	{
		if ((HAL_GetTick() - calibrate_timechk) > 1000)
		{
			set_tamagawa_zero = 1;
			calibrate_timechk = HAL_GetTick();
		}
	}
	if (set_tamagawa_zero == 1)
	{
		if ((HAL_GetTick() - calibrate_timechk) > 1000)
		{
			set_tamagawa_zero = 4;
			calibrate_timechk = HAL_GetTick();
		}
	}
	if (set_tamagawa_zero == 4)
	{
		if ((HAL_GetTick() - calibrate_timechk) > 500)
		{
			motors->control.target_Iq = 0;
			motors->control.target_speed = 0;
			//operation_mode = 2; // const speed
			Tamagawa_count_temp = 0;
			motors->wkc.lic_aprove.bits.motor_on=0;
			//control_word.all = 0x86;
			set_tamagawa_zero = 0;
		}
	}
}

void motor_driver_handle(void)
{
		Motor_t* motors = &motor;

	wkc_handle(&motors->wkc);
}