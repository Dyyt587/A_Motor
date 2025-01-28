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

/* Private defines -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
short vbus_voltage = 120, device_temperature = 250;
int ADCValue[6], ADC_Offset[6], ADC_Value[6];
int Id, Iq, Iq_real, Id_real;
// short phase_dir = 1;
//  short phase_dir_B = 1, hall_phase_dir = 1,
short vel_dir = 1;
int Iq_demand = 0, Id_demand = 0;
int speed_demand = 0, position_demand;
int commutation_current = 2000, motor_rated_current = 2000, motor_peak_current = 2000, motor_overload_time = 1000;

uint16_t motor_code = 0;
Encoder_Type_e feedback_type = Default;
short over_voltage, under_voltage, chop_voltage, over_temperature;
short tamagawa_offset = 0, tamagawa_dir = 1;
short Driver_Ready = 0;

ENC_Z enc_z = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
Hall_t hall = {0, 0, 0, 0, 0, 0}; // 霍尔传感器结构体
// svpwm_t motor.svpwm = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int encoder_direction_temp = 0, encoder_direction_temp_b = 0;
short hall_phase[8], ENC_Z_Offset = 2680, hall_phase_offset = 0, ENC_Z_Phase = 0, ENC_Z_Phase_B = 0, ENC_Z_Phase_Err = 0;
int hall_position = 0, hall_position_b = 0;
short encoder_offset_diff = 0, hall_phase_offset_diff = 0;

int led_blink_counter = 0, led_blink_period = 1000;

u16 store_parameter = 0;

Motor_t motor = {
	.motor_timer = &htim1,
	.PWM1_Duty = TIM_1_8_PERIOD_CLOCKS / 2,
	.PWM2_Duty = TIM_1_8_PERIOD_CLOCKS / 2,
	.PWM3_Duty = TIM_1_8_PERIOD_CLOCKS / 2,
	.control_deadline = TIM_1_8_PERIOD_CLOCKS,
	.PhaseU_current_ma = 0,
	.PhaseV_current_ma = 0,
	.PhaseW_current_ma = 0,

	.shunt_conductance = 500, // 100 means 1 mOh, current sensing resistor
	.phase_resistor = 5,	  //[S]
	.phase_inductance = 5,	  //[S]
	.encoder_timer = &htim3,
	.encoder_offset = 0,
	.encoder_state = 0,
	.phase = 0.0f,	 // [rad]
	.pll_pos = 0.0f, // [rad]
	.pll_vel = 0.0f, // [rad/s]
	.pll_kp = 0.0f,	 // [rad/s / rad]
	.pll_ki = 0.0f,	 // [(rad/s^2) / rad]

	.motion = {
		.feedback_resolution = 4000,
		.commutation_time = 1000,
	},
	.param = {
		.poles_num = 2,
		.phase_dir = 1,
	},

};
int update_motor_work_handle(wkc_t *wkc)
{
	update_motor(&motor, tamagawa_angle); // 更新电角度
	return 0;
}
int Current_loop_work_handle(wkc_t *wkc)
{
	Current_loop(&motor, Id_demand, Iq_demand);
	return 0;
}

int Velocity_loop_work_handle(wkc_t *wkc) // 电流环控制
{
	send_to_tamagawa();
	Velocity_loop(&motor, speed_demand);
	return 0;
}
int Position_loop_work_handle(wkc_t *wkc)
{
	Position_Loop(&motor, position_demand);
	return 0;
}
/* Private constant data -----------------------------------------------------*/
static const int one_by_sqrt3 = 577;
static const int sqrt3_by_2 = 866;

/* Function implementations --------------------------------------------------*/
wkc_work_t update_motor_work = {
	.name = "update_motor",
	.handle = update_motor_work_handle,
	.licence = {0},
	.trig_level = 0, // 触发等级，每次触发
	.trig_cnt = 0,
	.next = 0,
	.user_date = &motor,
};
wkc_work_t Current_loop_work = {
	.name = "Current_loop",
	.handle = Current_loop_work_handle,
	.licence = {0},
	.trig_level = 0, // 触发等级，每次触发
	.trig_cnt = 0,
	.next = 0,
	.user_date = &motor,
};
wkc_work_t Velocity_loop_work = {
	.name = "Velocity_loop",
	.handle = Velocity_loop_work_handle,
	.licence = {0},
	.trig_level = 3, // 触发等级，每4次触发
	.trig_cnt = 1,//防止第一次超时
	.next = 0,
	.user_date = &motor,
};
wkc_work_t Position_loop_work = {
	.name = "Position_loop",
	.handle = Position_loop_work_handle,
	.licence = {0},
	.trig_level = 7, // 触发等级，每8次触发
	.trig_cnt = 2,
	.next = 0,
	.user_date = &motor,
};
// Initalises the low level motor control and then starts the motor control threads
void init_motor_control(void)
{
	wkc_init(&motor.wkc);
	wkc_work_add(&motor.wkc, &update_motor_work);
	wkc_work_add(&motor.wkc, &Current_loop_work);
	wkc_work_add(&motor.wkc, &Velocity_loop_work);
	wkc_work_add(&motor.wkc, &Position_loop_work);
	delay_ms(10);
	// if(vbus_voltage<140)
	// motor.motion.Error_State=motor.motion.Error_State|0x0010;

	// if(motor.motion.Error_State*0x10==0)
	start_adc();

	Calibrate_ADC_Offset();

	// Start Encoders
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

	Driver_Ready = 1;

	delay_ms(20);
	if (feedback_type == Tamagawa)
	{
		if (Tamagawa_First < 10)
		{
			motor.motion.Error_State.bits.ENC_error = 1;
		}
		else
		{
			motor.angle_b = motor.angle = tamagawa_angle;
			pos_offest = tamagawa_angle;
			motor.encoder_state = pos_offest;
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
 * @brief 启动motor.svpwm
 *
 * @param htim
 */
void start_pwm(TIM_HandleTypeDef *htim)
{
	// Init PWM
	int half_load = TIM_1_8_PERIOD_CLOCKS / 2;
	htim->Instance->CCR1 = half_load;
	htim->Instance->CCR2 = half_load;
	htim->Instance->CCR3 = half_load;

	// This hardware obfustication layer really is getting on my nerves
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_3);
}

/**
 * @brief 停止motor.svpwm
 *
 * @param htim
 */
void stop_pwm(TIM_HandleTypeDef *htim)
{
	// Init PWM
	int half_load = TIM_1_8_PERIOD_CLOCKS / 2;
	htim->Instance->CCR1 = half_load;
	htim->Instance->CCR2 = half_load;
	htim->Instance->CCR3 = half_load;

	// This hardware obfustication layer really is getting on my nerves
	HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(htim, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Stop(htim, TIM_CHANNEL_3);
}
/**
 * @brief 计算电流
 *
 * @param ADCValue
 * @return int
 */
int phase_current_from_adcval(uint32_t ADCValue)
{
	int amp_gain = AMP_GAIN;

	int amp_out_volt = ONE_ADC_VOLTAGE * ADCValue;				   // adc value to voltage /unit uv
	int shunt_volt = amp_out_volt / amp_gain;					   // 实际电阻两端电压/unit uv
	int current_ma = (shunt_volt * 100) / motor.shunt_conductance; // unit mA=uv/mohm
	return current_ma;
}

void Calibrate_ADC_Offset(void)
{
	delay_ms(200);
	ADC_Offset[0] = ADCValue[0];
	ADC_Offset[1] = ADCValue[1];
	if ((ADC_Offset[0] < 1800) || (ADC_Offset[0] > 2200))
		motor.motion.Error_State.bits.ADC_error = 1;
	if ((ADC_Offset[1] < 1800) || (ADC_Offset[1] > 2200))
		motor.motion.Error_State.bits.ADC_error = 1;
}
int tA, tB, tC;
void queue_modulation_timings(Motor_t *motors, int mod_alpha, int mod_beta)
{

	SVM(mod_alpha, mod_beta, &tA, &tB, &tC);
	motors->PWM1_Duty = (tC * TIM_1_8_PERIOD_CLOCKS) / 1000;
	motors->PWM2_Duty = (tB * TIM_1_8_PERIOD_CLOCKS) / 1000;
	motors->PWM3_Duty = (tA * TIM_1_8_PERIOD_CLOCKS) / 1000;

	motors->motor_timer->Instance->CCR1 = motors->PWM1_Duty;
	motors->motor_timer->Instance->CCR2 = motors->PWM2_Duty;
	motors->motor_timer->Instance->CCR3 = motors->PWM3_Duty;
}

int32_t get_electric_phase(int commutation_current)
{
	int32_t phase_offset, current_step, i;

	Iq_demand = commutation_current;
	delay_ms(motor.motion.commutation_time);
	phase_offset = motor.encoder_state;
	delay_ms(motor.motion.commutation_time);
	return phase_offset;
}

void find_commutation(void)
{
	static int my_p0, my_p1, my_dir;
	switch (motor.motion.commutation_mode)
	{
	case 0:
		switch (feedback_type)
		{
		case Default:
		case Tamagawa:
		case Unknown_8:
			motor.param.phase_dir = 1; // must set back to the default value
			start_pwm(&htim1);
			motor_on = 1;
			motor.phase = 0;
			my_p0 = get_electric_phase(commutation_current);
			motor.phase = PI / 2;
			my_p1 = get_electric_phase(commutation_current);
			motor.phase = 0;
			my_p0 = get_electric_phase(commutation_current);

			Iq_demand = 0;

			if (my_p1 >= my_p0)
			{
				motor.param.phase_dir = 1;
				if ((motor.motion.feedback_resolution / (5 * motor.param.poles_num)) < (my_p1 - my_p0) && (my_p1 - my_p0) < (motor.motion.feedback_resolution / (3 * motor.param.poles_num)))
				{
					motor.motion.commutation_founded = 1;
					motor.encoder_state = 0;
					motor.encoder_offset = my_p0 % motor.motion.feedback_resolution;
				}
			}
			else
			{
				motor.param.phase_dir = -1;
				if ((motor.motion.feedback_resolution / (5 * motor.param.poles_num)) < (my_p0 - my_p1) && (my_p1 - my_p0) < (motor.motion.feedback_resolution / (3 * motor.param.poles_num)))
				{
					motor.motion.commutation_founded = 1;
					motor.encoder_state = 0;
					motor.encoder_offset = motor.motion.feedback_resolution - my_p0 % motor.motion.feedback_resolution;
				}
			}
			if (feedback_type == Tamagawa)
			{
				if (motor.motion.commutation_founded == 1)
				{
					tamagawa_dir = motor.param.phase_dir;
					tamagawa_offset = motor.encoder_offset;
				}
			}
			if (motor.motion.commutation_founded == 0)
			{
				stop_pwm(&htim1);
				motor_on = 0;
			}
			break;
		default:
			break;
		}
		break;
	case 1:
	case 2:
		switch (feedback_type)
		{
		case Default:
			motor.encoder_state = 0;
			enc_z.count = 0;
			enc_z.count_back = 0;
			enc_z.first = 0;
			motor.motion.commutation_founded = 1;
			break;
		case Tamagawa:
			motor.param.phase_dir = tamagawa_dir;
			motor.encoder_state = tamagawa_angle;
			motor.encoder_offset = tamagawa_offset;
			motor.motion.commutation_founded = 1;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	// motors[0].rotor.encoder_offset=get_electric_phase(commutation_current)+motor.motion.feedback_resolution/(4*motor.param.poles_num);
}
int rad_of_round = 2 * PI * 2;
void update_motor(Motor_t *motors, uint16_t angle)
{
	int16_t delta_enc;
	//@TODO stick parameter into struct
	rad_of_round = 2 * PI * motor.param.poles_num;

	motors->angle = angle;

	delta_enc = motors->angle - motors->angle_b;
	if (delta_enc < (-motor.motion.feedback_resolution / 2))
		delta_enc += motor.motion.feedback_resolution;
	if (delta_enc > (motor.motion.feedback_resolution / 2))
		delta_enc -= motor.motion.feedback_resolution;
	motors->angle_b = motors->angle;
	motors->encoder_state += (int32_t)delta_enc;

	int ph;
	// 计算电角度
	int32_t enc_state_mod = motors->encoder_state % motor.motion.feedback_resolution;
	if (motor.param.phase_dir == 1)
		ph = PI / 2 + (rad_of_round * (enc_state_mod - motors->encoder_offset)) / motor.motion.feedback_resolution;
	else
		ph = PI / 2 + (rad_of_round * ((motor.motion.feedback_resolution - enc_state_mod) - motors->encoder_offset)) / motor.motion.feedback_resolution;

	// ph = fmod(ph, 2 * PI);
	ph = ph % (2 * PI);

	if (motor.motion.commutation_founded)
		motors->phase = ph;
}

u32 calibrate_timechk;
void calibrate_tamagawa_encoder(void)
{
	if (set_tamagawa_zero == 2)
	{
		auto_reverse_p_time = 0;
		auto_reverse_n_time = 0;
		motor.control.target_Iq = commutation_current;
		motor.control.target_speed = 0;
		motor.param.phase_dir = 1;
		operation_mode = 0;
		control_word.all = 0x0f;
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
			motor.control.target_Iq = 0;
			motor.control.target_speed = 0;
			operation_mode = 2; // const speed
			Tamagawa_count_temp = 0;
			control_word.all = 0x86;
			set_tamagawa_zero = 0;
		}
	}
}

void motor_driver_handle(void)
{
	static int loop_counter_c = 0, loop_counter_v = 1, loop_counter_p = 2, current_loop_ready = 0, velocity_loop_ready = 0, position_loop_ready = 0;

	if (Driver_Ready)
	{
		wkc_handle(&motor.wkc);
	}
}