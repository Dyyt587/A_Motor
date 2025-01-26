/* Includes ------------------------------------------------------------------*/

#include <mcpwm.h>

#include <stdlib.h>
#include <math.h>
// #include <cmsis_os.h>

#include <main.h>
#include <adc.h>
#include <tim.h>
#include <utils.h>
#include "delay.h"

/* Private defines -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
short vbus_voltage = 120, device_temperature = 250;
int ADCValue[6], ADC_Offset[6], ADC_Value[6];
int Id, Iq, Iq_real, Id_real;
short phase_dir = 1, phase_dir_B = 1, hall_phase_dir = 1, vel_dir = 1;
int Iq_demand = 0, Id_demand = 0, target_Iq = 0, target_Id = 0;
int target_speed = 0, speed_demand = 0, target_position = 0, target_position_b = 0, position_demand;
int commutation_current = 2000, motor_rated_current = 2000, motor_peak_current = 2000, motor_overload_time = 1000;
short commutation_founded = 0, commutation_mode = 0, commutation_time = 1000;
int loop_counter_c = 0, loop_counter_v = 1, loop_counter_p = 2, current_loop_ready = 0, velocity_loop_ready = 0, position_loop_ready = 0;
int phase_view;
uint16_t  poles_num = 2, motor_code = 0;
Encoder_Type_e feedback_type = Default;
int feedback_resolution = 4000;
short over_voltage, under_voltage, chop_voltage, over_temperature;
short tamagawa_offset = 0, tamagawa_dir = 1;
short Driver_Ready = 0;
int drv8301_error = 0;

// unsigned short enc_z.count = 0, enc_z.count_back = 0, enc_z.count_c = 0, enc_z.first = 0, enc_z.trig = 0, enc_z.counting_error = 0;
// short enc_z.diff = 0, enc_z.diff_back = 0;
// int enc_z.pos = 0, enc_z.pos_back = 0, enc_z.pos_offset = 0, enc_z.pos_diff = 0, enc_z.pos_diff_back = 0;


ENC_Z enc_z={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
Hall_t hall = {0, 0, 0, 0, 0, 0}; // 霍尔传感器结构体

short hall_count = 0, hall_get_position = 0, start_calibrate_hall_phase = 0;
int encoder_direction_temp = 0, encoder_direction_temp_b = 0;
short hall_phase[8], ENC_Z_Offset = 2680, hall_phase_offset = 0, ENC_Z_Phase = 0, ENC_Z_Phase_B = 0, ENC_Z_Phase_Err = 0;
int hall_position = 0, hall_position_b = 0;
short encoder_offset_diff = 0, hall_phase_offset_diff = 0;
union error_uint32_t Error_State;
union can_int32_t Scop_Buffer[4][512] = {0};
short Scop_Period = 31, Scop_Period_counter = 0, Scop_Buffer_point = 0, Scop_Start = 0, Scop_Data_Ready = 0, Scop_Send_point = 0;
short Scop_Point[4];

int led_blink_counter = 0, led_blink_period = 1000;

union can_int32_t Scop_Chanel[4];
unsigned char Scop_Send_Buf[20];

u16 store_parameter = 0;

Motor_t motor = {
	.motor_timer = &htim1,
	.PWM1_Duty = TIM_1_8_PERIOD_CLOCKS / 2,
	.PWM2_Duty = TIM_1_8_PERIOD_CLOCKS / 2,
	.PWM3_Duty = TIM_1_8_PERIOD_CLOCKS / 2,
	.control_deadline = TIM_1_8_PERIOD_CLOCKS,
	.PhaseU_current = 0,
	.PhaseV_current = 0,
	.PhaseW_current = 0,

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
	.pll_ki = 0.0f	 // [(rad/s^2) / rad]

};

/* Private constant data -----------------------------------------------------*/
static const int one_by_sqrt3 = 577;
static const int sqrt3_by_2 = 866;

/* Function implementations --------------------------------------------------*/

// Initalises the low level motor control and then starts the motor control threads
void init_motor_control(void)
{
	delay_ms(10);
	// if(vbus_voltage<140)
	//	Error_State=Error_State|0x0010;

	// if(Error_State*0x10==0)
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
			Error_State.bits.ENC_error = 1;
		}
		else
		{
			tamagawa_angle_b = tamagawa_angle;
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
 * @brief 启动svpwm
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
 * @brief 停止svpwm
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

	int amp_out_volt = ONE_ADC_VOLTAGE * ADCValue;
	int shunt_volt = amp_out_volt / amp_gain;
	int current = (shunt_volt * 100) / motor.shunt_conductance; // unit mA
	return current;
}

void Calibrate_ADC_Offset(void)
{
	delay_ms(200);
	ADC_Offset[0] = ADCValue[0];
	ADC_Offset[1] = ADCValue[1];
	if ((ADC_Offset[0] < 1800) || (ADC_Offset[0] > 2200))
		Error_State.bits.ADC_error = 1;
	if ((ADC_Offset[1] < 1800) || (ADC_Offset[1] > 2200))
		Error_State.bits.ADC_error = 1;
}
int tA, tB, tC;
void queue_modulation_timings(Motor_t *motors, int mod_alpha, int mod_beta)
{

	SVM(mod_alpha, mod_beta, &tA, &tB, &tC);
	motors->PWM1_Duty = (tC * TIM_1_8_PERIOD_CLOCKS) / 1000;
	motors->PWM2_Duty = (tB * TIM_1_8_PERIOD_CLOCKS) / 1000;
	motors->PWM3_Duty = (tA * TIM_1_8_PERIOD_CLOCKS) / 1000;
}

int32_t get_electric_phase(int commutation_current)
{
	int32_t phase_offset, current_step, i;

	Iq_demand = commutation_current;
	delay_ms(commutation_time);
	phase_offset = motor.encoder_state;
	delay_ms(commutation_time);
	return phase_offset;
}

void find_commutation(void)
{
	static int my_p0, my_p1, my_dir;
	switch (commutation_mode)
	{
	case 0:
		switch (feedback_type)
		{
		case Default:
		case Tamagawa:
		case Unknown_8:
			phase_dir = 1; // must set back to the default value
			start_pwm(&htim1);
			motor_on = 1;
			motor.phase = 0;
			my_p0 = get_electric_phase(commutation_current);
			motor.phase = M_PI / 2;
			my_p1 = get_electric_phase(commutation_current);
			motor.phase = 0;
			my_p0 = get_electric_phase(commutation_current);

			Iq_demand = 0;

			if (my_p1 >= my_p0)
			{
				phase_dir = 1;
				if ((feedback_resolution / (5 * poles_num)) < (my_p1 - my_p0) && (my_p1 - my_p0) < (feedback_resolution / (3 * poles_num)))
				{
					commutation_founded = 1;
					motor.encoder_state = 0;
					motor.encoder_offset = my_p0 % feedback_resolution;
				}
			}
			else
			{
				phase_dir = -1;
				if ((feedback_resolution / (5 * poles_num)) < (my_p0 - my_p1) && (my_p1 - my_p0) < (feedback_resolution / (3 * poles_num)))
				{
					commutation_founded = 1;
					motor.encoder_state = 0;
					motor.encoder_offset = feedback_resolution - my_p0 % feedback_resolution;
				}
			}
			if (feedback_type == Tamagawa)
			{
				if (commutation_founded == 1)
				{
					tamagawa_dir = phase_dir;
					tamagawa_offset = motor.encoder_offset;
				}
			}
			if (commutation_founded == 0)
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
		case 1:
			motor.encoder_state = 0;
			enc_z.count = 0;
			enc_z.count_back = 0;
			enc_z.first = 0;
			commutation_founded = 1;
			break;
		case 4:
			phase_dir = tamagawa_dir;
			motor.encoder_state = tamagawa_angle;
			motor.encoder_offset = tamagawa_offset;
			commutation_founded = 1;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	// motors[0].rotor.encoder_offset=get_electric_phase(commutation_current)+feedback_resolution/(4*poles_num);
}
int rad_of_round = 2 * M_PI * 2;
void update_motor(Motor_t *motors)
{
	int16_t delta_enc;
	//@TODO stick parameter into struct
	rad_of_round = 2 * M_PI * poles_num;
	// update internal encoder state
	// int16_t delta_enc = (int16_t)rotor->encoder_timer->Instance->CNT - (int16_t)rotor->encoder_state;
	// rotor->encoder_state += (int32_t)delta_enc;
	switch (feedback_type)
	{
	case 0:
	case Default:
	case Unknown_2:
		delta_enc = (int16_t)motors->encoder_timer->Instance->CNT - (int16_t)motors->encoder_state;
		motors->encoder_state += (int32_t)delta_enc;
		break;
	case Tamagawa:
		delta_enc = tamagawa_angle - tamagawa_angle_b;
		if (delta_enc < (-feedback_resolution / 2))
			delta_enc += feedback_resolution;
		if (delta_enc > (feedback_resolution / 2))
			delta_enc -= feedback_resolution;
		tamagawa_angle_b = tamagawa_angle;
		motors->encoder_state += (int32_t)delta_enc;
		break;
	case 5:
		delta_enc = tamagawa_angle - tamagawa_angle_b;
		if (delta_enc < (-feedback_resolution / 2))
			delta_enc += feedback_resolution;
		if (delta_enc > (feedback_resolution / 2))
			delta_enc -= feedback_resolution;
		tamagawa_angle_b = tamagawa_angle;
		motors->encoder_state += (int32_t)delta_enc;
		break;
	case Unknown_8:

		break;
	default:
		break;
	}

	// hall_u=HAL_GPIO_ReadPin(HALL_U_GPIO_Port,HALL_U_Pin);
	// hall.v=HAL_GPIO_ReadPin(HALL_V_GPIO_Port,HALL_V_Pin);
	// hall.w=HAL_GPIO_ReadPin(HALL_W_GPIO_Port,HALL_W_Pin);

	// hall.state_back=hall.state;
	// hall.state=hall_u+(hall.v<<1)+(hall.w<<2);

	hall.state_back = hall.state;
	hall.state = hall.u + (hall.v << 1) + (hall.w << 2);

	int ph;
	// compute electrical phase
	int32_t enc_state_mod = motors->encoder_state % feedback_resolution;
	if (phase_dir == 1)
		ph = M_PI / 2 + (rad_of_round * (enc_state_mod - motors->encoder_offset)) / feedback_resolution;
	else
		ph = M_PI / 2 + (rad_of_round * ((feedback_resolution - enc_state_mod) - motors->encoder_offset)) / feedback_resolution;

	//ph = fmod(ph, 2 * M_PI);
	ph = ph % (2 * M_PI);





	// if(ph<0)
	//	ph+=2*M_PI;

	switch (operation_mode)
	{
	case 1:
	case 3:
	case 4:
	case 2:
	case 5:
	case 7:
		if (commutation_founded)
			motors->phase = ph;
		break;
	case 14:
	case 13:
	case 12:
	case 11:
	case 17:
		// hall
		if (hall_phase[hall.state] > hall_phase[hall.state_back])
		{
			if ((hall_phase[hall.state] - hall_phase[hall.state_back]) < M_PI)
			{
				hall_position += 1;
				// ph=hall_phase[hall.state]-hall_phase_offset;
			}
			else
			{
				hall_position -= 1;
				// ph=hall_phase[hall.state_back]-hall_phase_offset;
			}
		}
		else if (hall_phase[hall.state] < hall_phase[hall.state_back])
		{
			if ((hall_phase[hall.state_back] - hall_phase[hall.state]) < M_PI)
			{
				hall_position -= 1;
				// ph=hall_phase[hall.state_back]-hall_phase_offset;
			}
			else
			{
				hall_position += 1;
				// ph=hall_phase[hall.state]-hall_phase_offset;
			}
		}
		if (speed_demand > 0)
			ph = hall_phase[hall.state] + hall_phase_offset + 862;
		else
			ph = hall_phase[hall.state] + hall_phase_offset + 862;
		ph = ph % (2 * M_PI);
		if (ph < 0)
			ph += 2 * M_PI;
		motors->phase = ph;
		break;
	case 0:
		if (hall_get_position)
		{
			get_hall_edge_phase(&motor);
		}
		break;
	default:
		break;
	}

	/*
	Align the electrical angle by hall_u edge.
	*/

	if ((commutation_founded == 1) && (commutation_mode == 2))
	{
		// enc_z.first=1;
		if ((hall.state == 4) && (hall.state_back == 5))
		{
			if (hall_phase_dir == 1)
				hall_phase_offset_diff = ((hall_phase[hall.state] + M_PI / 2) - motors->phase);
			else
				hall_phase_offset_diff = ((hall_phase[hall.state_back] + M_PI / 2) - motors->phase);

			hall_phase_offset_diff = hall_phase_offset_diff % (2 * M_PI);
			encoder_offset_diff = hall_phase_offset_diff * feedback_resolution / (M_PI * poles_num * 2);
			if ((commutation_founded == 1) && (commutation_mode == 2))
				if (enc_z.first == 0)
				{
					motors->encoder_offset -= encoder_offset_diff;
					enc_z.first = 1;
				}
		}
		if ((hall.state == 5) && (hall.state_back == 4))
		{
			if (hall_phase_dir == 1)
				hall_phase_offset_diff = ((hall_phase[hall.state_back] + M_PI / 2) - motors->phase);
			else
				hall_phase_offset_diff = ((hall_phase[hall.state] + M_PI / 2) - motors->phase);

			hall_phase_offset_diff = hall_phase_offset_diff % (2 * M_PI);
			encoder_offset_diff = hall_phase_offset_diff * feedback_resolution / (M_PI * poles_num * 2);
			if ((commutation_founded == 1) && (commutation_mode == 2))
				if (enc_z.first == 0)
				{
					motors->encoder_offset -= encoder_offset_diff;
					enc_z.first = 1;
				}
		}
	}
}

void get_hall_edge_phase(Motor_t *motors)
{
	if (hall_get_position == 1) // initial value and start to get hall and Z signal
	{
		motors->encoder_state = 0;
		motors->encoder_timer->Instance->CNT = 0;
		enc_z.count = 0;
		enc_z.count_back = 0;
		enc_z.first = 0;
		ENC_Z_Phase = 6800; // set bigger than 6280
		hall_count = 0;
		hall_get_position = 2;
	}
	if (hall.state != hall.state_back)
	{
		hall_phase[hall.state] = motors->phase;
		hall_count++;
		encoder_direction_temp_b = encoder_direction_temp;
		encoder_direction_temp = motor.encoder_state;
	}
	if (hall_get_position == 2)
	{
		if (hall_count > 3) // check encoder direction
		{
			if (encoder_direction_temp > encoder_direction_temp_b) // positive direction
			{
				// phase_dir_B=phase_dir;
				phase_dir = 1;
				// if(phase_dir_B==phase_dir) // same as last time , go on
				//{
				hall_get_position = 3;
				//}
				// else  // different as last time , retry
				//{
				//	hall_get_position=1;
				//}
			}
			else // negative direction
			{
				// phase_dir_B=phase_dir;
				phase_dir = -1;
				// if(phase_dir_B==phase_dir) // same as last time , go on
				//{
				//	hall_get_position=3;
				// }
				// else // different as last time , retry
				//{
				hall_get_position = 1;
				//}
			}
		}
	}
	if (hall_get_position == 3)
	{
		if (hall_count > 18)
		{
			hall_get_position = 4;
			ENC_Z_Phase = 6800;
		}
	}
	if (hall_get_position == 4)
	{
		if (ENC_Z_Phase != 6800)
		{
			ENC_Z_Phase_B = ENC_Z_Phase;
			hall_get_position = 0;
		}
	}
}

void Process_Scop_Data(void)
{
	int i, j;
	Scop_Period_counter++;
	if (Scop_Buffer_point < 512)
	{
		if (Scop_Period_counter > Scop_Period)
		{
			/*
			for(i=0;i<2;i++)
			{
				Scop_Buffer[i][Scop_Buffer_point]=(*Modbus_Output_Reg[Scop_Point[i]]<<16)+(*Modbus_Output_Reg[Scop_Point[i+1]]);
			}
			*/
			Scop_Chanel[0].all = real_speed_filter;
			Scop_Chanel[1].all = pos_actual;
			Scop_Chanel[2].all = real_speed_filter;
			Scop_Chanel[3].all = real_speed_filter;

			// Scop_Buffer[0][Scop_Buffer_point].all=mod_q*10;
			// Scop_Buffer[1][Scop_Buffer_point].all=Iq_real;
			Scop_Buffer[0][Scop_Buffer_point].all = Iq_demand;
			Scop_Buffer[1][Scop_Buffer_point].all = Iq_real;
			Scop_Buffer[2][Scop_Buffer_point].all = speed_demand;
			Scop_Buffer[3][Scop_Buffer_point].all = real_speed_filter;

			// Scop_Buffer[0][Scop_Buffer_point].all=motors[0].current_meas.phB;//real_speed_filter;
			// Scop_Buffer[1][Scop_Buffer_point].all=motors[0].current_meas.phC;//Iq_real;

			Scop_Period_counter = 0;
			Scop_Buffer_point++;
		}
	}
	else
	{
		Scop_Buffer_point = 0;
		Scop_Data_Ready = 1;
		Scop_Start = 0;
	}
}

void Send_Scop_Data(void)
{
	int i;

	// if((Scop_Data_Ready==0)&&(Scop_Start==0))
	// if(auto_reverse_status==1)
	// Scop_Start=1;

	if (Scop_Data_Ready)
	{
		Scop_Send_point++;
		if (Scop_Send_point < 512)
		{
			Scop_Send_Buf[0] = 0x03;
			Scop_Send_Buf[1] = 0xfc;
			for (i = 0; i < 4; i++)
			{
				Scop_Send_Buf[2 + i * 4] = Scop_Buffer[i][Scop_Send_point].byte.byte_0;
				Scop_Send_Buf[3 + i * 4] = Scop_Buffer[i][Scop_Send_point].byte.byte_1;
				Scop_Send_Buf[4 + i * 4] = Scop_Buffer[i][Scop_Send_point].byte.byte_2;
				Scop_Send_Buf[5 + i * 4] = Scop_Buffer[i][Scop_Send_point].byte.byte_3;
			}
			Scop_Send_Buf[18] = 0xfc;
			Scop_Send_Buf[19] = 0x03;

			Modbus_Solve_PutString(Scop_Send_Buf, 20);
		}
		else
		{
			Scop_Data_Ready = 0;
			Scop_Send_point = 0;
			// Scop_Start=1;
		}
	}
}

void calibrate_hall_phase(void)
{
	if (start_calibrate_hall_phase == 1)
	{
		auto_reverse_p_time = 0;
		auto_reverse_n_time = 0;
		target_Iq = commutation_current;
		target_speed = 1;
		operation_mode = 0;
		control_word.all = 0x0f;
		delay_ms(1000);
		hall_get_position = 1;
		start_calibrate_hall_phase = 2;
	}
	if (start_calibrate_hall_phase == 2)
	{
		if (hall_get_position == 0)
		{
			target_Iq = 0;
			target_speed = 0;
			operation_mode = 0;
			control_word.all = 0x06;
			start_calibrate_hall_phase = 3;
		}
	}
}

u32 calibrate_timechk;
void calibrate_tamagawa_encoder(void)
{
	if (set_tamagawa_zero == 2)
	{
		auto_reverse_p_time = 0;
		auto_reverse_n_time = 0;
		target_Iq = commutation_current;
		target_speed = 0;
		phase_dir = 1;
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
			target_Iq = 0;
			target_speed = 0;
			operation_mode = 2;
			Tamagawa_count_temp = 0;
			control_word.all = 0x86;
			set_tamagawa_zero = 0;
		}
	}
}
