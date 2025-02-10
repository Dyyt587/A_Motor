#include "amotor_port.h"
#include "mcpwm.h"
#include "tim.h"
typedef struct
{
	TIM_HandleTypeDef *htim;
} motor_user_date_t;
motor_user_date_t motor_user_date = {
	.htim = &htim1,
};

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
	.phase_resistor_moh = 5,  //[S]
	.phase_inductance = 5,	  //[S]
							  //	.encoder_timer = &htim3,
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
		.work_mode = Default_Mode,

	},
	.param = {
		.poles_num = 2, .phase_dir = 1, .pid_id = {600, 30, 0, 100 * 10, 700 * 1000}, .pid_iq = {600, 30, 0, 100 * 10, 700 * 1000},

		.pid_vel = {200, 1, 0, -1, 3000 * 1000},
		.pid_pos = {5, 1, 0, 100, 50000}, // 50000;   //速度限制，单位是0.001转每秒

	},
	.control = {
		.requested_operation_mode = Default_Mode,
	},

	.feedback_type = Default,
	.user_date = &motor_user_date,
};
void amotor_part_init(void)
{
	init_motor_control(&motor);
}

void pwm_U_start(Motor_t *motors)
{
	motor_user_date_t *date = (motor_user_date_t *)motors->user_date;
	TIM_HandleTypeDef *htim = date->htim;

	// Init PWM
	int half_load = TIM_1_8_PERIOD_CLOCKS / 2;
	htim->Instance->CCR3 = half_load;

	// This hardware obfustication layer really is getting on my nerves
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_3);
}

void pwm_V_start(Motor_t *motors)
{
	motor_user_date_t *date = (motor_user_date_t *)motors->user_date;
	TIM_HandleTypeDef *htim = date->htim;

	// Init PWM
	int half_load = TIM_1_8_PERIOD_CLOCKS / 2;
	htim->Instance->CCR2 = half_load;

	// This hardware obfustication layer really is getting on my nerves
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_2);


}

void pwm_W_start(Motor_t *motors)
{
	motor_user_date_t *date = (motor_user_date_t *)motors->user_date;
	TIM_HandleTypeDef *htim = date->htim;

	// Init PWM
	int half_load = TIM_1_8_PERIOD_CLOCKS / 2;
	htim->Instance->CCR1 = half_load;

	// This hardware obfustication layer really is getting on my nerves
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_1);
}

void pwm_start(Motor_t *motors)
{
	motor_user_date_t *date = (motor_user_date_t *)motors->user_date;
	TIM_HandleTypeDef *htim = date->htim;

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

void pwm_U_stop(Motor_t *motors)
{
	motor_user_date_t *date = (motor_user_date_t *)motors->user_date;
	TIM_HandleTypeDef *htim = date->htim;
	HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(htim, TIM_CHANNEL_1);
}
void pwm_V_stop(Motor_t *motors)
{
	motor_user_date_t *date = (motor_user_date_t *)motors->user_date;
	TIM_HandleTypeDef *htim = date->htim;
	HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(htim, TIM_CHANNEL_2);

}
void pwm_W_stop(Motor_t *motors)
{
	motor_user_date_t *date = (motor_user_date_t *)motors->user_date;
	TIM_HandleTypeDef *htim = date->htim;
	HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Stop(htim, TIM_CHANNEL_3);
}
void pwm_stop(Motor_t *motors)
{
	motor_user_date_t *date = (motor_user_date_t *)motors->user_date;
	TIM_HandleTypeDef *htim = date->htim;

	// This hardware obfustication layer really is getting on my nerves
	HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(htim, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Stop(htim, TIM_CHANNEL_3);
}
int tA, tB, tC;
void queue_modulation_timings(Motor_t *motors)
{

	SVM(motors->svpwm.Alpha, motors->svpwm.Beta, &tA, &tB, &tC);

	motors->PWM1_Duty = (tC * TIM_1_8_PERIOD_CLOCKS) / 1000;
	motors->PWM2_Duty = (tB * TIM_1_8_PERIOD_CLOCKS) / 1000;
	motors->PWM3_Duty = (tA * TIM_1_8_PERIOD_CLOCKS) / 1000;

	//  motors->PWM1_Duty = (0 * TIM_1_8_PERIOD_CLOCKS) / 1000;
	//  motors->PWM2_Duty = (1000 * TIM_1_8_PERIOD_CLOCKS) / 1000;
	//  motors->PWM3_Duty = (1000 * TIM_1_8_PERIOD_CLOCKS) / 1000;

	motors->motor_timer->Instance->CCR1 = motors->PWM1_Duty;
	motors->motor_timer->Instance->CCR2 = motors->PWM2_Duty;
	motors->motor_timer->Instance->CCR3 = motors->PWM3_Duty;
}