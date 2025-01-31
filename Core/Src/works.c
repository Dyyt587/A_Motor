#include "utils.h"
#include "workchain.h"
#include <mcpwm.h>
#include <tim.h>
#include <amotor_port.h>

int safe_check(wkc_t *wkc)
{
	Motor_t *motors = (Motor_t *)wkc->user_date;

	if (motors->motion.Error_State.all)
	{
		pwm_stop(motors);
		motors->wkc.lic_aprove.bits.motor_on = 0;
	}
	return 0;
}
int update_motor_work_handle(wkc_t *wkc)
{
	Motor_t *motors = (Motor_t *)wkc->user_date;
	send_to_tamagawa();
	update_motor(motors, get_Tamagawa_encoder()); // 更新电角度
	Update_Speed(motors);
	pos_actual = motors->encoder_state - pos_offest;

	motors->PhaseU_current_ma = phase_current_from_adcval(ADCValue[0] - ADC_Offset[0]);

	motors->PhaseW_current_ma = phase_current_from_adcval(ADCValue[1] - ADC_Offset[1]);

	motors->PhaseV_current_ma = -motors->PhaseU_current_ma - motors->PhaseW_current_ma;

	return 0;
}
int Current_loop_work_handle(wkc_t *wkc)
{
		Motor_t * motors = (Motor_t *)wkc->user_date;

	Current_loop((Motor_t *)wkc->user_date, motors->motion.Id_demand, motors->motion.Iq_demand);
	return 0;
}

int Velocity_loop_work_handle(wkc_t *wkc) // 电流环控制
{
	Motor_t * motors = (Motor_t *)wkc->user_date;
	Velocity_loop((Motor_t *)wkc->user_date, motors->motion.speed_demand);
	return 0;
}
int Position_loop_work_handle(wkc_t *wkc)
{
		Motor_t * motors = (Motor_t *)wkc->user_date;

	// Position_Loop((Motor_t *)wkc->user_date, 0);
	Position_Loop((Motor_t *)wkc->user_date,  motors->motion.position_demand);
	return 0;
}
int Apply_SVM_PWM_work_handle(wkc_t *wkc)
{
	// Apply SVM
	queue_modulation_timings((Motor_t *)wkc->user_date);
	return 0;
}
//int32_t get_electric_phase(int commutation_current)
//{
//	int32_t phase_offset, current_step, i;

//	motors->motion.Iq_demand = commutation_current;
//	delay_ms(motor.motion.commutation_time);
//	phase_offset = motor.encoder_state;
//	delay_ms(motor.motion.commutation_time);
//	return phase_offset;
//}

int Find_Commutation_Jitter_work_handle(wkc_t *wkc)
{
	Motor_t *motors = (Motor_t *)wkc->user_date;

	static int step = 0;
	static int cnt = 0;
	static int my_p0, my_p1 = 0;
	static int64_t start_time_ms = 0;
	// static int64_t start_time_ms = 0;
	static int phase_offset;
	switch (step)
	{
	case 0: // init
		// 修改许可域,启动电机,记录时间
		motors->wkc.lic_aprove.bits.torque_mode = 1;
		// motors->wkc.lic_aprove.bits.velocity_mode = 0;
		// motors->wkc.lic_aprove.bits.position_mode = 0;

		motors->param.phase_dir = 1; // must set back to the default value
		pwm_start(motors);
		motors->wkc.lic_aprove.bits.motor_on = 1;

		start_time_ms = get_time_ms();

		motors->phase = 0;
		step = 1;
		break;
	case 1:
		motors->motion.Iq_demand = motors->param.commutation_current;
		// 等待一段时间电机运行
		if (get_time_ms() - start_time_ms > motors->motion.commutation_time)
		{
			start_time_ms = get_time_ms();
			step = 2;
		}

		break;
	case 2:
		phase_offset = motors->encoder_state;
		// 等待一段时间防止出问题
		if (get_time_ms() - start_time_ms > motors->motion.commutation_time)
		{
			// start_time_ms = get_time_ms();
			step = 3;
		}
		break;
	case 3:
		if (++cnt == 3) // 3: 0  pi/2  0//第一次0消抖，后面两次记录数据
		{
			/*完成角度获取*/
			motors->motion.Iq_demand = 0;
			step = 4;
			break;
		}
		if (motors->phase == 0)
		{
			my_p0 = phase_offset;
			motors->phase = PI / 2;
			step = 1; // 按照PI / 2再来一次
		}
		else
		{
			my_p1 = phase_offset;
			motors->phase = 0;
			step = 1; // 按照PI / 2再来一次
		}
		break;
	case 4:
		// 计算参数
		if (my_p1 >= my_p0)
		{
			motors->param.phase_dir = 1;
			if ((motors->motion.feedback_resolution / (5 * motors->param.poles_num)) < (my_p1 - my_p0) && (my_p1 - my_p0) < (motors->motion.feedback_resolution / (3 * motors->param.poles_num)))
			{
				motors->wkc.lic_aprove.bits.commutation_founded = 1;
				motors->encoder_state = 0;
				motors->encoder_offset = my_p0 % motors->motion.feedback_resolution;
			}
		}
		else
		{
			motors->param.phase_dir = -1;
			if ((motors->motion.feedback_resolution / (5 * motors->param.poles_num)) < (my_p0 - my_p1) && (my_p1 - my_p0) < (motors->motion.feedback_resolution / (3 * motors->param.poles_num)))
			{
				motors->wkc.lic_aprove.bits.commutation_founded = 1;
				motors->encoder_state = 0;
				motors->encoder_offset = motors->motion.feedback_resolution - my_p0 % motors->motion.feedback_resolution;
			}
		}
		if (motors->feedback_type == Tamagawa)
		{
			if (motors->wkc.lic_aprove.bits.commutation_founded == 1)
			{
				tamagawa_dir = motors->param.phase_dir;
				tamagawa_offset = motors->encoder_offset;
			}
		}
		if (motors->wkc.lic_aprove.bits.commutation_founded == 0)
		{
			pwm_stop(motors);
			motors->wkc.lic_aprove.bits.motor_on = 0;
		}

		step = 0; // 归于开始
		cnt = 0;
		start_time_ms = 0;
		// motors->wkc.lic_aprove.bits.torque_mode = 1;
		// motors->wkc.lic_aprove.bits.velocity_mode = 1;
		// motors->wkc.lic_aprove.bits.position_mode = 1;
		wkc_work_del(wkc, wkc->current_work);
		break;
	}

	return 0;
}
int Find_Commutation_Saved_work_handle(wkc_t *wkc)
{
	Motor_t *motors = (Motor_t *)wkc->user_date;

	motors->param.phase_dir = tamagawa_dir;
	motors->encoder_state = tamagawa_angle;
	motors->encoder_offset = tamagawa_offset;
	motors->wkc.lic_aprove.bits.commutation_founded = 1;
	pwm_start(motors);
	motors->wkc.lic_aprove.bits.motor_on = 1;
	wkc_work_del(wkc, wkc->current_work);

	return 0;
}
int Measure_Resistance_work_handle(wkc_t *wkc)
{
	Motor_t *motors = (Motor_t *)wkc->user_date;
	static int step = 0;
	static int64_t start_time_ms = 0;
	static int Vd = 100;
	static int cnt = 0;
	static svpwm_t svpwm;
	volatile int64_t tmp;
	switch (step)
	{
	case 0:
		// 修改许可域,启动电机,记录时间
		motors->wkc.lic_aprove.bits.torque_mode = 0;
		motors->wkc.lic_aprove.bits.velocity_mode = 0;
		motors->wkc.lic_aprove.bits.position_mode = 0;
		start_time_ms = get_time_ms();
		step = 1;
	case 1:
		ipark_calc(&motors->svpwm, Vd, 0, 0);
		if (get_time_ms() - start_time_ms > 100)
		{
			start_time_ms = get_time_ms();
			// 读取电流进行
			clarke_calc_2(&svpwm, motors->PhaseU_current_ma, motors->PhaseW_current_ma);
			int64_t i = (svpwm.Alpha * svpwm.Alpha + svpwm.Beta * svpwm.Beta);
			if (i > 600 * 600)
			{
				Vd -= 10;
			}
			else if (i < 500 * 500)
			{
				Vd += 10;
			}
			else
			{
				// 电流合适，计算电压
				int64_t u = motors->motion.vbus_voltage * Vd / 1000; // 120 -->12.0V
				//					int v_U = motors->PWM1_Duty*motors->motion.vbus_voltage;
				//					int v_W = motors->PWM2_Duty*motors->motion.vbus_voltage;
				//					int v_V = motors->PWM3_Duty*motors->motion.vbus_voltage;
				// TODO: 多次平均，低通滤波
				//				tmp  = u*100000;
				//				tmp/=i;
				motors->phase_resistor_moh += u * 100000000 / i;
				// motors->phase_resistor_moh = motors->phase_resistor_moh*1000;
				cnt++;
			}
			// override
			// Vd = 100;
			if (cnt > 200)
			{
				step = 2;
				motors->phase_resistor_moh /= cnt;
				cnt = 1;
			}
			else
			{
				step = 1;
			}
		}
		/* code */
		break;
	case 2:
		step = 0;
		motors->wkc.lic_aprove.bits.torque_mode = 1;
		motors->wkc.lic_aprove.bits.velocity_mode = 1;
		motors->wkc.lic_aprove.bits.position_mode = 1;
		wkc_work_del(wkc, wkc->current_work);

		/* code */
		break;

	default:
		break;
	}

	return 0;
}
wkc_work_t update_motor_work = {
	.name = "update_motor",
	.handle = update_motor_work_handle,
	.licence = {
		.bits.drv_ready = 1,
		.bits.drv_init = 1,
	},
	.trig_level = 0, // 触发等级，每次触发
	.trig_cnt = 0,
	.next = 0,
	.user_date = &motor,
};
wkc_work_t Current_loop_work = {
	.name = "Current_loop",
	.handle = Current_loop_work_handle,
	.licence = {
		.bits.drv_ready = 1,
		.bits.drv_init = 1,
		.bits.torque_mode = 1,

	},
	.trig_level = 0, // 触发等级，每次触发
	.trig_cnt = 0,
	.next = 0,
	.user_date = &motor,
};
wkc_work_t Velocity_loop_work = {
	.name = "Velocity_loop",
	.handle = Velocity_loop_work_handle,
	.licence = {
		.bits.drv_ready = 1,
		.bits.drv_init = 1,
		.bits.commutation_founded = 1,
		.bits.motor_on = 1,
		.bits.velocity_mode = 1,
	},
	.trig_level = 3, // 触发等级，每4次触发
	.trig_cnt = 1,	 // 防止第一次超时
	.next = 0,
	.user_date = &motor,
};
wkc_work_t Position_loop_work = {
	.name = "Position_loop",
	.handle = Position_loop_work_handle,
	.licence = {
		.bits.drv_ready = 1,
		.bits.drv_init = 1,
		.bits.commutation_founded = 1,
		.bits.motor_on = 1,
		.bits.position_mode = 1,
	},
	.trig_level = 7, // 触发等级，每8次触发
	.trig_cnt = 2,
	.next = 0,
	.user_date = &motor,
};
wkc_work_t Apply_SVM_PWM_work = {
	.name = "Apply_SVM_PWM",
	.handle = Apply_SVM_PWM_work_handle,
	.licence = {
		.bits.drv_ready = 1,
		.bits.drv_init = 1,
	},
	.trig_level = 0, // 触发等级，每次触发
	.trig_cnt = 0,
	.next = 0,
	.user_date = &motor,
};

// 该工作只会调用一次
wkc_work_t Find_Commutation_Jitter_work = {
	.name = "Find_Commutation_Jitter",
	.handle = Find_Commutation_Jitter_work_handle,
	.licence = {
		.bits.drv_ready = 1,
		.bits.drv_init = 1,
	},
	.trig_level = 0, // 触发等级，每次触发
	.trig_cnt = 0,
	.next = 0,
	.user_date = &motor,
};
wkc_work_t Find_Commutation_Saved_work = {
	.name = "Find_Commutation_Saved",
	.handle = Find_Commutation_Saved_work_handle,
	.licence = {
		.bits.drv_ready = 1,
		.bits.drv_init = 1,
	},
	.trig_level = 0, // 触发等级，每次触发
	.trig_cnt = 0,
	.next = 0,
	.user_date = &motor,
};
wkc_work_t Measure_Resistance_work = {
	.name = "Measure_Resistance",
	.handle = Measure_Resistance_work_handle,
	.licence = {
		.bits.drv_ready = 1,
		.bits.drv_init = 1,
		.bits.commutation_founded = 1,
	},
	.trig_level = 0, // 触发等级，每次触发
	.trig_cnt = 0,
	.next = 0,
	.user_date = &motor,
};
wkc_work_t Safe_Check_work = {
	.name = "Safe_Check",
	.handle = safe_check,
	.licence = {
		.bits.drv_ready = 1,
		.bits.drv_init = 1,
	},
	.trig_level = 5, // 触发等级，每6次触发
	.trig_cnt = 0,
	.next = 0,
	.user_date = &motor,
};
void works_init(void)
{
	wkc_init(&motor.wkc);
	motor.wkc.user_date = &motor;

	wkc_work_add(&motor.wkc, &Safe_Check_work);
	// wkc_work_add(&motor.wkc, &Measure_Resistance_work);

	// 注意，根据实际情况，动态加载工作可能不会被调用
	 wkc_work_add(&motor.wkc, &Find_Commutation_Jitter_work);
	//wkc_work_add(&motor.wkc, &Find_Commutation_Saved_work);
	wkc_work_add(&motor.wkc, &update_motor_work);
	wkc_work_add(&motor.wkc, &Current_loop_work);
	wkc_work_add(&motor.wkc, &Velocity_loop_work);
	wkc_work_add(&motor.wkc, &Position_loop_work);
	wkc_work_add(&motor.wkc, &Apply_SVM_PWM_work);
}