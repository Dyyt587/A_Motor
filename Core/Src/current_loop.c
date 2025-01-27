#include <stm32g0xx_hal.h> //Sets up the correct chip specifc defines required by arm_math
// #define ARM_MATH_CM4
// #include <arm_math.h>

#include <mcpwm.h>

#include <stdlib.h>
#include <math.h>
#include "apid.h"
// #include <cmsis_os.h>

#include <main.h>
#include <adc.h>
#include <tim.h>
#include <spi.h>
#include <utils.h>
#include "delay.h"
#include "perf_counter.h"

/* Private constant data -----------------------------------------------------*/
static const int one_by_sqrt3 = 577;
static const int sqrt3_by_2 = 866;

uint64_t nCycleUsed = 0;
int vfactor = 1;
int Ialpha, Ibeta, Ierr_d, Ierr_q, Vd, Vd_filter, Vq, Vq_filter, V_current_control_integral_d = 0, V_current_control_integral_q = 0, vfactor, mod_d, mod_q, mod_scalefactor, mod_alpha, mod_beta, mod_alpha_filter, mod_beta_filter;
int Vq_out_limit = 700, Vd_out_limit = 700, kci_sum_limit = 100;
short kcp = 20, kci = 0;
short current_in_lpf_a = 1000, current_out_lpf_a = 1000;
int check_current_overshot_p = 0, check_current_overshot_n = 0;
int Driver_IIt_Real = 0, Driver_IIt_Current, Driver_IIt_Real_DC = 0, Driver_IIt_Current_DC;
short Driver_IIt_Filter, Driver_IIt_Filter_DC;

extern apid_t apidd;
extern apid_t apidq;
void Current_loop(Motor_t *motors, int Id_des, int Iq_des)
{

	Id_des = -Id_des * phase_dir;
	Iq_des = -Iq_des * phase_dir;

	motors->PhaseU_current_ma = phase_current_from_adcval(ADCValue[0] - ADC_Offset[0]);

	motors->PhaseW_current_ma = phase_current_from_adcval(ADCValue[1] - ADC_Offset[1]);

	motors->PhaseV_current_ma = -motors->PhaseU_current_ma - motors->PhaseW_current_ma;

	// Clarke transform
	// Ialpha = motors->PhaseU_current_ma*1000;
	// Ibeta =  (motors->PhaseV_current_ma - motors->PhaseW_current_ma)*577;

	clarke_calc_3(&svpwm, motors->PhaseU_current_ma, motors->PhaseV_current_ma, motors->PhaseW_current_ma);

	// Park transform
	int c = arm_cos_f32(motors->phase);
	int s = arm_sin_f32(motors->phase);
	// Id = (c * Ialpha + s * Ibeta) / 16384000;
	// Iq = (c * Ibeta - s * Ialpha) / 16384000;

	park_calc(&svpwm, svpwm.Alpha, svpwm.Beta, motors->phase);
	// Iq_real = -Iq * phase_dir;
	// Id_real = -Id * phase_dir;
	Iq_real = -svpwm.Qs * phase_dir;
	Id_real = -svpwm.Ds * phase_dir;

	if (motor_on)
	{

		 __cycleof__("my algorithm", {nCycleUsed = _;})
		{

			APID_Set_Target(&apidd, Id_des);
			APID_Set_Target(&apidq, Iq_des);

			APID_Set_Present(&apidd, svpwm.Ds);
			APID_Set_Present(&apidq, svpwm.Qs);
			// Current error
			// Ierr_d = Id_des - svpwm.Ds;
			// Ierr_q = Iq_des - svpwm.Qs;

			//@TODO look into feed forward terms (esp omega, since PI pole maps to RL tau)
			//@TODO current limit
			// Apply PI control
			// Vd = (V_current_control_integral_d / 10 + (Ierr_d * kcp)) / 1000;
			// Vq = (V_current_control_integral_q / 10 + (Ierr_q * kcp)) / 1000;

			APID_Hander(&apidd, 1);
			APID_Hander(&apidq, 1);

			Vd = apidd.parameter.out / 1000;
			Vq = apidq.parameter.out / 1000;
			Vq_filter = Low_pass_filter_1(current_out_lpf_a, Vq, Vq_filter);
			Vd_filter = Low_pass_filter_1(current_out_lpf_a, Vd, Vd_filter);

			mod_d = vfactor * Vd_filter;
			mod_q = vfactor * Vq_filter;

			// Vector modulation saturation, lock integrator if saturated
			//@TODO make maximum modulation configurable (currently 90%)
			//			tt1=SysTick->VAL;
			//			//mod_scalefactor = 800 * sqrt3_by_2 /sqrt(mod_d*mod_d + mod_q*mod_q);
			//			//mod_scalefactor = 64000000  /(mod_q*mod_q);
			//			tt2=SysTick->VAL;
			//			//tt3=tt2-tt1;
			//			tt3=tt1-tt2;
			//			if(tt3>tt4)
			//				tt4=tt3;
			// t2=t1;

			// Compute estimated bus current
			// *IbusEst = mod_d * Id + mod_q * Iq;

			// Inverse park transform
			mod_alpha = (c * mod_d - s * mod_q) / 16384;
			mod_beta = (c * mod_q + s * mod_d) / 16384;

			mod_alpha = mod_alpha;
			mod_beta = mod_beta;

			// Apply SVM
			queue_modulation_timings(motors, mod_alpha, mod_beta);

			// 测试代码
			if (Iq_des > 0)
			{
				if (Ierr_q < check_current_overshot_p)
					check_current_overshot_p = Ierr_q;
			}
			if (Iq_des < 0)
			{
				if (Ierr_q > check_current_overshot_n)
					check_current_overshot_n = Ierr_q;
			}
		}
	}
}

// Y=A*X+(1-A)*Y0 -> Y=Y0+((X-Y0)*A)/1000; 0<A<1000
int IIt_Remaider = 0, IIt_temp = 0;
int IIt_filter(int A, int X, int Y)
{
	if (X < 0)
		X = -X;
	IIt_temp = (X - Y) * A + IIt_Remaider;
	IIt_Remaider = IIt_temp % 60000;
	return Y + IIt_temp / 60000;
}

int IIt_DC_Remaider = 0, IIt_DC_temp = 0;
int IIt_DC_filter(int A, int X, int Y)
{
	if (X < 0)
		X = -X;
	IIt_DC_temp = (X - Y) * A + IIt_DC_Remaider;
	IIt_DC_Remaider = IIt_DC_temp % 60000;
	return Y + IIt_DC_temp / 60000;
}
