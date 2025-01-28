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


int Vq_out_limit = 700, Vd_out_limit = 700, kci_sum_limit = 100;
short kcp = 20, kci = 0;
// short current_in_lpf_a = 1000, current_out_lpf_a = 1000;
// int check_current_overshot_p = 0, check_current_overshot_n = 0;
// int Driver_IIt_Real = 0, Driver_IIt_Current, Driver_IIt_Real_DC = 0, Driver_IIt_Current_DC;
// short Driver_IIt_Filter, Driver_IIt_Filter_DC;

//extern apid_t apidd;
//extern apid_t apidq;
void Current_loop(Motor_t *motors, int Id_des, int Iq_des)
{

	Id_des = -Id_des * motors->param.phase_dir;
	Iq_des = -Iq_des * motors->param.phase_dir;

	motors->PhaseU_current_ma = phase_current_from_adcval(ADCValue[0] - ADC_Offset[0]);

	motors->PhaseW_current_ma = phase_current_from_adcval(ADCValue[1] - ADC_Offset[1]);

	motors->PhaseV_current_ma = -motors->PhaseU_current_ma - motors->PhaseW_current_ma;

	// Clarke transform
	clarke_calc_3(&motors->svpwm, motors->PhaseU_current_ma, motors->PhaseV_current_ma, motors->PhaseW_current_ma);

	// Park transform
	park_calc(&motors->svpwm, motors->svpwm.Alpha, motors->svpwm.Beta, motors->phase);

	Iq_real = -motors->svpwm.Qs * motors->param.phase_dir;
	Id_real = -motors->svpwm.Ds * motors->param.phase_dir;

	if (motor_on)
	{

		 __cycleof__("my algorithm", {nCycleUsed = _;})
		{

			APID_Set_Target(&motors->apidd, Id_des);
			APID_Set_Target(&motors->apidq, Iq_des);

			APID_Set_Present(&motors->apidd, motors->svpwm.Ds);
			APID_Set_Present(&motors->apidq, motors->svpwm.Qs);
			// Current error
			// Ierr_d = Id_des - motors->svpwm.Ds;
			// Ierr_q = Iq_des - motors->svpwm.Qs;

			//@TODO look into feed forward terms (esp omega, since PI pole maps to RL tau)
			//@TODO current limit
			// Apply PI control
			// Vd = (V_current_control_integral_d / 10 + (Ierr_d * kcp)) / 1000;
			// Vq = (V_current_control_integral_q / 10 + (Ierr_q * kcp)) / 1000;

			APID_Hander(&motors->apidd, 1);
			APID_Hander(&motors->apidq, 1);

			int Vd = motors->apidd.parameter.out / 1000;
			int Vq = motors->apidq.parameter.out / 1000;
			int Vq_filter = Vq;// Low_pass_filter_1(current_out_lpf_a, Vq, Vq_filter);
			int Vd_filter = Vd;//Low_pass_filter_1(current_out_lpf_a, Vd, Vd_filter);

			int mod_d = vfactor * Vd_filter;
			int mod_q = vfactor * Vq_filter;

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
			//ipark_calc(&motors->svpwm, mod_d, mod_q, motors->phase);
			 motors->svpwm.Ds = mod_d;
			 motors->svpwm.Qs = mod_q;
			 _ipark_calc(&motors->svpwm);

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
