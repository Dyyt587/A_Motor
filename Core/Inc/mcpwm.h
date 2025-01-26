/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LOW_LEVEL_H
#define __LOW_LEVEL_H

/* Includes ------------------------------------------------------------------*/
//#include <cmsis_os.h>
#include "delay.h"
#include "modbus.h"
#include "parameter.h"
#include "NTC_Calculate.h"
#include "tamagawa.h"	

#define CURRENT_MEAS_PERIOD ( (float)2*TIM_1_8_PERIOD_CLOCKS*(TIM_1_8_RCR+1) / (float)TIM_1_8_CLOCK_HZ )
#define CURRENT_MEAS_HZ ( (float)(TIM_1_8_CLOCK_HZ) / (float)(2*TIM_1_8_PERIOD_CLOCKS*(TIM_1_8_RCR+1)) )

#define SOFTWARE_VERSION 2022032001

#define ENC_Z_DIFF_ERROR 20

#ifndef M_PI
#define M_PI 3142
#endif

#define ONE_ADC_VOLTAGE 806  // unit uV
#define AMP_GAIN        11.0

/* Exported types ------------------------------------------------------------*/
typedef struct {
    int phB;
    int phC;
} Iph_BC_t;

typedef struct {
    int current_lim; // [mA]
    int p_gain; // [V/A]
    int i_gain; // [V/As]
    int v_current_control_integral_d; // [V]
    int v_current_control_integral_q; // [V]
} Current_control_t;

typedef struct {
    //osThreadId motor_thread;
    TIM_HandleTypeDef* motor_timer; //PWM timer
    uint16_t PWM1_Duty;
    uint16_t PWM2_Duty;
    uint16_t PWM3_Duty;
    uint16_t control_deadline;
    int PhaseU_current;
    int PhaseV_current;
    int PhaseW_current;
    int shunt_conductance; // 100 means 1 mOh, current sensing resistor
    int phase_inductance; // 1 means 1 uH
    int phase_resistor; // 1 means 1 mOh
    TIM_HandleTypeDef* encoder_timer;
    int16_t encoder_offset;
    int32_t encoder_state;
    int phase;
    float pll_pos;
    float pll_vel;
    float pll_kp;
    float pll_ki;
} Motor_t;

enum Motor_thread_signals {
    M_SIGNAL_PH_CURRENT_MEAS = 1u << 0
};

/* Exported constants --------------------------------------------------------*/
extern short vbus_voltage,device_temperature;
extern Motor_t motor;
extern const int num_motors;
extern int ADCValue[6],ADC_Offset[6],ADC_Value[6];
extern int Id,Iq,Iq_real,Id_real;
extern short commutation_founded,commutation_mode,commutation_time;
extern int commutation_current,motor_rated_current,motor_peak_current,motor_overload_time;
extern short phase_dir,hall_phase_dir,vel_dir;
extern int loop_counter_c,loop_counter_v,loop_counter_p,current_loop_ready,velocity_loop_ready,position_loop_ready;
extern s32 pos_actual,pos_offest;
extern int Iq_demand,Id_demand,target_Iq,target_Id,target_speed,speed_demand,target_position,target_position_b,position_demand;
extern short operation_mode,operation_mode_buff;
extern union Control_uint16_t  control_word,control_word_b;
extern u16 motor_on;
extern short Driver_Ready;
extern int drv8301_error;
extern unsigned short ENC_Z_Count,ENC_Z_Count_B,ENC_Z_Count_C,ENC_Z_First,ENC_Z_Trig,ENC_Counting_Error;
extern short ENC_Z_Diff,ENC_Z_Diff_B;
extern int ENC_Z_Pos,ENC_Z_Pos_B,ENC_Z_Pos_Offset,ENC_Z_Pos_Diff,ENC_Z_Pos_Diff_B;
extern short hall_u,hall_v,hall_w,hall_state,hall_state_b;
extern short hall_phase[],ENC_Z_Offset,hall_phase_offset,ENC_Z_Phase,ENC_Z_Phase_B,ENC_Z_Phase_Err,start_calibrate_hall_phase;
extern int hall_position,hall_position_b;
extern short encoder_offset_diff,hall_phase_offset_diff;
extern uint16_t feedback_type,poles_num,motor_code; 
extern int feedback_resolution;
extern short tamagawa_offset,tamagawa_dir;
extern short over_voltage,under_voltage,chop_voltage,over_temperature;
extern int Driver_IIt_Real,Driver_IIt_Real_RE,Driver_IIt_Current,Driver_IIt_Real_DC,Driver_IIt_Real_DC_RE,Driver_IIt_Current_DC;;
extern short Driver_IIt_Filter,Driver_IIt_Filter_DC;
extern union error_uint32_t  Error_State;
extern union Status_uint16_t status_word;

extern int V_current_control_integral_d,V_current_control_integral_q,mod_q;
extern int Vq_out_limit,Vd_out_limit,kci_sum_limit;
extern short kcp,kci;
extern short current_in_lpf_a,current_out_lpf_a;
extern int real_speed,real_speed_filter,speed_err,kvi_sum,kvi_sum_limit,Ilim;
extern short real_speed_filter_num,low_pass_filter_on;
extern short kvp,kvi;
extern short speed_in_lpf_a,speed_out_lpf_a;
extern short kpp,kpi;
extern int vel_lim,kpi_sum_limit;

extern short position_in_lpf_a,position_out_lpf_a;

extern int display_speed_loop_count,display_speed_update,display_speed,display_encoder_state_b;
extern s32 profile_acce,profile_dece,profile_speed,end_speed,target_speed_now,searching_speed;
extern s32 profile_target_position,profile_target_position_b;
extern s32 home_offest,homing_speed,homing_acce;
extern s8 homing_method;
extern s32 acce_diatance,dece_diatance,distance_diff,decelerating_position,target_pos_now,direction;
extern u8 accelerating,decelerating,positionning;
extern int64_t target_pos_diff;
extern u8 motion_state;
extern s32 auto_p_pos,auto_n_pos;
extern u32 auto_reverse_p_time,auto_reverse_n_time,auto_reverse_time,auto_reverse_status;
extern u16 auto_switch_on;
extern const short Sin_Table[];
extern int pluse_num,pul_dir,gear_factor_a,gear_factor_b;
extern u16 pluse_temp,pluse_temp_b,delta_pulse,delta_pulse_r;

extern short motion_out_lpf_a;

extern union can_int32_t Scop_Buffer[4][512];
extern short Scop_Period,Scop_Period_counter,Scop_Buffer_point,Scop_Start,Scop_Data_Ready,Scop_Send_point;
extern short Scop_Point[4];

extern int led_blink_counter,led_blink_period;
extern char display_buff[40];
extern short OLED_count,OLED_Period;

extern int t1,t2,t3,t4,t5,t6;
extern int tt1,tt2,tt3,tt4,tt5,tt6;
extern int ttt1,ttt2,ttt3,ttt4;

extern u16 store_parameter;
extern u32 software_version;

/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void safe_assert(int arg);
void init_motor_control(void);
void pwm_trig_adc_cb(ADC_HandleTypeDef* hadc);
void vbus_sense_adc_cb(ADC_HandleTypeDef* hadc);

//@TODO move motor thread to high level file
void Init_Driver_State(void);
void start_adc(void);
void start_pwm(TIM_HandleTypeDef* htim);
void stop_pwm(TIM_HandleTypeDef* htim);
void update_motor(Motor_t* motors);
void Calibrate_ADC_Offset(void);
int phase_current_from_adcval(uint32_t ADCValue);
void queue_modulation_timings(Motor_t* motor, int mod_alpha, int mod_beta);
int32_t get_electric_phase(int commutation_current);
void Current_loop(Motor_t* motors, int Id_des, int Iq_des);
void Update_Speed(Motor_t* motors);
void Velocity_loop(Motor_t* motors, int target_vel);
void Position_Loop(Motor_t* motors, int target_pos);
void Motion_process(void);
void DS402_process(void);
void find_commutation(void);
int Low_pass_filter(int* Buffer,int X,int n);
int Low_pass_filter_1(int A,int X,int Y);
int IIt_filter(int A,int X,int Y);
int IIt_DC_filter(int A,int X,int Y);
void Auto_reserve_process(void);
void Update_Parameter(void);
void get_hall_edge_phase(Motor_t* motors);
void Process_Scop_Data(void);
void ENC_Z_Check(void);
void Send_Scop_Data(void);
void Check_DCBus(void);
void Check_Temperature(void);
void Check_IIt(void);
void LED_Process(void);
void OLED_Process(void);
void IO_Process(void);
void calibrate_tamagawa_encoder(void);
void calibrate_hall_phase(void);
void KEY_Process(void);
int arm_cos_f32(int pahse);
int arm_sin_f32(int pahse);
#endif //__LOW_LEVEL_H
