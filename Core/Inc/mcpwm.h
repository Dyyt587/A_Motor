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
#include "utils.h"	
#include "apid.h"
#include "workchain.h"
#define CURRENT_MEAS_PERIOD ( (float)2*TIM_1_8_PERIOD_CLOCKS*(TIM_1_8_RCR+1) / (float)TIM_1_8_CLOCK_HZ )
#define CURRENT_MEAS_HZ ( (float)(TIM_1_8_CLOCK_HZ) / (float)(2*TIM_1_8_PERIOD_CLOCKS*(TIM_1_8_RCR+1)) )

#define SOFTWARE_VERSION 2022032001

#define ENC_Z_DIFF_ERROR 20

#define PI 3142


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
typedef struct{
    uint16_t poles_num;
    uint8_t phase_dir;
}param_t;
typedef struct{
    union error_uint32_t Error_State;
    short commutation_founded , commutation_mode , commutation_time;
    int feedback_resolution;

}motion_t;
typedef struct{
    int target_speed, target_position, target_position_b, target_Iq, target_Id ;

}control_t;
typedef struct {
    //osThreadId motor_thread;
    TIM_HandleTypeDef* motor_timer; //PWM timer
    uint16_t PWM1_Duty;
    uint16_t PWM2_Duty;
    uint16_t PWM3_Duty;
    uint16_t control_deadline;
    int PhaseU_current_ma;
    int PhaseV_current_ma;
    int PhaseW_current_ma;
    int shunt_conductance; // 100 means 1 mOh, current sensing resistor
    int phase_inductance; // 1 means 1 uH
    int phase_resistor; // 1 means 1 mOh
    TIM_HandleTypeDef* encoder_timer;
    int16_t encoder_offset;
    int32_t encoder_state; //编码器角度
    int phase;
    float pll_pos;
    float pll_vel;
    float pll_kp;
    float pll_ki;

    uint16_t angle;//编码器原始角度
    uint16_t angle_b;
    control_t control;
    motion_t motion;
    param_t param;

    svpwm_t svpwm;
    apid_t apidd;
    apid_t apidq;
		
	wkc_t wkc;
} Motor_t;

typedef enum{
    Incremental=0U,
    Default = 1U,
    Unknown_2 = 2U,
    Tamagawa=4U,
    Tamagawa_1=5U,
    Unknown_8=8U,
}Encoder_Type_e;
enum Motor_thread_signals {
    M_SIGNAL_PH_CURRENT_MEAS = 1u << 0
};
typedef struct
{
	unsigned short  count,  count_back,  count_c,  first,  trig,  counting_error;
	short  diff,  diff_back;
	int  pos,  pos_back,  pos_offset,  pos_diff,  pos_diff_back;
} ENC_Z;
typedef struct{
    short u;
    short v;
    short w;
    short state;
    short state_back;
    short error; 
}Hall_t;

/* Exported constants --------------------------------------------------------*/
extern short vbus_voltage,device_temperature;
extern Motor_t motor;
extern const int num_motors;
extern int ADCValue[6],ADC_Offset[6],ADC_Value[6];
extern int Id,Iq,Iq_real,Id_real;
extern short commutation_founded,commutation_mode,commutation_time;
extern int commutation_current,motor_rated_current,motor_peak_current,motor_overload_time;
extern short phase_dir,hall_phase_dir,vel_dir;
extern s32 pos_actual,pos_offest;
extern int Iq_demand,Id_demand,target_Iq,target_Id,target_speed,speed_demand,target_position,target_position_b,position_demand;
extern short operation_mode,operation_mode_buff;
extern union Control_uint16_t  control_word,control_word_b;
extern u16 motor_on;
extern short Driver_Ready;
extern int drv8301_error;
// extern unsigned short enc_z.count,enc_z.count_back,enc_z.count_c,enc_z.first,enc_z.trig,enc_z.counting_error;
// extern short enc_z.diff,enc_z.diff_back;
// extern int enc_z.pos,enc_z.pos_back,enc_z.pos_offset,enc_z.pos_diff,enc_z.pos_diff_back;
// extern short hall_u,hall.v,hall.w,hall.state,hall.state_back;
extern ENC_Z enc_z;
extern Hall_t hall;
extern Encoder_Type_e feedback_type;
extern svpwm_t svpwm;


extern short hall_phase[],ENC_Z_Offset,hall_phase_offset,ENC_Z_Phase,ENC_Z_Phase_B,ENC_Z_Phase_Err,start_calibrate_hall_phase;
extern int hall_position,hall_position_b;
extern short encoder_offset_diff,hall_phase_offset_diff;
extern uint16_t poles_num,motor_code; 
//extern int motor.motion.feedback_resolution;
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

//extern int display_speed_loop_count,display_speed_update,display_speed,display_encoder_state_b;
extern int display_speed;
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
void update_motor(Motor_t *motors,uint16_t angle);
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

#endif //__LOW_LEVEL_H
