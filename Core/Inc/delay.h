#ifndef __DELAY_H
#define __DELAY_H 			   
#include "sys.h"  
void delay_init(u8 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);


union error_uint32_t
{
	uint32_t all;
	struct
	{
		uint8_t drv_fault:1;						 //0
		uint8_t ADC_error:1;						//1
		uint8_t :1;											//2
		uint8_t ENC_error:1;						//3
		uint8_t hall_state_error:1;			//4
		uint8_t commutation_error:1;		//5
		uint8_t following_error:1;			//6
		uint8_t :1;											//7
		uint8_t voltage_low:1;					//8
		uint8_t voltage_high:1;					//9
		uint8_t over_temperature:1;			//10
		uint8_t over_current:1;					//11
		uint8_t over_load:1;						//12
		uint8_t :1;											//13
		uint8_t :1;											//14
		uint8_t :1;											//15
	}bits;
};


union Status_uint16_t
{
	uint16_t all;
	struct
	{
		uint8_t driver_ready:1;						 //0
		uint8_t motor_on:1;						//1
		uint8_t operation_enable:1;											//2
		uint8_t error:1;											//3
		uint8_t voltage_enable:1;			//4
		uint8_t quick_stop:1;		//5
		uint8_t switch_on_disable:1;			//6
		uint8_t warning:1;											//7
		uint8_t manufacture0:1;					//8
		uint8_t remote:1;					//9
		uint8_t target_reach:1;			//10
		uint8_t internal_limit_active:1;					//11
		uint8_t set_point_ack:1;											//12
		uint8_t following_error:1;											//13
		uint8_t commutation_found:1;											//14
		uint8_t reference_found:1;											//15
	}bits;
};

union Control_uint16_t
{
	uint16_t all;
	struct
	{
		uint8_t Switch_on:1;						 //0
		uint8_t Enable_voltage:1;						//1
		uint8_t Quick_stop:1;											//2
		uint8_t Enable_operation:1;											//3
		uint8_t New_set_point:1;			//4
		uint8_t Change_set_immediately:1;		//5
		uint8_t Abs_relative:1;			//6
		uint8_t Fault_reset:1;											//7
		uint8_t Halt:1;					//8
		uint8_t :1;					//9
		uint8_t :1;			//10
		uint8_t :1;					//11
		uint8_t :1;											//12
		uint8_t :1;											//13
		uint8_t :1;											//14
		uint8_t :1;											//15
	}bits;
};


union can_int32_t
{
	int32_t all;
	struct
	{
		int8_t byte_0;
		int8_t byte_1;
		int8_t byte_2;
		int8_t byte_3;
	}byte;
	struct
	{
		int16_t word_0;
		int16_t word_1;
	}word;		
};
union can_int16_t
{
	int16_t all;
	
	struct
	{
		int8_t byte_0;
		int8_t byte_1;
	}byte;		
};

typedef struct {
	uint8_t cmd;
	uint16_t identifier;
    union can_int32_t od;
    union can_int32_t data;
	struct 
	{
		int8_t len;
		int8_t data_0;
		int8_t data_1;
		int8_t data_2;
		int8_t data_3;
		int8_t data_4;
		int8_t data_5;
		int8_t data_6;
		int8_t data_7;
	}pdo;
} rtx_task_action_t;


#endif





























