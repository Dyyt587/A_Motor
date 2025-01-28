
#ifndef _WORKCHAIN_H
#define _WORKCHAIN_H
#ifdef __cplusplus
extern "C"
{
#endif
#include <stdint.h>
#include <stdbool.h>
typedef union LicenseDomain
{
	uint32_t all;
	struct
	{
		uint8_t drv_fault:1;						//0
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
}LicenseDomain;

typedef int(*work_handle)(void* argv,int argc);//一定会传入wc_t 主要是修改许可
typedef struct work work_t;
struct work{
	work_handle handle;
	LicenseDomain licence;

	int trig_level;
	int trig_cnt;
	
	work_t *next; 
};

typedef struct wc wc_t;

struct wc{
	LicenseDomain lic_aprove;
	work_t works;//工作的链表头
};

bool check_licence(wc_t*wc,LicenseDomain licence);

#ifdef __cplusplus
}
#endif
#endif
