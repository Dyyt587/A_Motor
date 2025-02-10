
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
		uint8_t drv_init:1;						//0
		uint8_t drv_ready:1;						//1
		uint8_t motor_on:1;											//2
		uint8_t enc_touched:1;						//3
		uint8_t svm_apply:1;			//4
		uint8_t :1;		//5
		uint8_t time_triged:1;			//6
		uint8_t :1;											//7
		uint8_t is_voltage_ok:1;					//8
		uint8_t voltage_high:1;					//9
		uint8_t is_temperature_ok:1;			//10
		uint8_t is_current_ok:1;					//11
		uint8_t is_load_ok:1;						//12
		uint8_t :1;											//13
		uint8_t :1;											//14
		uint8_t :1;											//15
		uint8_t commutation_founded:1;											//11
		uint8_t resistance_measured:1;											//12
		uint8_t poles_num_measured:1;											//13
		uint8_t inductance_measured:1;											//14
		uint8_t :1;											//15
		uint8_t torque_mode:1;											//16
		uint8_t :1;											//17
		uint8_t :1;											//18
		uint8_t velocity_mode:1;											//19
		uint8_t :1;											//20
		uint8_t :1;											//21
		uint8_t position_mode:1;											//22
		uint8_t :1;											//23
		uint8_t :1;											//24
		uint8_t :1;											//25
		uint8_t :1;											//26
		uint8_t motor_require_on:1;											//27
		uint8_t motor_require_off:1;											//28
		uint8_t :1;											//29
		uint8_t :1;											//30
		uint8_t :1;											//31
	}bits;
}LicenseDomain;
typedef struct wkc_work wkc_work_t;
typedef struct wkc wkc_t;

typedef int(*work_handle)(wkc_t* wkc);//一定会传入wc_t 主要是修改许可
struct wkc_work{
	
	char* name;
	work_handle handle;
	LicenseDomain licence;

	int trig_level;
	int trig_cnt;
	
	wkc_work_t *next; 
	
	void* user_date;//用于任务块自定义数据
};


struct wkc{
	LicenseDomain lic_aprove;
	wkc_work_t* works;//工作的链表头
	wkc_work_t* current_work;//当前工作
	void* user_date;//用于标记电机

};

bool check_licence(wkc_t*wc,LicenseDomain licence);
void wkc_init(wkc_t *wkc);
int wkc_handle(wkc_t *wkc);
void wkc_work_add(wkc_t *wkc,wkc_work_t* work);
void wkc_work_del(wkc_t *wkc, wkc_work_t *work);
#ifdef __cplusplus
}
#endif
#endif
