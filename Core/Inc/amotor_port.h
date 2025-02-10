/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AMOTOR_PORT_H
#define __AMOTOR_PORT_H

/* Includes ------------------------------------------------------------------*/
//#include <cmsis_os.h>
#include "mcpwm.h"
#include "modbus.h"
#include "parameter.h"
#include "NTC_Calculate.h"
#include "tamagawa.h"	
#include "utils.h"	
#include "apid.h"
#include "workchain.h"

void pwm_U_start(Motor_t* motors);
void pwm_V_start(Motor_t* motors);
void pwm_W_start(Motor_t* motors);


void pwm_start(Motor_t* motors);

void pwm_U_stop(Motor_t* motors);
void pwm_V_stop(Motor_t* motors);
void pwm_W_stop(Motor_t* motors);

void pwm_stop(Motor_t* motors);

void queue_modulation_timings(Motor_t* motor);

void amotor_part_init(void);

#endif //__AMOTOR_PORT_H
