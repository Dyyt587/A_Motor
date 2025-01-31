#ifndef __TAMAGAWA_H
#define __TAMAGAWA_H
#include "stdio.h"	
//#include "sys.h" 

#define USART3_REC_LEN  			200  		//定义最大接收字节数 200
#define EN_USART3_RX 			1			//使能（1）/禁止（0）串口1接收
#define USART3_RX_TIEMOUT 			5			//

extern u16 USART3_RX_STA;         			//接收状态标记	
extern u32 USART3_RX_TIMECHK;

extern uint8_t Tamagawa_FrameFlag;
extern uint8_t Tamagawa_TX_EN;
extern uint8_t Tamagawa_RX_CNT;
extern uint8_t Tamagawa_RX_BUFF[];
extern uint8_t Tamagawa_TX_BUFF[];
extern uint8_t Tamagawa_RX_BUFF_B[];
extern uint16_t Tamagawa_Addr;
extern u16 Tamagawa_Addr_Base;
extern u32 Tamagawa_Baudrate;
extern u16 Tamagawa_Protocol,Tamagawa_calCRC,Tamagawa_CRC_count,Tamagawa_First,Tamagawa_count_temp,Tamagawa_lost;

extern int angle_low,angle_high;	
extern uint16_t Wait_Tamagawa,tamagawa_count,tamagawa_angle,tamagawa_angle_b,tamagawa_angle_offset,tamagawa_angle_first;
extern uint16_t set_tamagawa_zero,set_tamagawa_zero_count;
extern uint32_t tamagawa_angle_32,tamagawa_ENID,tamagawa_ALMC;
extern int16_t tamagawa_multi_turn;
//extern uint16_t tamagawa_angle_1,tamagawa_angle_2,tamagawa_angle_3,tamagawa_angle_4,tamagawa_angle_delta,tamagawa_angle_delta_1,tamagawa_angle_delta_2,tamagawa_angle_delta_3;

#define RXBUFFERSIZE   1 					//缓存大小
extern u8 USART3RxBuffer[RXBUFFERSIZE];			//HAL库USART接收Buffer

void uart3_init(u32 baudrate);
void Tamagawa_Read_Cmd(uint8_t cmd);
void send_to_tamagawa(void);
uint16_t get_Tamagawa_encoder(void);
#endif

