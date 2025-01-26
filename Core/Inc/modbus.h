#ifndef __MODBUS_H
#define __MODBUS_H
#include "stdio.h"	
#include "sys.h" 


#define USART1_REC_LEN  			200  		//�����������ֽ��� 200
#define EN_USART1_RX 			1			//ʹ�ܣ�1��/��ֹ��0������1����
#define USART1_RX_TIEMOUT 			1			//

#define USART2_REC_LEN  			200  		//�����������ֽ��� 200
#define EN_USART2_RX 			1			//ʹ�ܣ�1��/��ֹ��0������1����
#define USART2_RX_TIEMOUT 			5			//
	  	
#define MODBUS_REG_NUM     500

extern u16 USART1_RX_STA;         			//����״̬���	
extern u32 USART1_RX_TIMECHK;

extern u16 USART2_RX_STA;         			//����״̬���	
extern u32 USART2_RX_TIMECHK;

extern uint8_t RS232_FrameFlag;
extern uint8_t RS232_TX_EN;
extern uint8_t RS232_RX_CNT;
extern uint8_t RS232_RX_BUFF[];
extern uint8_t RS232_TX_BUFF[];

extern u16 RS232_Addr;
extern u32 RS232_Baudrate;
extern u16 RS232_Protocol;

extern uint8_t RS485_FrameFlag;
extern uint8_t RS485_TX_EN;
extern uint8_t RS485_RX_CNT;
extern uint8_t RS485_RX_BUFF[];
extern uint8_t RS485_TX_BUFF[];
extern uint16_t RS485_Addr;
extern u16 Modbus_Addr_Base;
extern u32 RS485_Baudrate;
extern u16 RS485_Protocol;
extern uint16_t *Modbus_Output_Reg[];

#define RXBUFFERSIZE   1 					//�����С
extern u8 USART1RxBuffer[RXBUFFERSIZE];			//HAL��USART����Buffer
extern u8 USART2RxBuffer[RXBUFFERSIZE];			//HAL��USART����Buffer

//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart1_init(u32 baudrate);
void uart2_init(u32 baudrate);
void Modbus_Solve_PutString(uint8_t *buf,uint16_t len);
void RS485_Process(void);
void Modbus_Solve_485_Enable(void);
void Modbus_Solve_485_Disenable(void);
void Modbus_Solve_PutString(uint8_t *buf,uint16_t len);
void Modbus_Solve_Service(void);
void Modbus_03_Solve(void);
void Modbus_06_Solve(void);
void Modbus_16_Solve(void);

void RS232_Process(void);
void RS232_Solve_PutString(uint8_t *buf,uint16_t len);
void RS232_Solve_Service(void);
void RS232_03_Solve(void);
void RS232_06_Solve(void);
void RS232_16_Solve(void);
void Init_Modbus_Addr_List(void);
#endif


