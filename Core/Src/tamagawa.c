#include "sys.h"
#include "delay.h"
#include "modbus.h"	
#include "crc_16.h"
#include "usart.h"
#include "mcpwm.h"
#include "tamagawa.h"	


//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART3_RX_STA=0;       //接收状态标记	  
u32 USART3_RX_TIMECHK;
u8 USART3RxBuffer[RXBUFFERSIZE];//HAL库使用的串口接收缓冲


uint8_t Tamagawa_FrameFlag=0;
uint8_t Tamagawa_TX_EN=0;
uint8_t Tamagawa_RX_CNT=0;
uint8_t Tamagawa_RX_BUFF[USART1_REC_LEN];
uint8_t Tamagawa_TX_BUFF[USART1_REC_LEN];

uint8_t Tamagawa_RX_BUFF_B[15];

u16 Tamagawa_Addr=1;
u16 Tamagawa_Addr_Base=0;
u32 Tamagawa_Baudrate=2500000;
u16 Tamagawa_Protocol=0;
uint16_t Tamagawa_calCRC,Tamagawa_CRC_count=0,Tamagawa_First=0,Tamagawa_count_temp=0,Tamagawa_lost=0;
  

int angle_low=0,angle_high=0;	
uint16_t Wait_Tamagawa=0,tamagawa_count=0,tamagawa_angle=0,tamagawa_angle_b=0,tamagawa_angle_offset=0,tamagawa_angle_first=0;
uint16_t set_tamagawa_zero=0,set_tamagawa_zero_count=0;
uint32_t tamagawa_angle_32=0,tamagawa_ENID=0,tamagawa_ALMC=0;
int16_t tamagawa_multi_turn=0;
uint16_t tamagawa_angle_1=0,tamagawa_angle_2=0,tamagawa_angle_3=0,tamagawa_angle_4=0,tamagawa_angle_delta=0,tamagawa_angle_delta_1=0,tamagawa_angle_delta_2=0,tamagawa_angle_delta_3=0;
	
	
//初始化IO 串口1 
//bound:波特率
void uart3_init(u32 baudrate)
{	
	//UART 初始化设置
	huart3.Instance=USART3;					    //USART3
	huart3.Init.BaudRate=baudrate;				    //波特率
	huart3.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	huart3.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	huart3.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	huart3.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	huart3.Init.Mode=UART_MODE_TX_RX;		    //收发模式
	HAL_UART_Init(&huart3);					    //HAL_UART_Init()会使能uart3
	
	//HAL_UART_Receive_IT(&huart1, (u8 *)USART1RxBuffer, RXBUFFERSIZE);//该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量
  
}
void Tamagawa_Read_Cmd(uint8_t *buf,uint16_t len)
{
	uint16_t i;
	USART3_RX_STA|=0x8000;
	Tamagawa_FrameFlag=1;
	Tamagawa_RX_CNT=USART3_RX_STA&0X3FFF;
	Tamagawa_TX_EN=1;
  HAL_GPIO_WritePin(TAMAGAWA_TX_EN_GPIO_Port,TAMAGAWA_TX_EN_Pin,GPIO_PIN_SET);
	for(i=0;i<len;i++)
	{
		while((USART3->ISR&0X40)==0); //循环发送,直到发送完毕   
		USART3->TDR = buf[i]; 
	} 
	while((USART3->ISR&0X40)==0);//循环发送,直到发送完毕   
	//delay_us(2);
  HAL_GPIO_WritePin(TAMAGAWA_TX_EN_GPIO_Port,TAMAGAWA_TX_EN_Pin,GPIO_PIN_RESET);
	//delay_us(5);
	switch(feedback_type)
	{
		case 4:
			HAL_UART_Receive_DMA(&huart3,Tamagawa_RX_BUFF,6);
			//UART_Start_Receive_DMA(&huart3,Tamagawa_RX_BUFF,6);
		break;
		case 5:
			HAL_UART_Receive_DMA(&huart3,Tamagawa_RX_BUFF,11);
		break;
	}
	Tamagawa_TX_EN=0;		
	
}



