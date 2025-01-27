#include "sys.h"
#include "delay.h"
#include "modbus.h"	
#include "crc_16.h"
#include "usart.h"
#include "mcpwm.h"

////////////////////////////////////////////////////////////////////
////加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
//#pragma import(__use_no_semihosting)             
////标准库需要的支持函数                 
//struct __FILE 
//{ 
//	int handle; 

//}; 

//FILE __stdout;       
////定义_sys_exit()以避免使用半主机模式    
//void _sys_exit(int x) 
//{ 
//	x = x; 
//} 
////重定义fputc函数 
//int fputc(int ch, FILE *f)
//{      
//	while((USART1->ISR&0X40)==0); //循环发送,直到发送完毕   
//    
//	USART1->TDR = (u8) ch;      
//	return ch;
//}


#if !defined(__MICROLIB)  
 
#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
__asm (".global __use_no_semihosting\n\t");
 
//FILE __stdout;
 
/* __use_no_semihosting was requested, but _sys_exit was */
void _sys_exit(int x)
{
    x = x;
}
/* __use_no_semihosting was requested, but _ttywrch was */
void _ttywrch(int ch)
{
    ch = ch;
}
#elif defined(__CC_ARM)
#pragma import(__use_no_semihosting)
 
struct __FILE
{
    int handle;
};
FILE __stdout;
 
/* __use_no_semihosting was requested, but _sys_exit was */
void _sys_exit(int x)
{
    x = x;
}
#endif /* __ARMCC_VERSION */
 
#endif /* __MICROLIB */
 
#if defined ( __GNUC__ ) && !defined (__clang__) 
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
 
/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 1000);
	return ch;
}



//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART1_RX_STA=0;       //接收状态标记	  
u32 USART1_RX_TIMECHK;
u8 USART1RxBuffer[RXBUFFERSIZE];//HAL库使用的串口接收缓冲

u16 USART2_RX_STA=0;       //接收状态标记	  
u32 USART2_RX_TIMECHK;
u8 USART2RxBuffer[RXBUFFERSIZE];//HAL库使用的串口接收缓冲


uint8_t RS485_FrameFlag=0;
uint8_t RS485_TX_EN=0;
uint8_t RS485_RX_CNT=0;
uint8_t RS485_RX_BUFF[USART1_REC_LEN];
uint8_t RS485_TX_BUFF[USART1_REC_LEN];

uint8_t RS232_FrameFlag=0;
uint8_t RS232_TX_EN=0;
uint8_t RS232_RX_CNT=0;
uint8_t RS232_RX_BUFF[USART1_REC_LEN];
uint8_t RS232_TX_BUFF[USART1_REC_LEN];

u16 RS232_Addr=1;
u32 RS232_Baudrate=38400;
u16 RS232_Protocol=0;

u16 RS485_Addr=1;
u16 Modbus_Addr_Base=0;
u32 RS485_Baudrate=38400;
u16 RS485_Protocol=0;
uint16_t startRegAddr=0;
uint16_t RegNum;
uint16_t calCRC;
uint16_t *Modbus_Output_Reg[MODBUS_REG_NUM];
u32 software_version;
  
//初始化IO 串口1 
//bound:波特率
void uart1_init(u32 baudrate)
{	
	//UART 初始化设置
	huart1.Instance=USART1;					    //USART3
	huart1.Init.BaudRate=baudrate;				    //波特率
	huart1.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	huart1.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	huart1.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	huart1.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	huart1.Init.Mode=UART_MODE_TX_RX;		    //收发模式
	HAL_UART_Init(&huart1);					    //HAL_UART_Init()会使能uart3
	
	HAL_UART_Receive_IT(&huart1, (u8 *)USART1RxBuffer, RXBUFFERSIZE);//该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量
  
}

void uart2_init(u32 baudrate)
{	
	//UART 初始化设置
	huart2.Instance=USART2;					    //USART2
	huart2.Init.BaudRate=baudrate;				    //波特率
	huart2.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	huart2.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	huart2.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	huart2.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	huart2.Init.Mode=UART_MODE_TX_RX;		    //收发模式
	HAL_UART_Init(&huart2);					    //HAL_UART_Init()会使能uart3
	
	HAL_UART_Receive_IT(&huart2, (u8 *)USART2RxBuffer, RXBUFFERSIZE);//该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量
  
}
char shot=0;
int tamagawa_angle_delta_4;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  char res;
	if(huart->Instance==USART3)
	{
		switch(feedback_type)
		{
			case Tamagawa:
				Tamagawa_calCRC=Tamagawa_RX_BUFF[0]^Tamagawa_RX_BUFF[1]^Tamagawa_RX_BUFF[2]^Tamagawa_RX_BUFF[3]^Tamagawa_RX_BUFF[4]^Tamagawa_RX_BUFF[5];
				if(Tamagawa_calCRC==0)
				{
					Tamagawa_count_temp=0;
					tamagawa_angle_32=Tamagawa_RX_BUFF[4]*0x10000+Tamagawa_RX_BUFF[3]*0x100+Tamagawa_RX_BUFF[2];
					tamagawa_angle=tamagawa_angle_32>>2;
		
					if(Tamagawa_First<10)
						Tamagawa_First++;
						//tamagawa_angle=Tamagawa_RX_BUFF[3]*0x40+(Tamagawa_RX_BUFF[2]>>2);
				}
				else
				{
					Tamagawa_CRC_count++;
				}
			break;
			default:
			break;				
		}
	}
	if(huart->Instance==USART2)//如果是串口3
	{
			if((USART2_RX_STA&0x8000)==0)//接收未完成
			{
				USART2_RX_TIMECHK=HAL_GetTick();
				RS232_RX_BUFF[USART2_RX_STA&0X3FFF]=USART2RxBuffer[0] ;
				USART2_RX_STA++;
				if(USART2_RX_STA>(USART2_REC_LEN-1))USART2_RX_STA=0;//接收数据错误,重新开始接收	  	 
			}
	}
	if(huart->Instance==USART1)//如果是串口3
	{
		if(RS485_TX_EN==0)
			if((USART1_RX_STA&0x8000)==0)//接收未完成
			{
				USART1_RX_TIMECHK=HAL_GetTick();
				RS485_RX_BUFF[USART1_RX_STA&0X3FFF]=USART1RxBuffer[0] ;
				USART1_RX_STA++;
				if(USART1_RX_STA>(USART1_REC_LEN-1))USART1_RX_STA=0;//接收数据错误,重新开始接收	  	 
			}
	}
	/*
	if(huart->Instance==USART1)//如果是串口3
	{
		if(Tamagawa_TX_EN==0)
			if((USART1_RX_STA&0x8000)==0)//接收未完成
			{
				USART1_RX_TIMECHK=HAL_GetTick();
				Tamagawa_RX_BUFF[USART1_RX_STA&0X3FFF]=USART1RxBuffer[0] ;
				USART1_RX_STA++;
				if(USART1_RX_STA>(USART1_REC_LEN-1))USART1_RX_STA=0;//接收数据错误,重新开始接收	  	 
			}
	}*/
}

void RS485_Process(void)
{
		if(((USART1_RX_STA&0x8000)==0x0)&&(USART1_RX_STA&0X3FFF)>0)
		{
			if((HAL_GetTick()-USART1_RX_TIMECHK)>USART1_RX_TIEMOUT)
			{
				USART1_RX_STA|=0x8000;
				RS485_FrameFlag=1;
				RS485_RX_CNT=USART1_RX_STA&0X3FFF;
				
			}
		}
}

void RS232_Process(void)
{
		if(((USART2_RX_STA&0x8000)==0x0)&&(USART2_RX_STA&0X3FFF)>0)
		{
			if((HAL_GetTick()-USART2_RX_TIMECHK)>USART2_RX_TIEMOUT)
			{
				USART2_RX_STA|=0x8000;
				RS232_FrameFlag=1;
				RS232_RX_CNT=USART2_RX_STA&0X3FFF;
				
			}
		}
}

void Modbus_Solve_485_Enable(void)
{
  HAL_GPIO_WritePin(RS485_EN_GPIO_Port,RS485_EN_Pin,GPIO_PIN_SET);	
}

void Modbus_Solve_485_Disenable(void)
{
	int i;
  for(i=0;i<35500;i++){}//切换延时
  HAL_GPIO_WritePin(RS485_EN_GPIO_Port,RS485_EN_Pin,GPIO_PIN_RESET);	
}

void Modbus_Solve_PutString(uint8_t *buf,uint16_t len)
{
		uint16_t i;
	RS485_TX_EN=1;
   Modbus_Solve_485_Enable();
		for(i=0;i<len;i++)
		{
			while((USART1->ISR&0X40)==0); //循环发送,直到发送完毕   
			USART1->TDR = buf[i]; 
		} 
  Modbus_Solve_485_Disenable();	
	RS485_TX_EN=0;		
}
void RS232_Solve_PutString(uint8_t *buf,uint16_t len)
{
		uint16_t i;
	RS232_TX_EN=1;
		for(i=0;i<len;i++)
		{
			while((USART2->ISR&0X40)==0); //循环发送,直到发送完毕   
			USART2->TDR = buf[i]; 
		} 
	RS232_TX_EN=0;		
}

void Modbus_Solve_Service(void)
{
	uint16_t recCRC;
	
	if(RS485_FrameFlag==1)
	{
		if(RS485_RX_CNT>5)
			if((RS485_RX_BUFF[0]==RS485_Addr)||(RS485_RX_BUFF[0]==0))
			{
				if((RS485_RX_BUFF[1]==03)||(RS485_RX_BUFF[1]==06)||(RS485_RX_BUFF[1]==16))
				{ 
					startRegAddr=(((uint16_t)RS485_RX_BUFF[2])<<8)|RS485_RX_BUFF[3];
					startRegAddr=startRegAddr-Modbus_Addr_Base;
					if(startRegAddr<MODBUS_REG_NUM)
					{
						calCRC=Get_Crc16(RS485_RX_BUFF,RS485_RX_CNT-2);
						recCRC=(((uint16_t)RS485_RX_BUFF[RS485_RX_CNT-1])<<8)|RS485_RX_BUFF[RS485_RX_CNT-2];                                   
						if(calCRC==recCRC)
						{
							switch(RS485_RX_BUFF[1])
							{   
								case 03: 
								{       
									if(RS485_RX_BUFF[0]!=0)
									{
									 Modbus_03_Solve();
									}
									break;
								}
								case 06: 
								{
									Modbus_06_Solve();
									break;
								}
								case 16: 
								{
									Modbus_16_Solve();
									break;
								}                                                                    
							}                                          
						}       
					}
				}
			}

			RS485_FrameFlag=0;
			USART1_RX_STA=0;
			RS485_RX_CNT=0;
			RS485_TX_EN=0;
	}


}

void Modbus_03_Solve(void)
{
	uint8_t i;
	RegNum= (((uint16_t)RS485_RX_BUFF[4])<<8)|RS485_RX_BUFF[5];
	if((startRegAddr+RegNum)<MODBUS_REG_NUM)
	{
		RS485_TX_BUFF[0]=RS485_RX_BUFF[0];
		RS485_TX_BUFF[1]=RS485_RX_BUFF[1];
		RS485_TX_BUFF[2]=RegNum*2;
		for(i=0;i<RegNum;i++)
		{
			RS485_TX_BUFF[3+i*2]=(*Modbus_Output_Reg[startRegAddr+i]>>8)&0xFF;
			RS485_TX_BUFF[4+i*2]=*Modbus_Output_Reg[startRegAddr+i]&0xFF;
		}
		calCRC=Get_Crc16(RS485_TX_BUFF,RegNum*2+3);
		RS485_TX_BUFF[RegNum*2+3]=calCRC&0xFF;
		RS485_TX_BUFF[RegNum*2+4]=(calCRC>>8)&0xFF;
		Modbus_Solve_PutString(RS485_TX_BUFF,RegNum*2+5);
	}
	else
	{
		RS485_TX_BUFF[0]=RS485_RX_BUFF[0];
		RS485_TX_BUFF[1]=RS485_RX_BUFF[1]|0x80;
		RS485_TX_BUFF[2]=0x02; 
		Modbus_Solve_PutString(RS485_TX_BUFF,3);
	}
}



void Modbus_06_Solve(void)
{
	*Modbus_Output_Reg[startRegAddr]=RS485_RX_BUFF[5];
	*Modbus_Output_Reg[startRegAddr]|=((uint16_t)RS485_RX_BUFF[4])<<8;

	RS485_TX_BUFF[0]=RS485_RX_BUFF[0];
	RS485_TX_BUFF[1]=RS485_RX_BUFF[1];
	RS485_TX_BUFF[2]=RS485_RX_BUFF[2];
	RS485_TX_BUFF[3]=RS485_RX_BUFF[3];
	RS485_TX_BUFF[4]=RS485_RX_BUFF[4];
	RS485_TX_BUFF[5]=RS485_RX_BUFF[5];
	
	calCRC=Get_Crc16(RS485_TX_BUFF,6);
	RS485_TX_BUFF[6]=calCRC&0xFF;			
	RS485_TX_BUFF[7]=(calCRC>>8)&0xFF;	

	if(RS485_RX_BUFF[0]!=0)
	{
	 Modbus_Solve_PutString(RS485_TX_BUFF,8);
	}

}


void Modbus_16_Solve(void)
{
	uint8_t i;

	RegNum= (((uint16_t)RS485_RX_BUFF[4])<<8)|RS485_RX_BUFF[5];
	if((startRegAddr+RegNum)<MODBUS_REG_NUM)
	{
		for(i=0;i<RegNum;i++)
		{
			*Modbus_Output_Reg[startRegAddr+i]=RS485_RX_BUFF[8+i*2];
			*Modbus_Output_Reg[startRegAddr+i]|=((uint16_t)RS485_RX_BUFF[7+i*2])<<8; 
		}
		
			RS485_TX_BUFF[0]=RS485_RX_BUFF[0];
			RS485_TX_BUFF[1]=RS485_RX_BUFF[1];
			RS485_TX_BUFF[2]=RS485_RX_BUFF[2];
			RS485_TX_BUFF[3]=RS485_RX_BUFF[3];
			RS485_TX_BUFF[4]=RS485_RX_BUFF[4];
			RS485_TX_BUFF[5]=RS485_RX_BUFF[5];
			
			calCRC=Get_Crc16(RS485_TX_BUFF,6);
			RS485_TX_BUFF[6]=calCRC&0xFF;						
			RS485_TX_BUFF[7]=(calCRC>>8)&0xFF;	
		
			if(RS485_RX_BUFF[0]!=0)//
			{
			 Modbus_Solve_PutString(RS485_TX_BUFF,8);
			}
				
	}
	else
	{
			RS485_TX_BUFF[0]=RS485_RX_BUFF[0];
			RS485_TX_BUFF[1]=RS485_RX_BUFF[1]|0x80;
			RS485_TX_BUFF[2]=0x02; 
			if(RS485_RX_BUFF[0]!=0)
			{
			 Modbus_Solve_PutString(RS485_TX_BUFF,3);
			}
	}
}


void RS232_Solve_Service(void)
{
	uint16_t recCRC;
	
	if(RS232_FrameFlag==1)
	{
		if(RS232_RX_CNT>5)
			if((RS232_RX_BUFF[0]==RS232_Addr)||(RS232_RX_BUFF[0]==0))
			{
				if((RS232_RX_BUFF[1]==03)||(RS232_RX_BUFF[1]==06)||(RS232_RX_BUFF[1]==16))
				{ 
					startRegAddr=(((uint16_t)RS232_RX_BUFF[2])<<8)|RS232_RX_BUFF[3];
					startRegAddr=startRegAddr-Modbus_Addr_Base;
					if(startRegAddr<MODBUS_REG_NUM)
					{
						calCRC=Get_Crc16(RS232_RX_BUFF,RS232_RX_CNT-2);
						recCRC=(((uint16_t)RS232_RX_BUFF[RS232_RX_CNT-1])<<8)|RS232_RX_BUFF[RS232_RX_CNT-2];                                   
						if(calCRC==recCRC)
						{
							switch(RS232_RX_BUFF[1])
							{   
								case 03: 
								{       
									if(RS232_RX_BUFF[0]!=0)
									{
									 RS232_03_Solve();
									}
									break;
								}
								case 06: 
								{
									RS232_06_Solve();
									break;
								}
								case 16: 
								{
									RS232_16_Solve();
									break;
								}                                                                    
							}                                          
						}       
					}
				}
			}

			RS232_FrameFlag=0;
			USART2_RX_STA=0;
			RS232_RX_CNT=0;
			RS232_TX_EN=0;
	}


}

void RS232_03_Solve(void)
{
	uint8_t i;
	RegNum= (((uint16_t)RS232_RX_BUFF[4])<<8)|RS232_RX_BUFF[5];
	if((startRegAddr+RegNum)<MODBUS_REG_NUM)
	{
		RS232_TX_BUFF[0]=RS232_RX_BUFF[0];
		RS232_TX_BUFF[1]=RS232_RX_BUFF[1];
		RS232_TX_BUFF[2]=RegNum*2;
		for(i=0;i<RegNum;i++)
		{
			RS232_TX_BUFF[3+i*2]=(*Modbus_Output_Reg[startRegAddr+i]>>8)&0xFF;
			RS232_TX_BUFF[4+i*2]=*Modbus_Output_Reg[startRegAddr+i]&0xFF;
		}
		calCRC=Get_Crc16(RS232_TX_BUFF,RegNum*2+3);
		RS232_TX_BUFF[RegNum*2+3]=calCRC&0xFF;
		RS232_TX_BUFF[RegNum*2+4]=(calCRC>>8)&0xFF;
		RS232_Solve_PutString(RS232_TX_BUFF,RegNum*2+5);
	}
	else
	{
		RS232_TX_BUFF[0]=RS232_RX_BUFF[0];
		RS232_TX_BUFF[1]=RS232_RX_BUFF[1]|0x80;
		RS232_TX_BUFF[2]=0x02; 
		RS232_Solve_PutString(RS232_TX_BUFF,3);
	}
}



void RS232_06_Solve(void)
{
	*Modbus_Output_Reg[startRegAddr]=RS232_RX_BUFF[5];
	*Modbus_Output_Reg[startRegAddr]|=((uint16_t)RS232_RX_BUFF[4])<<8;

	RS232_TX_BUFF[0]=RS232_RX_BUFF[0];
	RS232_TX_BUFF[1]=RS232_RX_BUFF[1];
	RS232_TX_BUFF[2]=RS232_RX_BUFF[2];
	RS232_TX_BUFF[3]=RS232_RX_BUFF[3];
	RS232_TX_BUFF[4]=RS232_RX_BUFF[4];
	RS232_TX_BUFF[5]=RS232_RX_BUFF[5];
	
	calCRC=Get_Crc16(RS232_TX_BUFF,6);
	RS232_TX_BUFF[6]=calCRC&0xFF;			
	RS232_TX_BUFF[7]=(calCRC>>8)&0xFF;	

	if(RS232_RX_BUFF[0]!=0)
	{
	 RS232_Solve_PutString(RS232_TX_BUFF,8);
	}

}


void RS232_16_Solve(void)
{
	uint8_t i;

	RegNum= (((uint16_t)RS232_RX_BUFF[4])<<8)|RS232_RX_BUFF[5];
	if((startRegAddr+RegNum)<MODBUS_REG_NUM)
	{
		for(i=0;i<RegNum;i++)
		{
			*Modbus_Output_Reg[startRegAddr+i]=RS232_RX_BUFF[8+i*2];
			*Modbus_Output_Reg[startRegAddr+i]|=((uint16_t)RS232_RX_BUFF[7+i*2])<<8; 
		}
		
			RS232_TX_BUFF[0]=RS232_RX_BUFF[0];
			RS232_TX_BUFF[1]=RS232_RX_BUFF[1];
			RS232_TX_BUFF[2]=RS232_RX_BUFF[2];
			RS232_TX_BUFF[3]=RS232_RX_BUFF[3];
			RS232_TX_BUFF[4]=RS232_RX_BUFF[4];
			RS232_TX_BUFF[5]=RS232_RX_BUFF[5];
			
			calCRC=Get_Crc16(RS232_TX_BUFF,6);
			RS232_TX_BUFF[6]=calCRC&0xFF;						
			RS232_TX_BUFF[7]=(calCRC>>8)&0xFF;	
		
			if(RS232_RX_BUFF[0]!=0)//
			{
			 RS232_Solve_PutString(RS232_TX_BUFF,8);
			}
				
	}
	else
	{
			RS232_TX_BUFF[0]=RS232_RX_BUFF[0];
			RS232_TX_BUFF[1]=RS232_RX_BUFF[1]|0x80;
			RS232_TX_BUFF[2]=0x02; 
			if(RS232_RX_BUFF[0]!=0)
			{
			 RS232_Solve_PutString(RS232_TX_BUFF,3);
			}
	}
}


uint16_t Reserve=0;
void Init_Modbus_Addr_List(void)
{
	int i;

	for(i=0;i<MODBUS_REG_NUM;i++)
	{
		Modbus_Output_Reg[i]=(uint16_t*)&Reserve;
	}
	
//	Modbus_Output_Reg[10]=(uint16_t*)&store_parameter;
//	
//	Modbus_Output_Reg[30]=(uint16_t*)&device_temperature;
//	Modbus_Output_Reg[31]=(uint16_t*)&vbus_voltage;
//	
//	Modbus_Output_Reg[40]=(uint16_t*)&Iq_real+1;
//	Modbus_Output_Reg[41]=(uint16_t*)&Iq_real;
//	Modbus_Output_Reg[42]=(uint16_t*)&real_speed_filter+1;
//	Modbus_Output_Reg[43]=(uint16_t*)&real_speed_filter;
//	Modbus_Output_Reg[44]=(uint16_t*)&pos_actual+1;
//	Modbus_Output_Reg[45]=(uint16_t*)&pos_actual;
//	Modbus_Output_Reg[46]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[47]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[48]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[49]=(uint16_t*)&Reserve;
//	
//	Modbus_Output_Reg[50]=(uint16_t*)&motor.motion.Error_State.all;
//	Modbus_Output_Reg[51]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[52]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[53]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[54]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[55]=(uint16_t*)&Reserve;
//	
//	Modbus_Output_Reg[56]=(uint16_t*)&Driver_IIt_Real+1;
//	Modbus_Output_Reg[57]=(uint16_t*)&Driver_IIt_Real;
//	Modbus_Output_Reg[58]=(uint16_t*)&Driver_IIt_Real_DC+1;
//	Modbus_Output_Reg[59]=(uint16_t*)&Driver_IIt_Real_DC;
//	
//	Modbus_Output_Reg[60]=(uint16_t*)&operation_mode;
//	Modbus_Output_Reg[61]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[62]=(uint16_t*)&control_word.all;
//	Modbus_Output_Reg[63]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[64]=(uint16_t*)&target_Iq+1;
//	Modbus_Output_Reg[65]=(uint16_t*)&target_Iq;
//	Modbus_Output_Reg[66]=(uint16_t*)&target_speed+1;
//	Modbus_Output_Reg[67]=(uint16_t*)&target_speed;
//	Modbus_Output_Reg[68]=(uint16_t*)&target_position+1;
//	Modbus_Output_Reg[69]=(uint16_t*)&target_position;
//	Modbus_Output_Reg[70]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[71]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[72]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[73]=(uint16_t*)&Reserve;
//	
//	Modbus_Output_Reg[74]=(uint16_t*)&Iq_demand+1;
//	Modbus_Output_Reg[75]=(uint16_t*)&Iq_demand;
//	Modbus_Output_Reg[76]=(uint16_t*)&speed_demand+1;
//	Modbus_Output_Reg[77]=(uint16_t*)&speed_demand;
//	Modbus_Output_Reg[78]=(uint16_t*)&position_demand+1;
//	Modbus_Output_Reg[79]=(uint16_t*)&position_demand;
//	
//	//Modbus_Output_Reg[80]=(uint16_t*)&start_calibrate_hall_phase;
//	Modbus_Output_Reg[81]=(uint16_t*)&set_tamagawa_zero;
//	
//	Modbus_Output_Reg[100]=(uint16_t*)&software_version+1;
//	Modbus_Output_Reg[101]=(uint16_t*)&software_version;
//	Modbus_Output_Reg[102]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[103]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[104]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[105]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[106]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[107]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[108]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[109]=(uint16_t*)&Reserve;
//	
//	Modbus_Output_Reg[110]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[111]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[112]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[113]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[114]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[115]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[116]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[117]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[118]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[119]=(uint16_t*)&Reserve;
//	
//	Modbus_Output_Reg[120]=(uint16_t*)&RS485_Addr;
//	Modbus_Output_Reg[121]=(uint16_t*)&Modbus_Addr_Base;
//	Modbus_Output_Reg[122]=(uint16_t*)&RS485_Baudrate+1;
//	Modbus_Output_Reg[123]=(uint16_t*)&RS485_Baudrate;
//	Modbus_Output_Reg[124]=(uint16_t*)&RS485_Protocol;
//	Modbus_Output_Reg[125]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[126]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[127]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[128]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[129]=(uint16_t*)&Reserve;
//	
//	Modbus_Output_Reg[130]=(uint16_t*)&operation_mode;
//	//Modbus_Output_Reg[132]=(uint16_t*)&control_word;
//	Modbus_Output_Reg[131]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[132]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[133]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[134]=(uint16_t*)&target_Iq+1;
//	Modbus_Output_Reg[135]=(uint16_t*)&target_Iq;
//	Modbus_Output_Reg[136]=(uint16_t*)&target_speed+1;
//	Modbus_Output_Reg[137]=(uint16_t*)&target_speed;
//	Modbus_Output_Reg[138]=(uint16_t*)&target_position+1;
//	Modbus_Output_Reg[139]=(uint16_t*)&target_position;
//	
//	
//	Modbus_Output_Reg[140]=(uint16_t*)&kcp;
//	Modbus_Output_Reg[141]=(uint16_t*)&kci;
//	Modbus_Output_Reg[142]=(uint16_t*)&Ilim+1;
//	Modbus_Output_Reg[143]=(uint16_t*)&Ilim;
//	Modbus_Output_Reg[144]=(uint16_t*)&kci_sum_limit+1;
//	Modbus_Output_Reg[145]=(uint16_t*)&kci_sum_limit;
//	Modbus_Output_Reg[146]=(uint16_t*)&current_in_lpf_a;
//	Modbus_Output_Reg[147]=(uint16_t*)&current_out_lpf_a;
//	Modbus_Output_Reg[148]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[149]=(uint16_t*)&Reserve;
//	
//	Modbus_Output_Reg[150]=(uint16_t*)&kvp;
//	Modbus_Output_Reg[151]=(uint16_t*)&kvi;
//	Modbus_Output_Reg[152]=(uint16_t*)&kvi_sum_limit+1;
//	Modbus_Output_Reg[153]=(uint16_t*)&kvi_sum_limit;
//	Modbus_Output_Reg[154]=(uint16_t*)&speed_in_lpf_a;
//	Modbus_Output_Reg[155]=(uint16_t*)&low_pass_filter_on;
//	Modbus_Output_Reg[156]=(uint16_t*)&real_speed_filter_num;
//	Modbus_Output_Reg[157]=(uint16_t*)&vel_lim+1;
//	Modbus_Output_Reg[158]=(uint16_t*)&vel_lim;
//	Modbus_Output_Reg[159]=(uint16_t*)&speed_out_lpf_a;
//	
//	Modbus_Output_Reg[160]=(uint16_t*)&kpp;
//	Modbus_Output_Reg[161]=(uint16_t*)&kpi;
//	Modbus_Output_Reg[162]=(uint16_t*)&kpi_sum_limit+1;
//	Modbus_Output_Reg[163]=(uint16_t*)&kpi_sum_limit;
//	Modbus_Output_Reg[164]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[165]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[166]=(uint16_t*)&position_in_lpf_a;
//	Modbus_Output_Reg[167]=(uint16_t*)&position_out_lpf_a;
//	Modbus_Output_Reg[168]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[169]=(uint16_t*)&Reserve;
//	
//	Modbus_Output_Reg[170]=(uint16_t*)&auto_reverse_p_time+1;
//	Modbus_Output_Reg[171]=(uint16_t*)&auto_reverse_p_time;
//	Modbus_Output_Reg[172]=(uint16_t*)&auto_reverse_n_time+1;
//	Modbus_Output_Reg[173]=(uint16_t*)&auto_reverse_n_time;
//	Modbus_Output_Reg[174]=(uint16_t*)&auto_p_pos+1;
//	Modbus_Output_Reg[175]=(uint16_t*)&auto_p_pos;
//	Modbus_Output_Reg[176]=(uint16_t*)&auto_n_pos+1;
//	Modbus_Output_Reg[177]=(uint16_t*)&auto_n_pos;
//	Modbus_Output_Reg[178]=(uint16_t*)&auto_switch_on;
//	Modbus_Output_Reg[179]=(uint16_t*)&Reserve;
//	
//	
//	Modbus_Output_Reg[180]=(uint16_t*)&motor_code;
//	Modbus_Output_Reg[181]=(uint16_t*)&poles_num;
//	Modbus_Output_Reg[182]=(uint16_t*)&feedback_resolution+1;
//	Modbus_Output_Reg[183]=(uint16_t*)&feedback_resolution;
//	Modbus_Output_Reg[184]=(uint16_t*)&commutation_mode;
//	Modbus_Output_Reg[185]=(uint16_t*)&motor.motion.commutation_time;
//	Modbus_Output_Reg[186]=(uint16_t*)&commutation_current+1;
//	Modbus_Output_Reg[187]=(uint16_t*)&commutation_current;
//	Modbus_Output_Reg[188]=(uint16_t*)&tamagawa_offset;
//	Modbus_Output_Reg[189]=(uint16_t*)&tamagawa_dir;
//	
//	Modbus_Output_Reg[190]=(uint16_t*)&hall_phase_offset;
//	Modbus_Output_Reg[191]=(uint16_t*)&hall_phase[1];
//	Modbus_Output_Reg[192]=(uint16_t*)&hall_phase[2];
//	Modbus_Output_Reg[193]=(uint16_t*)&hall_phase[3];
//	Modbus_Output_Reg[194]=(uint16_t*)&hall_phase[4];
//	Modbus_Output_Reg[195]=(uint16_t*)&hall_phase[5];
//	Modbus_Output_Reg[196]=(uint16_t*)&hall_phase[6];
//	Modbus_Output_Reg[197]=(uint16_t*)&ENC_Z_Phase_B;
//	Modbus_Output_Reg[198]=(uint16_t*)&phase_dir;
//	Modbus_Output_Reg[199]=(uint16_t*)&vel_dir;
//	
//	Modbus_Output_Reg[200]=(uint16_t*)&motor_rated_current+1;
//	Modbus_Output_Reg[201]=(uint16_t*)&motor_rated_current;
//	Modbus_Output_Reg[202]=(uint16_t*)&motor_peak_current+1;
//	Modbus_Output_Reg[203]=(uint16_t*)&motor_peak_current;
//	Modbus_Output_Reg[204]=(uint16_t*)&motor_overload_time+1;
//	Modbus_Output_Reg[205]=(uint16_t*)&motor_overload_time;
//	Modbus_Output_Reg[206]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[207]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[208]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[209]=(uint16_t*)&Reserve;
//	
//	Modbus_Output_Reg[210]=(uint16_t*)&gear_factor_a+1;
//	Modbus_Output_Reg[211]=(uint16_t*)&gear_factor_a;
//	Modbus_Output_Reg[212]=(uint16_t*)&gear_factor_b+1;
//	Modbus_Output_Reg[213]=(uint16_t*)&gear_factor_b;
//	Modbus_Output_Reg[214]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[215]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[216]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[217]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[218]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[219]=(uint16_t*)&Reserve;
//	
//	Modbus_Output_Reg[220]=(uint16_t*)&profile_target_position+1;
//	Modbus_Output_Reg[221]=(uint16_t*)&profile_target_position;
//	Modbus_Output_Reg[222]=(uint16_t*)&profile_speed+1;
//	Modbus_Output_Reg[223]=(uint16_t*)&profile_speed;
//	Modbus_Output_Reg[224]=(uint16_t*)&profile_acce+1;
//	Modbus_Output_Reg[225]=(uint16_t*)&profile_acce;
//	Modbus_Output_Reg[226]=(uint16_t*)&profile_dece+1;
//	Modbus_Output_Reg[227]=(uint16_t*)&profile_dece;
//	Modbus_Output_Reg[228]=(uint16_t*)&searching_speed+1;
//	Modbus_Output_Reg[229]=(uint16_t*)&searching_speed;
//	
//	Modbus_Output_Reg[230]=(uint16_t*)&motion_out_lpf_a;
//	Modbus_Output_Reg[231]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[232]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[233]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[234]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[235]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[236]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[237]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[238]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[239]=(uint16_t*)&Reserve;
//	
//	
//	Modbus_Output_Reg[240]=(uint16_t*)&feedback_type;
//	
//	Modbus_Output_Reg[250]=(uint16_t*)&over_voltage;
//	Modbus_Output_Reg[251]=(uint16_t*)&under_voltage;
//	Modbus_Output_Reg[252]=(uint16_t*)&chop_voltage;
//	Modbus_Output_Reg[253]=(uint16_t*)&over_temperature;
//	Modbus_Output_Reg[254]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[255]=(uint16_t*)&Reserve;
//	Modbus_Output_Reg[256]=(uint16_t*)&Driver_IIt_Filter;
//	Modbus_Output_Reg[257]=(uint16_t*)&Driver_IIt_Current+1;
//	Modbus_Output_Reg[258]=(uint16_t*)&Driver_IIt_Current;
//	Modbus_Output_Reg[259]=(uint16_t*)&Driver_IIt_Filter_DC;
//	Modbus_Output_Reg[260]=(uint16_t*)&Driver_IIt_Current_DC+1;
//	Modbus_Output_Reg[261]=(uint16_t*)&Driver_IIt_Current_DC;
//	
	
}

	


