#include "lcd_init.h"
#include "spi.h"

uint16_t swap_uint16(uint16_t data)
{
		return (data<<8)|(data>>8);
}

/****** 底层接口部分 ******/
void Lcd_WR_DATA8(unsigned char data)
{	
		LCD_DC_Set();
		HAL_SPI_Transmit(&SPI_LCD, &data, 0x01, 0x10);
}  

void Lcd_WR_DATA(uint16_t data)
{
		data = swap_uint16(data);
		LCD_DC_Set();
		/* note: 使用HAL库一次发送两个字节顺序与屏幕定义顺序相反 */
		HAL_SPI_Transmit(&SPI_LCD, (uint8_t *)&data, 0x02, 0x10);
}	  

void Lcd_WR_DATA_chunk(uint16_t* data, uint16_t len)
{
		LCD_DC_Set();
		HAL_SPI_Transmit(&SPI_LCD, (uint8_t *)data, len, 0x10);
}	  

void Lcd_WR_REG(unsigned char data)	 
{	
		LCD_DC_Clr();
		HAL_SPI_Transmit(&SPI_LCD, &data, 0x01, 0x10);
}


/******************************************************************************
      函数说明：设置起始和结束地址
      入口数据：x1,x2 设置列的起始和结束地址
                y1,y2 设置行的起始和结束地址
      返回值：  无
******************************************************************************/
void Lcd_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{
	if(USE_HORIZONTAL==0)
	{
		Lcd_WR_REG(0x2a);//列地址设置
		Lcd_WR_DATA(x1+26);
		Lcd_WR_DATA(x2+26);
		Lcd_WR_REG(0x2b);//行地址设置
		Lcd_WR_DATA(y1+1);
		Lcd_WR_DATA(y2+1);
		Lcd_WR_REG(0x2c);//储存器写
	}
	else if(USE_HORIZONTAL==1)
	{
		Lcd_WR_REG(0x2a);//列地址设置
		Lcd_WR_DATA(x1+26);
		Lcd_WR_DATA(x2+26);
		Lcd_WR_REG(0x2b);//行地址设置
		Lcd_WR_DATA(y1+1);
		Lcd_WR_DATA(y2+1);
		Lcd_WR_REG(0x2c);//储存器写
	}
	else if(USE_HORIZONTAL==2)
	{
		Lcd_WR_REG(0x2a);//列地址设置
		Lcd_WR_DATA(x1+1);
		Lcd_WR_DATA(x2+1);
		Lcd_WR_REG(0x2b);//行地址设置
		Lcd_WR_DATA(y1+26);
		Lcd_WR_DATA(y2+26);
		Lcd_WR_REG(0x2c);//储存器写
	}
	else
	{
		Lcd_WR_REG(0x2a);//列地址设置
		Lcd_WR_DATA(x1+1);
		Lcd_WR_DATA(x2+1);
		Lcd_WR_REG(0x2b);//行地址设置
		Lcd_WR_DATA(y1+26);
		Lcd_WR_DATA(y2+26);
		Lcd_WR_REG(0x2c);//储存器写
	}
}

void Lcd_Init(void)
{
	LCD_RES_Clr();  //复位
	HAL_Delay(100);
	LCD_RES_Set();
	HAL_Delay(100);
	
	//LCD_BLK_Set();//打开背光
  //HAL_Delay(100);
	
	Lcd_WR_REG(0x11);     //Sleep out
	HAL_Delay(120);                //Delay 120ms
	Lcd_WR_REG(0xB1);     //Normal mode
	Lcd_WR_DATA8(0x05);   
	Lcd_WR_DATA8(0x3C);   
	Lcd_WR_DATA8(0x3C);   
	Lcd_WR_REG(0xB2);     //Idle mode
	Lcd_WR_DATA8(0x05);   
	Lcd_WR_DATA8(0x3C);   
	Lcd_WR_DATA8(0x3C);   
	Lcd_WR_REG(0xB3);     //Partial mode
	Lcd_WR_DATA8(0x05);   
	Lcd_WR_DATA8(0x3C);   
	Lcd_WR_DATA8(0x3C);   
	Lcd_WR_DATA8(0x05);   
	Lcd_WR_DATA8(0x3C);   
	Lcd_WR_DATA8(0x3C);   
	Lcd_WR_REG(0xB4);     //Dot inversion
	Lcd_WR_DATA8(0x03);   
	Lcd_WR_REG(0xC0);     //AVDD GVDD
	Lcd_WR_DATA8(0xAB);   
	Lcd_WR_DATA8(0x0B);   
	Lcd_WR_DATA8(0x04);   
	Lcd_WR_REG(0xC1);     //VGH VGL
	Lcd_WR_DATA8(0xC5);   //C0
	Lcd_WR_REG(0xC2);     //Normal Mode
	Lcd_WR_DATA8(0x0D);   
	Lcd_WR_DATA8(0x00);   
	Lcd_WR_REG(0xC3);     //Idle
	Lcd_WR_DATA8(0x8D);   
	Lcd_WR_DATA8(0x6A);   
	Lcd_WR_REG(0xC4);     //Partial+Full
	Lcd_WR_DATA8(0x8D);   
	Lcd_WR_DATA8(0xEE);   
	Lcd_WR_REG(0xC5);     //VCOM
	Lcd_WR_DATA8(0x0F);   
	Lcd_WR_REG(0xE0);     //positive gamma
	Lcd_WR_DATA8(0x07);   
	Lcd_WR_DATA8(0x0E);   
	Lcd_WR_DATA8(0x08);   
	Lcd_WR_DATA8(0x07);   
	Lcd_WR_DATA8(0x10);   
	Lcd_WR_DATA8(0x07);   
	Lcd_WR_DATA8(0x02);   
	Lcd_WR_DATA8(0x07);   
	Lcd_WR_DATA8(0x09);   
	Lcd_WR_DATA8(0x0F);   
	Lcd_WR_DATA8(0x25);   
	Lcd_WR_DATA8(0x36);   
	Lcd_WR_DATA8(0x00);   
	Lcd_WR_DATA8(0x08);   
	Lcd_WR_DATA8(0x04);   
	Lcd_WR_DATA8(0x10);   
	Lcd_WR_REG(0xE1);     //negative gamma
	Lcd_WR_DATA8(0x0A);   
	Lcd_WR_DATA8(0x0D);   
	Lcd_WR_DATA8(0x08);   
	Lcd_WR_DATA8(0x07);   
	Lcd_WR_DATA8(0x0F);   
	Lcd_WR_DATA8(0x07);   
	Lcd_WR_DATA8(0x02);   
	Lcd_WR_DATA8(0x07);   
	Lcd_WR_DATA8(0x09);   
	Lcd_WR_DATA8(0x0F);   
	Lcd_WR_DATA8(0x25);   
	Lcd_WR_DATA8(0x35);   
	Lcd_WR_DATA8(0x00);   
	Lcd_WR_DATA8(0x09);   
	Lcd_WR_DATA8(0x04);   
	Lcd_WR_DATA8(0x10);
		 
	Lcd_WR_REG(0xFC);    
	Lcd_WR_DATA8(0x80);  
		
	Lcd_WR_REG(0x3A);     
	Lcd_WR_DATA8(0x05);   
	Lcd_WR_REG(0x36);
	if(USE_HORIZONTAL==0)Lcd_WR_DATA8(0x08);
	else if(USE_HORIZONTAL==1)Lcd_WR_DATA8(0xC8);
	else if(USE_HORIZONTAL==2)Lcd_WR_DATA8(0x78);
	else Lcd_WR_DATA8(0xA8);  
	//Lcd_WR_REG(0x21);     //Display inversion
	Lcd_WR_REG(0x29);     //Display on
	Lcd_WR_REG(0x2A);     //Set Column Address
	Lcd_WR_DATA8(0x00);   
	Lcd_WR_DATA8(0x1A);  //26  
	Lcd_WR_DATA8(0x00);   
	Lcd_WR_DATA8(0x69);   //105 
	Lcd_WR_REG(0x2B);     //Set Page Address
	Lcd_WR_DATA8(0x00);   
	Lcd_WR_DATA8(0x01);    //1
	Lcd_WR_DATA8(0x00);   
	Lcd_WR_DATA8(0xA0);    //160
	Lcd_WR_REG(0x2C); 
}

