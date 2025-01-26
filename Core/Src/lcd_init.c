#include "lcd_init.h"
#include "spi.h"

uint16_t swap_uint16(uint16_t data)
{
		return (data<<8)|(data>>8);
}

/****** �ײ�ӿڲ��� ******/
void Lcd_WR_DATA8(unsigned char data)
{	
		LCD_DC_Set();
		HAL_SPI_Transmit(&SPI_LCD, &data, 0x01, 0x10);
}  

void Lcd_WR_DATA(uint16_t data)
{
		data = swap_uint16(data);
		LCD_DC_Set();
		/* note: ʹ��HAL��һ�η��������ֽ�˳������Ļ����˳���෴ */
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
      ����˵����������ʼ�ͽ�����ַ
      ������ݣ�x1,x2 �����е���ʼ�ͽ�����ַ
                y1,y2 �����е���ʼ�ͽ�����ַ
      ����ֵ��  ��
******************************************************************************/
void Lcd_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{
	if(USE_HORIZONTAL==0)
	{
		Lcd_WR_REG(0x2a);//�е�ַ����
		Lcd_WR_DATA(x1+26);
		Lcd_WR_DATA(x2+26);
		Lcd_WR_REG(0x2b);//�е�ַ����
		Lcd_WR_DATA(y1+1);
		Lcd_WR_DATA(y2+1);
		Lcd_WR_REG(0x2c);//������д
	}
	else if(USE_HORIZONTAL==1)
	{
		Lcd_WR_REG(0x2a);//�е�ַ����
		Lcd_WR_DATA(x1+26);
		Lcd_WR_DATA(x2+26);
		Lcd_WR_REG(0x2b);//�е�ַ����
		Lcd_WR_DATA(y1+1);
		Lcd_WR_DATA(y2+1);
		Lcd_WR_REG(0x2c);//������д
	}
	else if(USE_HORIZONTAL==2)
	{
		Lcd_WR_REG(0x2a);//�е�ַ����
		Lcd_WR_DATA(x1+1);
		Lcd_WR_DATA(x2+1);
		Lcd_WR_REG(0x2b);//�е�ַ����
		Lcd_WR_DATA(y1+26);
		Lcd_WR_DATA(y2+26);
		Lcd_WR_REG(0x2c);//������д
	}
	else
	{
		Lcd_WR_REG(0x2a);//�е�ַ����
		Lcd_WR_DATA(x1+1);
		Lcd_WR_DATA(x2+1);
		Lcd_WR_REG(0x2b);//�е�ַ����
		Lcd_WR_DATA(y1+26);
		Lcd_WR_DATA(y2+26);
		Lcd_WR_REG(0x2c);//������д
	}
}

void Lcd_Init(void)
{
	LCD_RES_Clr();  //��λ
	HAL_Delay(100);
	LCD_RES_Set();
	HAL_Delay(100);
	
	//LCD_BLK_Set();//�򿪱���
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

