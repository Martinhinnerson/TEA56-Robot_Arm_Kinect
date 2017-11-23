/*
 * LCD.cpp
 *
 * Created: 4/26/2016 8:45:58 AM
 *  Author: marho949
 */ 
#ifndef F_CPU
#define F_CPU 14745600UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>

#define dataport PORTA
#define commport PORTD

#define rs PD4
#define wr PD5
#define en PD6

int LCD_init(void);
int LCD_SendData(unsigned char *);
int wrcomm(void);
int wrdata(void);

int main(void)
{
	DDRA = 0xFF; //PortA outputs
	DDRD = 0x70; // PortD 4, 5, 6 outputs
	
	LCD_init();
	
	LCD_SendData(" HEJ ");
	
	dataport = 0XC0;
	
	wrcomm();
	
	LCD_SendData(" HEEY ");
	
    while(1)
    {
        //TODO:: Please write your application code 
    }
	
	return 1;
}

int LCD_init()
{
	_delay_ms(50); // wait for VDD to rise
	dataport = 0x30;
	wrcomm();
	dataport = 0x30;
	wrcomm();
	dataport = 0x30;
	wrcomm();
	
	dataport = 0x38; // 2 lines, normal font
	wrcomm();
	dataport = 0xC; // display on
	wrcomm();
	dataport = 0x01; // display clear
	wrcomm();
	dataport = 0x06; // increment, don't shift
	wrcomm();
	return 1;
}

int LCD_SendData(unsigned char *s)
{
	unsigned char *j = s;
	for(int i = 0; i<strlen(j);i++)
	{
		dataport = j[i];
		//dataport = 0b01010011; // 'S'
		
		
		wrdata();
	}
	return 1;
}

int wrcomm()
{
	commport &= ~(1 << rs);
	commport &= ~(1 << wr);
	_delay_us(2);
	commport |= (1 << en);
	_delay_us(2);
	commport &= ~(1 << en);
	_delay_ms(10);
	return 1;
}

int wrdata()
{
	commport |= (1 << rs);
	commport &= ~(1 << wr);
	_delay_us(2);
	commport |= (1 << en);
	_delay_us(2);
	commport &= ~(1 << en);
	_delay_ms(100);
	return 1;
}
