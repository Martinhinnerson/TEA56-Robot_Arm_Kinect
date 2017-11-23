/*
 * Kommunikationsmodul_test.cpp
 *
 * Created: 4/2/2016 5:14:27 PM
 *  Author: marhi386
 */ 


//Sätt till rätt hastighet
#ifndef F_CPU
#define F_CPU 14745600UL
#endif

#define DEBUGMODE

#include "uart_master.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>

#include <string.h>


//*********Global Variabel************************************

//SPI

//Sens
#define spi_sensBuffer_max 20
volatile uint8_t spi_sensbuffer[spi_sensBuffer_max];
volatile uint8_t spi_sensCtr = 0;
volatile bool spi_sensRx_flag = false;

//Styr
#define spi_controlBuffer_max 20
volatile uint8_t spi_controlBuffer[spi_controlBuffer_max];
volatile uint8_t spi_controlCtr = 0;
volatile bool spi_controlRx_flag = false;

//*********INIT-funktioner************************************

//TEST INIT
void test_init()
{
	DDRA = 0xff;
}


// Initiera Masterenhet
void spi_init_master()
{
	// Sätt MOSI, SCK, SS0(styr) och SS1(sens) som output
	DDRB = (1<<5)|(1<<7)|(1<<4)|(1<<0);
	
	// Aktivera SPI, Sätt till Master
	//SCK-frekvens: Fosc/16
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0)|(1<<SPR1); 
	
	MCUCR |= (1<<1)|(1<<3); //Sätt negativ flank
	GICR |= (1<<INT0)|(1<<INT1); //Aktivera avbrott int0, int1
}

////////////SPI//////////////////////

void select_slave(int slave_nr)
{
	if (slave_nr == 2) //Ingen
	{
		PORTB |= (1<<4)|(1<<0);
	}
	else if (slave_nr == 1) //Styr
	{
		PORTB &= ~(1<<0);
		PORTB |= 1<<4;
	}
	else if (slave_nr == 0) //Sens
	{
		PORTB &= ~(1<<4);
		PORTB |= 1<<0;
	}
}

//Funktion för att skicka och ta emot 8 bitar
unsigned char spi_WriteRead_char (unsigned char data, int slave_nr)
{
	//sens = 0, styr = 1, ingen = 2
	select_slave(slave_nr);
	//Ladda in data som ska skickas
	SPDR = data;
	
	//Vänta tills överföring är klar
	while(!(SPSR &(1<<SPIF) ));
	
	//Returnera mottagen data
	return(SPDR);
	select_slave(2); //koppla från
}

////////Tolka kommandon//////////


void spi_sensProcess()
{
	switch(spi_sensbuffer[spi_sensCtr-1])
	{
		case '!':
		spi_sensCtr = 0;
		break;
		
		case 'B':
		PORTA = 0xff;
		uart_sendChar('B');
		spi_sensCtr = 0;
		break;
	}
}

void spi_controlProcess()
{
	uart_sendChar(spi_controlBuffer[spi_controlCtr]);
	switch(spi_controlBuffer[spi_controlCtr-1])
	{
		case '!':
		spi_controlCtr = 0;
		break;
		
		case 'A':
		PORTA = 0x0f;
		spi_WriteRead_char('B', 1);
		spi_sensCtr = 0;
		break;
	}
}

//******************Fråga efter data från slavar**********

//Be sens att skicka sensordata
void get_sensorData()
{
	unsigned char command = 'G';
	spi_WriteRead_char(command, 0);
}

//******************Avbrottsrutiner***********************
ISR(USART_RXC_vect)
{
	uint8_t dataRecieved = UDR;
	uart_rx_flag = true;
	if(uart_ctr == uart_buffer_max) // Är bufferräknaren vid max, nollställ
	{
		uart_ctr = 0;
	}
	uart_buffer[uart_ctr++] = dataRecieved;	
	
	#ifdef DEBUGMODE
	switch(dataRecieved)
	{
		case 'U': uart_sendString("Forward", 0);
		case 'D': uart_sendString("Backward", 0);
	}
	#endif
	
}

//Avbrott från sens
ISR(INT0_vect)
{
	if(spi_sensCtr == spi_sensBuffer_max) // Är bufferräknaren vid max, nollställ
	{
		spi_sensCtr = 0;
	}
	spi_sensbuffer[spi_sensCtr++] = spi_WriteRead_char('!', 0);
	spi_sensRx_flag = true;
}

//Avbrott från styr
ISR(INT1_vect)
{
	if(spi_controlCtr == spi_controlBuffer_max) // Är bufferräknaren vid max, nollställ
	{
		spi_controlCtr = 0;
	}
	spi_controlBuffer[spi_controlCtr++] = spi_WriteRead_char('!', 1);
	spi_controlRx_flag = true;
}

//********************************************************

void delay(int time)
{
	while (time > 0)
	{
		time--;
		delay(1);
	}
}

//******************MAIN**********************************
// SS: 1 = styr, 0 = sens
int main(void)
{	
	spi_init_master();
    uart_init();
	test_init();
	
	sei();
	
	while(1)
    {
		if (uart_rx_flag)// Om vi mottagit data via uart
		{
			uart_process();
			uart_rx_flag = false;
		}			
		if (spi_sensRx_flag)
		{
			spi_sensProcess();
			spi_sensRx_flag = false;
		}
		if (spi_controlRx_flag)
		{	
			spi_controlProcess();
			spi_controlRx_flag = false;
		}
	}
}