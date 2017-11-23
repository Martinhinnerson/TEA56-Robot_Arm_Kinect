/*
 * uart_arm.cpp
 *
 * Created: 4/20/2016 2:48:07 PM
 *  Author: timfo734
 */ 

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#ifndef BAUD
#define BAUD 1000000UL
#endif
#define BAUDRATE_servo ((F_CPU)/(BAUD*16UL)-1)

#define TXRXcontrol PORTB

#include "uart_arm.h"

//*****************Funktioner***********************

// Initiera uart för kommunikation med arm
void init_uart_servo(void)
{
	DDRB |= (1<<PORTB0);
	
	// Skifta registret höger 8 bitar
	UBRR1H = (BAUDRATE_servo>>8);
	
	// Sätt baudrate
	UBRR1L = BAUDRATE_servo;

	// Aktivera mottagare och sändare
	UCSR1B |= (1<<TXEN1)|(1<<RXEN1) |(1<<RXCIE1);

	// Välj 8-bitars dataformat
	UCSR1C |= (1<<UCSZ11)|(1<<UCSZ10);
	
	// Initiera så att arm intar startposition
	uart_rx_flag1 = true;
	uart_command = 'X';
	arm_speed = 1;
	uart_ctr1 = 0;
}

// Tillåt armen att skicka data
void TXoffRXon()
{
	TXRXcontrol |= (1<<PORTB0);
}

// Tillåt processorn att skicka data
void TXonRXoff()
{
	TXRXcontrol &= ~(1<<PORTB5);	
}

// Återställ räknare och flagga för uart1
void ClearBuffer()
{
	uart_ctr1 = 0;
	uart_rx_flag1 = false;
}

// Vänta tills sändning av data klart
void TXcomplete()
{
	while (!(UCSR1A & (1<<TXC1))) {}
}

// Skicka data till arm via uart
void uart_sendChar_arm(unsigned char data)
{
	// Vänta tills registret är ledigt
	while (!( UCSR1A & (1<<UDRE1)));
	
	// Ladda in data i registret
	UDR1 = data;
}