//***************uart_master.cpp*****************************

//Sätt till rätt hastighet
#ifndef F_CPU
#define F_CPU 14745600UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>
#include "uart_master.h"

// Funktion för att initiera UART
void uart_init()
{
	// Skifta registret höger 8 bitar
	UBRRH = (BAUDRATE>>8);

	// Sätt baudrate
	UBRRL = BAUDRATE;

	// Aktivera mottagare och sändare
	UCSRB|= (1<<TXEN)|(1<<RXEN)|(1<<RXCIE);

	// Välj 8-bitars dataformat
	UCSRC|= (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);
}


// Funktion för att skicka data
void uart_sendChar(unsigned char data)
{
	// Vänta tills registret är ledigt
	while (!( UCSRA & (1<<UDRE)));

	// Ladda in data i registret
	UDR = data;
}

void uart_sendString(const char *string, int delay)
{
	while(*string)
	{
		uart_sendChar(*string++);
		while(delay > 0)
		{
			_delay_ms(1);
			delay--;
		}
	}
}

// Funktion för att ta emot data
unsigned char uart_readChar()
{
	// Vänta tills data har mottagits
	while(!(UCSRA) & (1<<RXC));
	
	// Returnera 8 bitar data
	return UDR;
}

void uart_process()
{
	switch(uart_buffer[uart_ctr-1]) //Om kommandot vi fick in va...
	{
		// '!' nollstället buffer-räknaren, kan va bra om man inte längre bryr sig om det gamla i buffern
		case '!': uart_ctr = 0;
		break;
		
		////////Robotens rörelse///////////
		
		//Kör frammåt
		case 'U':
		spi_WriteRead_char('U', 1);
		uart_sendChar('U');
		uart_ctr = 0;
		break;
		
		//kör bakåt
		case 'D':
		spi_WriteRead_char('D', 1);
		uart_sendChar('D');
		uart_ctr = 0;
		break;
		
		//Stop
		case 'S':
		spi_WriteRead_char('S', 1);
		uart_sendChar('S');
		uart_ctr = 0;
		break;
		
		//Vänster
		case 'L':
		spi_WriteRead_char('L', 1);
		uart_sendChar('L');
		uart_ctr = 0;
		break;
		
		//Höger
		case 'R':
		spi_WriteRead_char('R', 1);
		uart_sendChar('R');
		uart_ctr = 0;
		break;
		
		//Rotera vänster
		case 'F':
		spi_WriteRead_char('F', 1);
		uart_sendChar('F');
		uart_ctr = 0;
		break;
		
		//Rotera höger
		case 'G':
		spi_WriteRead_char('G', 1);
		uart_sendChar('G');
		uart_ctr = 0;
		break;
		
		/////////////////////////////////////
	}
}