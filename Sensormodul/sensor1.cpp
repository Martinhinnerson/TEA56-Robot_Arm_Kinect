/*
 * GccApplication3.cpp
 *
 * Created: 4/8/2016 10:54:11 AM
 *  Author: timfo734
 */ 

//Sätt till rätt hastighet
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

// Definiera BAUD
#define BAUD 115200UL
#define BAUDRATE 8

#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/setbaud.h>
#include <string.h>
#include <stdlib.h>

//Sensor
unsigned char sensorData = 0b00110101; //mottagen sensordata
bool mission = false; //uppdrag startat?
bool calibrate_floor = false; 
bool calibrate_tape = false;
bool drive = false; //kollar att man lämnat starten för att kunna hitta målet
int floor_value; //sätts vid kalibrering av underlag
int tape_value; //sätts vid kalibrering av tejp
int threshold_value = 0; //tröskelvärde som sätts efter kalibrering
int index; //tyngdpunktsberäkning
int mass[7];

volatile bool conversion_complete = false;
volatile char lamps = 'C';

// Interrupt Vector plats: $0030, dvs ADC_vect, ADC Conversion Complete se sid 59'
// ADC0 används dvs pinne 40 till ADC conversion.
void init()
{
	//Reflexsensor
	
	//Använd  PA7 till output för signalerna
	DDRA = 0b10000000;
	
	/*ADC
	
	*ADCSRA* - ADC Control and Status Register A
	
	Bit 7 - ADEN: ADC enable.
	Writing this bit to one enables the ADC
	
	Bit 6 - ADSC: ADC Start Conversion. 
	In a single conversion mode, write this bit to one to start
	each conversion. In free running mode write this bit to one to start the first conversion.
	
	Bit 5 - ADATE: ADC Auto trigger Enable. 
	When this bit is written to one, Auto Triggering of the ADC
	is enabled. The ADC will start a conversion on a positive edge of the selected trigger signal.
	Trigger signal is set by selecting ADTS ADCSRB (ADC Trigger select bits)
	
	Bit 4 - ADIF: ADC Interrupt Flag. 
	This bit is set when an ADC conversion completes and the Data 
	Registers are updated. The ADC Conversion Complete Interrupt is executed, if the ADIE bit and 
	the I-bit in SREG are set. (Global interrupt enabled, sei())
	
	Bit 3 - ADIE: ADC Interreput Enable
	
	Bit 2:0 - ADPS2:0 ADC Prescaler bits (division factor), se sidan 257 - 258
	Klockfrekvens på 16MHz -> div.factor på 128 ty intevall på 50 - 200 kHz
	
	"By default, the successive approximation circuitry requires an input clock frequency 
	between 50kHz and 200 kHz to get maximum resolution" - sid 243
	
	*/
	ADCSRA |= (1<<ADEN) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); 
	
	/*
	
	*ADMUX* - ADC Multiplexer Selection Register
	
	Bit 7:6 - REFS1:0 - Reference Selection Bits
	These bits select the voltage reference for the ADC.
	
	Bit 5 - ADLAR - ADC Left Adjust Result
	Changes the presentation of the ADC conversion result in the ADC Data Register. 
	1 = Left adjusted result, otherwise it is right adjusted
	
	Bits 4:0 - MUX4:0: Analog Channel and Gain Selection Bits
	
	0000 -> Single Endeed Input ADC0, no gain
	
	The ADLAR bit in ADMUX, and the MUXn bits in ADMUX affect the way the result is read from
	the registers. If ADLAR is set, the result is left adjusted. If ADLAR is cleared (default),
	the result is right adjusted
	
	*/
	
	ADMUX |= (1<<REFS0) | (1<<ADLAR);
	
}



//UART
const int uart_buffer_max = 20; //Storlek på buffer
unsigned char uart_buffer[uart_buffer_max]; //Buffer där senast mottagna datan ligger
unsigned char uart_ctr = 0; // håller reda på var i buffern vi är
volatile bool uart_rx_flag = false; //true om vi fått ett nytt värde i buffern

//volatile uint8_t mode = 0;

// Funktion för att initiera UART
void uart_init()
{
	// Skifta registret höger 8 bitar
	UBRR0H = (BAUDRATE>>8);

	// Sätt baudrate
	UBRR0L = BAUDRATE;

	// Aktivera mottagare och sändare
	UCSR0B|= (1<<TXEN0)|(1<<RXEN0)|(1<<RXCIE0);

	// Välj 8-bitars dataformat
	UCSR0C|= (1<<UCSZ01)|(1<<UCSZ00);
}

//SPI
const int spi_buffer_max = 20;
volatile unsigned char spi_buffer[spi_buffer_max];
volatile unsigned char spi_ctr = 0;
volatile bool spi_rx_flag = false;

// Initiera Slavenhet
void spi_init_slave()
{
	DDRB = (1<<6); //MISO till OUTPUT
	SPCR = (1<<SPE)|(1<<SPIE); //Aktivera SPI och avbrott
	SPDR = 0; //Töm dataregister
	DDRD |= (1<<7); //avbrott till master
}

// Funktion för att skicka data
void uart_sendChar(unsigned char data)
{
	// Vänta tills registret är ledigt
	while (!( UCSR0A & (1<<UDRE0)));

	// Ladda in data i registret
	UDR0 = data;
}

// Funktion för att skicka data
void uart_sendNum(int data)
{
	// Vänta tills registret är ledigt
	while (!( UCSR0A & (1<<UDRE0)));

	// Ladda in data i registret
	UDR0 = data;
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
	while(!(UCSR0A) & (1<<RXC0));
	
	// Returnera 8 bitar data
	return UDR0;
}

////////SPI////////////

//Funktion för att skicka och ta emot data för master och slave
unsigned char spi_WriteRead_char (unsigned char data)
{
	//Ladda in data som ska skickas
	SPDR = data;
	
	//Vänta tills överföring är klar
	while(!(SPSR &(1<<SPIF) ));
	
	//Returnera mottagen data
	return(SPDR);
}

void spi_sendInterrupt()
{
	PORTD |= 1<<7;
	PORTD &= ~(1<<7);
}


void start_convert()
{
	ADCSRA |= (1<<ADSC);
}

void start_signal()
{
	PORTA |= (1<<7);
}

void stop_signal()
{
	PORTA &= ~(1<<7);
}

void select_input(int diod)
{
	ADMUX = (ADMUX & 11100000)|(diod-1);
}

// Kollar om man kommit fram till målet
void detect_stop(char *result)
{
	bool stop = true;
	for(int current_char = 0; current_char<7; current_char++)
	{
		if(result[current_char] == 'A')
		{
			stop = false; //om någon lampa inte är över tejpen
		}
	}
	
	if(stop && drive)
	{
		uart_sendChar('s'); // meddela styrmodul att mål är detekterat
		//spi_WriteRead_char('s'); // meddela komm.modul att mål är detekterat
		mission = false;
	}
}

// Kollar så man börjar vid starten
void detect_start()
{
	mission = true;
	start_signal();
	_delay_ms(50);
	for(int current_lamp = 1; current_lamp<8; current_lamp++)
	{
		select_input(current_lamp);
		start_convert();
		while(!conversion_complete) {	}
		conversion_complete = false;
		int temp = ADCH;
		if(temp < threshold_value)
		{
			mission = false; //om någon lampa inte är över tejpen
		}
	}
	drive = false;
}

////////Tolka kommandon//////////
void uart_process()
{
	switch(uart_buffer[uart_ctr-1]) //Om kommandot vi fick in va...
	{
		// '!' nollstället buffer-räknaren, kan va bra om man inte längre bryr sig om det gamla i buffern
		case '!': uart_ctr = 0;
		break;
		
		case 'B':
		PORTA = 0xff;
		SPDR = 'B';
		spi_sendInterrupt();
		uart_ctr = 0;
		break;
		
		case '5':
		//if(!mission)
		calibrate_floor = true;
		uart_ctr = 0;
		break;
				
		case '6':
		//if(!mission)
		calibrate_tape = true;
		uart_ctr = 0;
		break;
		
		case '7': //starta uppdrag
		detect_start();
		uart_ctr = 0;
		break;
		
		case '8':  //return home
		detect_start();
		uart_ctr = 0;
		break;
	}
}

void spi_process()
{
	switch(spi_buffer[spi_ctr-1])
	{
		// '!' nollstället buffer-räknaren, kan va bra om man inte längre bryr sig om det gamla i buffern
		case '!':
		spi_ctr = 0;
		break;
				
		case '5':
		//if(!mission)
		calibrate_floor = true;
		uart_ctr = 0;
		break;
		
		case '6':
		//if(!mission)
		calibrate_tape = true;
		uart_ctr = 0;
		break;
		
		case '7': //starta uppdrag
		detect_start();
		uart_ctr = 0;
		break;
		
		case '8':  //return home
		detect_start();
		uart_ctr = 0;
		break;
	}
}

//******************Avbrottsrutiner***********************
ISR(SPI_STC_vect)
{
	if(spi_ctr == spi_buffer_max) // Är bufferräknaren vid max, nollställ
	{
		spi_ctr = 0;
	}
	spi_buffer[spi_ctr++] = SPDR;
	spi_rx_flag = true;
}

ISR(USART0_RX_vect)
{
	//uint8_t temp;
	
	if(uart_ctr == uart_buffer_max) // Är bufferräknaren vid max, nollställ
	{
		uart_ctr = 0;
	}
	uart_buffer[uart_ctr++] = UDR0;	
	
	//temp = UDR0;
	uart_rx_flag = true;
}

ISR(ADC_vect)
{
	//Vad som händer när ADC är klar
	if(calibrate_floor)
	{
		floor_value = ADCH;
		calibrate_floor = false;
	}
	else if(calibrate_tape)
	{
		tape_value = ADCH;
		threshold_value = floor_value + (tape_value - floor_value)*0.75;
		calibrate_tape = false;
	}
	else if(mission)
	{
		int temp = ADCH;
		//int lamp = 7-current_lamp;
		if (temp >= threshold_value)
		{
				//lamps |= (1<<lamp);
				lamps = 'B';
		}
		else
		{
			lamps = 'A';
			drive = true;
		}
		mass[index] = temp;
	}
	conversion_complete = true;
}

int calculate_e()
{
		float k = 0;
		float p = 0;
		for(int i=1; i<8; i++)
		{
			k = k + mass[i-1]*i;
		}
		for(int i=1; i<8; i++)
		{
			p = p + mass[i-1];
		}
		k = k/p; //tyngdpunktsberäkning
		int e = 36*(4-k); //reglerfelet multipliceras med 36 för att behålla maximal noggrannhet
		return e;
}

int main(void)
{
	spi_init_slave();
	uart_init();
	init();
	char result[7]; //sensordata
	sei();
	
    while(1)
    {
		if(calibrate_floor || calibrate_tape)
		{
			start_signal();
			_delay_ms(50);
			select_input(4);
			start_convert();
			while(!conversion_complete) {	}
			conversion_complete = false;
		}	
		
		if(mission)
		{
			start_signal();
			_delay_ms(50);
			uart_sendChar('e');
			//spi_WriteRead_char('e');
			for(int current_lamp = 1; current_lamp<8; current_lamp++)
			{
				index = current_lamp - 1;
				select_input(current_lamp);
				start_convert();
				while(!conversion_complete) {	}
				conversion_complete = false;
				uart_sendChar(lamps);
				//spi_WriteRead_char(lamps);
				result[current_lamp - 1] = lamps;
			}
			//uart_sendString(result,5);
			//uart_sendChar('r');
			
			int temp = calculate_e();
			//unsigned char high = (unsigned char)(temp >> 8);
			//unsigned char low = temp & 0xFF;
			//uart_sendChar(high);
			//uart_sendChar(low);
			//convert_n_send(calculate_e()); //skicka reglerfelet till styrmodulen
			detect_stop(result);
		}
		else
		{
			stop_signal();
			_delay_ms(50);
		}
		
		if(uart_rx_flag)// Om vi mottagit data via uart
		{
			uart_rx_flag = false;
			uart_process();
		}
		if (spi_rx_flag)// Om vi mottagit data via spi
		{
			spi_rx_flag = false;
			spi_process();
		}
			
    }
}