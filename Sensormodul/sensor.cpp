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