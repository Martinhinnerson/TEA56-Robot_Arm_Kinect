#ifndef UART_MASTER
#define UART_MASTER
//***************uart_master.h*****************************

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>

// Definiera BAUD
#define BAUD 115200UL
#include <util/setbaud.h>
// Sätt Baudratevärde för UBRR
#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)

//UART
#define uart_buffer_max 20 //Storlek på buffer
volatile uint8_t uart_buffer[uart_buffer_max]; //Buffer där senast mottagna datan ligger
volatile uint8_t uart_ctr = 0; // håller reda på var i buffern vi är
volatile bool uart_rx_flag = false; //true om vi fått ett nytt värde i buffern

void uart_init();
void uart_sendChar(unsigned char data);
void uart_sendString(const char *string, int delay);
unsigned char uart_readChar();
void uart_process();

#endif