/*
 * uart_arm.h
 *
 * Created: 4/20/2016 2:47:57 PM
 *  Author: timfo734
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <util/setbaud.h>
#include <string.h>

#ifndef UART_ARM_H_
#define UART_ARM_H_

// Initieras i cpp till 'X'
extern volatile char uart_command;
// Initieras i cpp till true
extern volatile bool uart_rx_flag1;
// Initieras till 20
const uint8_t uart_buffer_max1 = 20; //Storlek på buffer
// Uart buffer
volatile uint8_t uart_buffer1[uart_buffer_max1];
// Hastighet på förflyttning av arm i manuellt läge
extern unsigned int arm_speed;
// Räknare till buffern
extern uint8_t uart_ctr1;

void init_uart_servo();
void TXoffRXon();
void TXonRXoff();
void clearbuffer();
void TXcomplete();

void uart_sendChar_arm(unsigned char data);

#endif /* UART_ARM_H_ */