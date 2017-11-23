/*
 * Styrmodul_servos.cpp
 *
 * Created: 4/4/2016 12:19:49 PM
 *  Author: timfo734
 */ 

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define BAUD 1000000UL
#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)
//#define BAUDRATE 8

// Definiera BAUD
#define BAUDFF 115200UL
//#define BAUDRATEFF ((F_CPU)/(BAUDFF*16UL)-1)
#define BAUDRATEFF 8


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <util/setbaud.h>
#include <string.h>

//UART
const uint8_t uart_buffer_max = 20; //Storlek på buffer
//volatile uint8_t uart_buffer[uart_buffer_max]; //Buffer där senast mottagna datan ligger
//volatile uint8_t uart_ctr = 0; // håller reda på var i buffern vi är
volatile char uart_command;
volatile bool uart_rx_flag = false; //true om vi fått ett nytt värde i buffern

//TEST
const uint8_t uart_buffer_max1 = 20; //Storlek på buffer
volatile uint8_t uart_buffer1[uart_buffer_max]; //Buffer där senast mottagna datan ligger
volatile uint8_t uart_ctr1 = 0; // håller reda på var i buffern vi är
volatile bool uart_rx_flag1 = false; //true om vi fått ett nytt värde i buffern

//Variabler för armen
unsigned int servos[6];
unsigned int arm_speed = 1;

#define TXRXcontrol PORTB

/* 
	Fördröjning i millisekunder
	Inargument: antal millisekunder
*/
void delay(int delay_time)
{
	for(int i = 0; i<delay_time;i++)
	{
		_delay_us(1);
	}
}

void ClearBuffer()
{
	uart_ctr1 = 0;
	uart_rx_flag1
}

void TXoffRXon()
{
	TXRXcontrol |= (1<<PORTB0);
	TXRXcontrol |= (1<<PORTB5);			// RIKTIGA ENDAST EN STYR!!!!!
}

void TXonRXoff()
{
	TXRXcontrol &= ~(1<<PORTB5);	
	TXRXcontrol &= ~(1<<PORTB0);
	
}

void TXcomplete()
{
	while (!(UCSR1A & (1<<TXC1))) {}
}

void uart_init()
{
	// Skifta registret höger 8 bitar
	UBRR0H = (BAUDRATEFF>>8);

	// Sätt baudrate
	UBRR0L = BAUDRATEFF;

	// Aktivera mottagare och sändare
	UCSR0B|= (1<<TXEN0)|(1<<RXEN0)|(1<<RXCIE0);

	// Välj 8-bitars dataformat
	UCSR0C|= (1<<UCSZ01)|(1<<UCSZ00);
}

void uart_sendCharFF(unsigned char data)
{
	// Vänta tills registret är ledigt
	while (!( UCSR0A & (1<<UDRE0)));

	// Ladda in data i registret
	UDR0 = data;
}

void uart_sendIntFF(unsigned int data)
{
	unsigned char high_value = (unsigned char)(data>>8);
	unsigned char low_value = data & 0xff;
	uart_sendCharFF(high_value);
	uart_sendCharFF(low_value);
}

// Funktion för att initiera UART
void init_servo(void)
{
	DDRB |= (1<<PORTB0) | (1<<PORTB5);
	
	// Skifta registret höger 8 bitar
	UBRR1H = (BAUDRATE>>8);
	
	// Sätt baudrate	
	UBRR1L = BAUDRATE;

	// Aktivera mottagare och sändare	
	UCSR1B |= (1<<TXEN1)|(1<<RXEN1) |(1<<RXCIE1);

	// Välj 8-bitars dataformat
	UCSR1C |= (1<<UCSZ11)|(1<<UCSZ10);	
}

// Funktion för att skicka data
void uart_sendChar(unsigned char data)
{
	// Vänta tills registret är ledigt
	while (!( UCSR1A & (1<<UDRE1)));
	
	// Ladda in data i registret
	UDR1 = data;
}


// Funktion för att ta emot data
unsigned char uart_readChar()
{
	// Vänta tills data har mottagits
	while(!(UCSR1A) & (1<<RXC1));
	
	// Returnera 8 bitar data
	return UDR1;
}

// Avbrottsvevktorn för fireflykommunikation
ISR(USART0_RX_vect)
{
	uart_command = UDR0;
	uart_rx_flag = true;
}

// Avbrottsvektorn för servokommunikation
ISR(USART1_RX_vect)
{
	/*if(uart_ctr1 == uart_buffer_max1) // Är bufferräknaren vid max, nollställ
	{
		uart_ctr1 = 0;
	}*/
	if(uart_rx_flag1 == false)
	{
		uart_ctr1 = 0;
		uart_rx_flag1 = true;
	}
	uart_buffer1[uart_ctr1] = UDR1;
	//uart_sendCharFF(uart_buffer1[uart_ctr1]);
	uart_ctr1++;
	
}

// Funktion för att beräkna sista biten som ska skickas till servona		 
uint8_t checksum_send(uint8_t id, uint8_t instruction, uint8_t adress, uint8_t par_length, char* parameters)
{
	//+ 1 för addres egentligen ska vara i parameters
	int length = par_length + 3;
	int temp = 0;
	for(int i = 0; i < par_length; i++)
	{
		temp += parameters[i];
	}
	return 255-((id+length+adress+instruction+temp)%256);
}


//Funktion för att beräkna check-summan vid mottagen data från servo
uint8_t checksum_rec(uint8_t id, uint8_t length, char* parameters)
{
	int temp = 0;
	for(int i = 0; i<length-2;i++)
	{
		temp += parameters[i];
	}
	return 255-((id+length+temp)%256);
}

//Vinkelberäkning för AX12
unsigned int goal_pos_12(int angle)
{
	return angle/0.2935;
}

//Vinkelberäkning för MX24/64
unsigned int goal_pos_2464(int angle)
{
	return angle/0.08789;	
}

//Konvertera från servoindex till grader för servo 12
unsigned int convert_degrees_12(unsigned int index)
{
	return index*0.2935;
}

//Konvertera från servoindex till grader för servo 24 - 64
unsigned int convert_degrees_2464(unsigned int index)
{
	return index*0.08789;
}
 
//Beräkna hastigheten
unsigned int calc_speed(int speed)
{
	return speed/0.111;
}

/* 
	Skicka information till servot enl. standard
	Inargument: id, instruktion, adress, parametrar, längden av parameters
*/
void send_info(uint8_t id, uint8_t instruction, uint8_t adress, char* parameters, int par_length)
{
	uint8_t length =  par_length + 3;

	char temp[length+4];
	
	//Startbitar
	temp[0] = 0xFF;
	temp[1] = 0xFF;
	
	// ID, längd, instruktion, adress, sen parametrar
	temp[2] = id;
	temp[3] = length;
	temp[4] = instruction;
	temp[5] = adress;
	
	// Lägg till alla parametrar + checksum i temp
	for(uint8_t i = 0; i < par_length + 1 ; i++)
	{
		if(i ==  par_length)
		{
			temp[i + 6] = checksum_send(id, instruction, adress, par_length, parameters);
		}
		else
		{
			temp[i + 6] = parameters[i];
		}
	}
	
	cli();
	// Skicka informationssträngen
	for(int i = 0; i < length + 4; i++)
	{
		uart_sendChar(temp[i]);
	}
	sei();
	delay(150);
}	

/* 
	Sätter vinkeln på servot
	Inargument: id, vinkel, hastighet (behövs inte)
*/
void set_angle(uint8_t id, int angle, int speed = 0.022)
{
	// För att skriva är instruktionen 0x03
	// För att programmera vinkel är adressen 0x1E
	uint8_t instruction = 0x03;
	uint8_t adress = 0x1E;
	int temp_angle;
	
	/* Beräkna vinkel
	Om id = 5 eller 6 så har vi ett AX-12 servo
	Om id = 1,2,3,4 har vi MX24/64 */
	if((id == 5) | (id == 6))
	{
		temp_angle = goal_pos_12(angle);
	}
	else if((id == 1) | (id == 2) | (id == 3) | (id == 4))
	{
		
		temp_angle = goal_pos_2464(angle);	
	}
	else
	{
		//Felaktigt ID på servot -> avbryt
		return;
	}
	
	// Skifta in de första 8 bitarna till high_angle och de sista till low_angle
	unsigned char high_angle = (unsigned char)(temp_angle>>8);
	unsigned char low_angle  = temp_angle & 0xff;
	
	//Beräkna hastighet & skifta in bitarna pss som för vinklarna
	unsigned int temp_speed = calc_speed(speed);
	unsigned char high_speed = (unsigned char)(temp_speed>>8);
	unsigned char low_speed  = temp_speed & 0xff;
	
	//Sätt in i listan
	char temp[] = {low_angle, high_angle, low_speed, high_speed};

	//Anropa send funktionen
	send_info(id, instruction, adress, temp, 4);
	TXcomplete();
	
}

void set_angle_index(uint8_t id, unsigned int angle_index, int angle_speed = 0.022)
{
	uint8_t instruction = 0x03;
	uint8_t adress = 0x1E;
	unsigned char high_angle_index = (unsigned char)(angle_index>>8);
	unsigned char low_angle_index  = angle_index & 0xff;
	unsigned int temp_speed = calc_speed(angle_speed);
	unsigned char high_speed = (unsigned char)(temp_speed>>8);
	unsigned char low_speed  = temp_speed & 0xff;
	char temp[] = {low_angle_index, high_angle_index, low_speed, high_speed};
	send_info(id, instruction, adress, temp, 4);
	TXcomplete();
}
/* 
	Läser på en adress
	Inargument: id, adress och parameters (antal adresser som ska läsas)
	parameters kan max vara 2.
*/
unsigned int read_info(uint8_t id, uint8_t adress, char* parameters)
{
	// Max 2 bytes åt gången
	char num_param = parameters[0];
	if(num_param > 2)
	{
		// För test returnera något som ger värde över 300 grader
		return 0x486; // = 340 grader för ax 12
	}
	// Läsning
	uint8_t instruction = 0x02;
	//Skicka till servot att vi vill läsa från adressen med num_param antal parametrar
	send_info(id, instruction, adress, parameters, 1);
	
	//Vänta tills sista byten är skickad för att byta till RX
	TXcomplete();
	TXoffRXon();
								
	//Mottagen data
	char reccived_data[num_param+6];
	
	//Hämta in 6 bytes + antalet parametrar
	while (uart_ctr1 < num_param+6) {} 

	for(int i = 0; i < num_param+6; i++)
	{
		reccived_data[i] = uart_buffer1[i];
		//uart_sendCharFF(reccived_data[i]);
	}
	uart_rx_flag1 = false;
	//Kunna skicka till check_sum
	char received_param[num_param];
	//Det vi ska returnera
	unsigned int index = 0;
	
	
	if(num_param == 1)
	{
		received_param[0] = reccived_data[num_param+4];
		index = reccived_data[num_param+4];
	}
	else if(num_param == 2)
	{
		//Börja leta här, troligtvis denna som gör att checksum blir fel
		received_param[0] = reccived_data[num_param + 3];
		received_param[1] = reccived_data[num_param + 4];
		char low = reccived_data[num_param+3];
		char high = reccived_data[num_param+4];
		index = (high << 8) + low;
	}
	
	uint8_t rec_checksum = reccived_data[num_param+5];

	//checksum blir fel för get angle
	if(checksum_rec(id,reccived_data[3],received_param) != rec_checksum)
	{ 
		//uart_sendCharFF(checksum_rec(id,reccived_data[3],received_param));
		// För test returnera något som ger värde över 300 grader vid fel
		//uart_sendCharFF(0xEF); // = 340 grader för ax 12
	}
	TXonRXoff();
	
	return index;
}

/* 
	Returnerar vinkeln för servot
	Inargument: id
*/
unsigned int get_angle(uint8_t id)
{
	char temp[1] = {0x02};
	unsigned int index = read_info(id,0x24,temp);
	//uart_sendCharFF(0xFC);
	unsigned int degrees = 0;
	//uart_sendIntFF(index);

	if((id == 1) | (id == 2) | (id == 3) | (id == 4))
	{
		degrees = convert_degrees_2464(index);
	}
	else if ((id == 5) | (id == 6))
	{
		degrees = convert_degrees_12(index);
	}
	else
	{
		// FEL ID
	}
	return degrees;
}

/* 
	Returnerar vinkel_index
	Inargument: id	
*/
unsigned int get_index(uint8_t id)
{
	char temp[1] = {0x02};
	return read_info(id,0x24,temp);
}


/* 
	Returnerar temperatur i celcius
	Inargument: ID på servo
*/
unsigned int get_temp(uint8_t id)
{
	char temp[1] = {0x01};
	unsigned int index = read_info(id,0x2B,temp);
	return index;
}


/* 
	Sätter responstiden för servot
	Inargument: id och responstid
*/
void set_respond(uint8_t id, uint8_t resp)
{
	char temp[1];
	temp[0] = resp;
	send_info(id,0x03,0x05,temp,1);
}

/* 
	Returnerar responstid
	Inargument: id 
*/
unsigned int get_return_time(uint8_t id)
{
	char temp[1] = {0x01};
	unsigned int index = read_info(id,0x05,temp);
	return index;
}

/* 
	Reset för servo #id
	Inargument: id 
*/
void reset(uint8_t id)
{
	char temp[6];
	temp[0] = 0xFF;
	temp[1] = 0xFF;
	temp[2] = id;
	temp[3] = 0x02;
	temp[4] = 0x06;
	temp[5] = checksum_send(id,0x02,0x06,0,0);
	
	// Skicka informationssträngen
	for(int i = 0; i < 6; i++)
	{
		uart_sendChar(temp[i]);
	}
}

/* 
	Sätter #id på servo
	Inargument: id och nytt id
*/
void set_id(uint8_t id, uint8_t new_id)
{
	char temp[1];
	temp[0] = new_id;
	send_info(id,0x03,0x03,temp,1);
}


void update_angles_start()
{
	servos[0] = 170;
	servos[1] = 135;
	servos[2] = 180;
	servos[3] = 135;
	servos[4] = 90;
	servos[5] = 200;
}

void update_angles_index_start()
{
	servos[0] = 1930;
	servos[1] = 1535;
	servos[2] = 2045;
	servos[3] = 1435;
	servos[4] = 305;
	servos[5] = 680;
}

void starting_positions()
{
	set_angle(2,135);
	set_angle(3,180);
	set_angle(4,135);
	set_angle(6,200);
	set_angle(5,90);
	set_angle(1,170);
}

void starting_positions_index()
{
	set_angle_index(2,1535);
	set_angle_index(3,2045);
	set_angle_index(4,1435);
	set_angle_index(6,680);
	set_angle_index(5,305);
	set_angle_index(1,1930);
}

void calc_limit(uint8_t id, unsigned int upper_lim, unsigned int lower_lim, bool turn)
{
	for(unsigned int i = 0; i < arm_speed; i++)
	{
		if(turn == true && (servos[id-1] < upper_lim ))
		{
			if((id == 5) | (id == 6))
			{
				servos[id-1] += 1;
			}
			else
			{
				servos[id-1] += 5;
			}
		}
		else if(turn == false && (servos[id-1] > lower_lim))
		{
			if((id == 5) | (id == 6))
			{
				servos[id-1] -= 1;
			}
			else
			{
				servos[id-1] -= 5;
			}
		}
	}
}

void rotate(uint8_t id, bool turn)
{
	switch(id)
	{
		//Servo 1
		case 1:
		calc_limit(1, 4095, 0, turn);
		break;
		
		//Servo2
		case 2:
		calc_limit(2, 4095, 0, turn);
		break;
		
		//Servo3
		case 3:
		calc_limit(3, 4095, 0, turn);
		break;
		
		//Servo4
		case 4:
		calc_limit(4, 4095, 0, turn);
		break;
		
		//Servo5
		case 5:
		calc_limit(5, 1023, 0, turn);
		break;
		
		//Servo6
		case 6:
		calc_limit(6, 1023, 0, turn);
		break;
	}
	set_angle_index(id,servos[id-1]);
}

void cases(char command)
{
		switch(command) 
		{
			//False moturs, True medurs
			//Servo 1
			case 'H': 
			rotate(1,false);
			break;
	
			case 'J':
			rotate(1,true);
			break;
			
			//Servo 2
			case 'K':
			rotate(2,false);
			break;
			
			case 'Z':
			rotate(2,true);
			break;
			
			//Servo 3
			case 'C':
			rotate(3,false);
			break;
			
			case 'V':
			rotate(3,true);
			break;
			
			//Servo 4
			case 'N':
			rotate(4,false);
			break;
			
			case 'M':
			rotate(4,true);
			break;
			
			//Servo 5
			case 'W':
			rotate(5,false);
			break;
			
			case 'E':
			rotate(5,true);
			break;
			
			//Servo 6
			case 'T':
			rotate(6,false);
			break;
			
			case 'Y':
			rotate(6,true);
			break;
			
			//Reset
			/*case 'X':
			starting_positions();
			break;
			
			case '213123123123' :
			arm_speed += +1;
			break;
			
			case '213123123123' :
			if (arm_speed == 1)
			{
				break;
			}
			armspeed += -1
			break;
			*/
		}
}



int main(void)
{
	uart_init();
	init_servo();
	TXonRXoff();
	sei();
	//delay(50);
	//int degrees = get_temp(5);
	
	//uart_sendIntFF(degrees);
	//starting_positions_index();
	
	starting_positions_index();
	delay(1000);
	update_angles_index_start();
	arm_speed = 3;
	while(1)
	{
		if(uart_rx_flag)// Om vi mottagit data via uart
		{
			uart_rx_flag = false;
			cases(uart_command);
		}
		delay(10000);
	}
	
	//delay(500);
	//uart_sendIntFF(get_angle(5));
	//uart_sendIntFF(get_temp(6));
	//set_angle(2,135);
	//set_angle(5,90);
	//TXoffRXon();
	
	
	
	//uart_sendCharFF(get_angle(5));
	//uart_sendIntFF(get_angle(1));
	//uart_sendCharFF(degrees);
	
	//set_angle(1,300);
	
	//set_angle(1,0);   
	//unsigned int degrees;
	//degrees = get_angle(1);
	//set_id(1,5);
	/*while(1)
	{
	set_angle(5,300);
	delay(1000);
	set_angle(5,0);
	delay(1000);
	}
	*/
	//reset(1);
	//degrees = read_info(1,0x05,temp);
	//uart_sendCharFF(degrees);
	//set_respond(1,temp);
	//char temp[1] = {0xA0};
	//send_info(1,0x03,0x05,temp,1);
	//delay(500);
	//degrees = get_return_time(1);
	//uart_sendCharFF(degrees);
	//char temp[1] = {0x01};
	//degrees = read_info(0x01,0x24,temp);
	//set_angle(1,240);
	
	/*
	set_angle(5,150);
	delay(500);
	set_angle(5,0);
	*/
	
	
	//uart_sendCharFF('A');

	
	/*
	
	set_angle(3,180);
	set_angle(2,135);
	
	set_angle(4,135);
	set_angle(6,200);
	set_angle(5,90);
	delay(1000);

	while(1)
	{
		set_angle(4,250);
		delay(1000);
		set_angle(4,120);
		delay(1000);
	}
	*/
	//set_angle(3,)
	

	/*
	degrees = get_angle(6);
	unsigned char high_angle = (unsigned char)(degrees>>8);
	unsigned char low_angle = degrees & 0xff;
	uart_sendCharFF(high_angle);
	uart_sendCharFF(low_angle);
	*/
	//TEST MED TEMPERATUR
	/*
	char resultat;
	resultat = get_temp(1);
	if(resultat > 10)
	{
		while(1)
		{
		set_angle(1,0);
		delay(1000);
		set_angle(1,300);
		delay(1000);
		set_angle(1,0);
		delay(1000);	
		}
	}
	*/
	/*
		char temp[4] = {0x00,0x08, 0x20, 0x00};
		send_info(0x01, 0x03, 0x1E, temp,4);
				
		char temp0[4] = {0x00,0x02,0x00,0x02};
		send_info(0x01,0x03, 0x1E, temp0,4); //sizeof(temp0)/sizeof(temp0[0])
		delay(500);
		char temp1[4] ={0xFF, 0x03,0x00,0x02};
		send_info(0x01,0x03,0x1E,temp1,4);
		delay(500);
	*/
	
		
		//delay(500);
		//char temp1[4] = {0xFF, 0x03, 0x00, 0x02};
		//send_info(0x01,0x03,0x1E,temp1,sizeof(temp1)/sizeof(temp1[0]));
		
		//TXoffRXon();
	
		//set_speed_angle(1,0);
		/*
		char temp[11] = {0xFF,0xFF,0x01,0x07,0x03,0x1E,0x00,0x02,0x00,0x02,0xD2};
		for(int i = 0; i < 9; i++)
		{
			uart_sendChar(temp[i]);
		}
			*/
		/*
		uart_sendChar(0xFF);
		uart_sendChar(0xFF);
		uart_sendChar(0x01);
		uart_sendChar(0x07);
		uart_sendChar(0x03);
		uart_sendChar(0x1E);
		uart_sendChar(0x00);
		uart_sendChar(0x02);
		uart_sendChar(0x00);
		uart_sendChar(0x02);
		uart_sendChar(0xD2);
		_delay_ms(500);
		
		uart_sendChar(0xFF);
		uart_sendChar(0xFF);
		uart_sendChar(0x01);
		uart_sendChar(0x07);
		uart_sendChar(0x03);
		uart_sendChar(0x1E);
		uart_sendChar(0xFF);
		uart_sendChar(0x03);
		uart_sendChar(0x00);
		uart_sendChar(0x02);
		uart_sendChar(0xD2);
		_delay_ms(500);
		
		uart_sendChar(0xFF);
		uart_sendChar(0xFF);
		uart_sendChar(0x01);
		uart_sendChar(0x07);
		uart_sendChar(0x03);
		uart_sendChar(0x1E);
		uart_sendChar(0x00);
		uart_sendChar(0x02);
		uart_sendChar(0x00);
		uart_sendChar(0x02);
		uart_sendChar(0xD2);
		_delay_ms(500);
		*/
}