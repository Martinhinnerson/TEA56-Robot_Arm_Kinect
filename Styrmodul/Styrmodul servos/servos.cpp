/*
 * servos.cpp
 *
 * Created: 4/20/2016 2:34:18 PM
 *  Author: timfo734
 */ 

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#ifndef BAUD
#define BAUD 1000000UL
#endif

#include "uart_arm.h"
#include "servos.h"

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

// Funktion för att beräkna check-summan vid mottagen data från servo
uint8_t checksum_rec(uint8_t id, uint8_t length, char* parameters)
{
	int temp = 0;
	for(int i = 0; i<length-2;i++)
	{
		temp += parameters[i];
	}
	return 255-((id+length+temp)%256);
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
		uart_sendChar_arm(temp[i]);
	}
	sei();
	delay_u(150);
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

// Delay-funktion, tar in antal mikrosekunder
void delay_u(int delay_time)
{
	for(int i = 0; i<delay_time;i++)
	{
		_delay_us(1);
	}
}

/* 
	Sätter vinkeln på servot, finns för grader respektive index
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

// Funktion för att beräkna en ny vinkel för ett givet servo, utifrån givna gränser
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

// Avgör vilket servo som önskas styras
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

// Funktion för att kolla vilket kommando som tagits emot
/*
void command_cases(char command)
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
			
		}
}
*/

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
	Sätter #id på servo
	Inargument: id och nytt id
*/
void set_id(uint8_t id, uint8_t new_id)
{
	char temp[1];
	temp[0] = new_id;
	send_info(id,0x03,0x03,temp,1);
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
	Returnerar responstid
	Inargument: id 
*/
unsigned int get_return_time(uint8_t id)
{
	char temp[1] = {0x01};
	unsigned int index = read_info(id,0x05,temp);
	return index;
}

// Vinkelberäkning för AX12
unsigned int goal_pos_12(int angle)
{
	return angle/0.2935;
}

// Vinkelberäkning för MX24/64
unsigned int goal_pos_2464(int angle)
{
	return angle/0.08789;
}

// Konvertera från servoindex till grader för servo 12
unsigned int convert_degrees_12(unsigned int index)
{
	return index*0.2935;
}

//Konvertera från servoindex till grader för servo 24 - 64
unsigned int convert_degrees_2464(unsigned int index)
{
	return index*0.08789;
}

// Beräkna hastigheten
unsigned int calc_speed(int speed)
{
	return speed/0.111;
}

// Startposition för armen i grader
void starting_positions()
{
	set_angle(2,135);
	set_angle(3,180);
	set_angle(4,135);
	set_angle(6,200);
	set_angle(5,90);
	set_angle(1,170);
}

// Startposition för armen i index
void starting_positions_index()
{
	set_angle_index(2,1535);
	set_angle_index(3,2045);
	set_angle_index(4,1435);
	set_angle_index(6,680);
	set_angle_index(5,305);
	set_angle_index(1,1930);
}

// Uppdatera respektive servos vinklar i grader
void update_angles_start()
{
	servos[0] = 170;
	servos[1] = 135;
	servos[2] = 180;
	servos[3] = 135;
	servos[4] = 90;
	servos[5] = 200;
}

// Uppdatera respektive servos vinklar i index
void update_angles_index_start()
{
	servos[0] = 1930;
	servos[1] = 1535;
	servos[2] = 2045;
	servos[3] = 1435;
	servos[4] = 305;
	servos[5] = 680;
}