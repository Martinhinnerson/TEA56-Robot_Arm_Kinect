/*
 * servos.h
 *
 * Created: 4/20/2016 2:34:45 PM
 *  Author: timfo734
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>

#ifndef SERVOS_H_
#define SERVOS_H_

extern unsigned int servos[6];

uint8_t checksum_send(uint8_t id, uint8_t instruction, uint8_t adress, uint8_t par_length, char* parameters);
uint8_t checksum_rec(uint8_t id, uint8_t length, char* parameters);

void set_angle(uint8_t id, int angle, int speed);
void set_angle_index(uint8_t id, unsigned int angle_index, int angle_speed);

void send_info(uint8_t id, uint8_t instruction, uint8_t adress, char* parameters, int par_length);
unsigned int read_info(uint8_t id, uint8_t adress, char* parameters);
void delay_u(int delay_time);

void calc_limit(uint8_t id, unsigned int upper_lim, unsigned int lower_lim, bool turn);
void rotate(uint8_t id, bool turn);
void command_cases(char command); 

void set_respond(uint8_t id, uint8_t resp);
void reset(uint8_t id);
void set_id(uint8_t id, uint8_t new_id);

unsigned int get_angle(uint8_t id);
unsigned int get_index(uint8_t id);
unsigned int get_temp(uint8_t id);
unsigned int get_return_time(uint8_t id);

unsigned int goal_pos_12(int angle);
unsigned int goal_pos_2464(int angle);
unsigned int convert_degrees_12(unsigned int index);
unsigned int convert_degrees_2464(unsigned int index);
unsigned int calc_speed(int speed);
void starting_positions();
void starting_positions_index();
void update_angles_index_start();
void update_angles_start();

#endif /* SERVOS_H_ */