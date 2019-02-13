/*
 * lre_move.h
 *
 *  Created on: 03.02.2019
 *      Author: Stefan
 */

#ifndef LRE_MOVE_H_
#define LRE_MOVE_H_

#include "stm32f0xx.h"
#include "main.h"
#include "mouse.h"
#include "lre_usart.h"

// ---------------- constants ------------ //
#define MOVE_IDLE	0
#define MOVE_ACTIVE 1
#define THRESHOLD_FRONT 45 // in mm
#define THRESHOLD_SITE 42 // in mm


// ---------------- global variables------------ //

uint8_t moveMode; //use to signalise, if a moving is occouring or not

uint8_t control_flag; // use in controllerHandler to check if abort criteria is met

// Reglerstruct
typedef struct{
	int16_t controller_speed;   			// mm/s
	int16_t controller_desired_distance;	// mm
	int16_t wall_distance;					// mm
	int16_t front_distance;
	int16_t error;
	int16_t corrector;
	int16_t differential;
	int16_t integral;
	float thetaAlt;
	int16_t dlAlt;
	int16_t drAlt;
	int16_t sensorAlt;

}controller_struct;

// variables
controller_struct controller;

// ---------------- Move Stuff ------------ //

void lre_move_rotate(int16_t degree);
//only for commands
void lre_move_distance(int16_t distance);
void lre_move_speed(int8_t speed);
void lre_move_stop();
void lre_move_straight(int16_t speed, int16_t desired_distance, int16_t wall_distance, int16_t front_distance);

// ---------------- Controller Stuff -------- //

// function prototypes
void lre_controller_init();
void lre_controller_start();
void lre_controller_stop();
#endif /* LRE_MOVE_H_ */
