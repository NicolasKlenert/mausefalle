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

// ---------------- constants ------------ //
#define MOVE_IDLE	0
#define MOVE_ACTIVE 1
#define THRESHOLD_FRONT 45 // in mm
#define THRESHOLD_SITE 42 // in mm


// ---------------- global variables------------ //

uint8_t moveMode; //use to signalise, if a moving is occouring or not

uint8_t control_flag; // use in controllerHandler to check if abort criteria is met

// ---------------- Move Stuff ------------ //

void lre_move_rotate(int8_t degree);
//only for commands
void lre_move_distance(int16_t distance);
void lre_move_speed(int8_t speed);
void lre_move_stop();
void lre_move_straight(int16_t speed, int16_t distance, int16_t wallDistance);

// ---------------- Controller Stuff -------- //

// function prototypes
void lre_controller_init();
void lre_controller_start();
void lre_controller_stop();
#endif /* LRE_MOVE_H_ */
