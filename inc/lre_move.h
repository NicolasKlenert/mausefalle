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

// ---------------- Move Stuff ------------ //

void lre_move_rotate(int16_t degree);
//only for commands
void lre_move_distance(int16_t distance);
void lre_move_speed(int8_t speed);
void lre_move_stop();
void lre_move_straight(int16_t speed, int16_t desired_distance, int16_t wall_distance, int16_t front_distance);

#endif /* LRE_MOVE_H_ */
