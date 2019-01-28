/*
 * communications.h
 *
 *  Created on: 24.01.2019
 *      Author: Stefan
 */

#ifndef LRE_COMMUNICATION_H_
#define LRE_COMMUNICATION_H_

// includes
#include "cmd.h"
#include "lre_stepper.h"
#include "lre_leds.h"
#include "lre_queue.h"
#include "mouse.h"
#include "main.h"


// defines
#define SPEED 60

// funcitons prototypes

void lre_communication_init();
void lre_move(int argc, char **argv);
void lre_telemetrie(int argc, char **argv);
void lre_maze_com(int argc, char **argv);


#endif /* LRE_COMMUNICATION_H_ */
