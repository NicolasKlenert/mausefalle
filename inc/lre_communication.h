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


// defines
#define SPEED 60

// funcitons prototypes

void lre_communication_init();
void lre_move(int argc, char **argv);


#endif /* LRE_COMMUNICATION_H_ */
