/*
 * lre_controler.h
 *
 *  Created on: 04.02.2019
 *      Author: Stefan
 */

#ifndef LRE_CONTROLER_H_
#define LRE_CONTROLER_H_

#include "mouse.h"
#include "lre_stepper.h"
#include "lre_move.h"


// ---------------- prototypes ------------ //

void lre_controller_leftWall(void);
void lre_controller_rightWall(void);
void lre_controller_bothWalls(void);

#endif /* LRE_CONTROLER_H_ */
