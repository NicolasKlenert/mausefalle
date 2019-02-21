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
#include "math.h"

// ---------------- constants ------------ //
#define LRE_CONTROLER_BREITE 88 // mm
#define LRE_CONTROLER_XSENSOR 51 // mm
#define LRE_CONTROLER_YSENSOR 0 // mm



// ---------------- prototypes ------------ //

void lre_controller_leftWall(void);
void lre_controller_rightWall(void);
void lre_controller_bothWalls(void);
int16_t actuating_variable_restiction(int16_t corrector, int16_t speed);
uint8_t SensorMagic(int8_t mouseSide);
int square(int b);

#endif /* LRE_CONTROLER_H_ */
