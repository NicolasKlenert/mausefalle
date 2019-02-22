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

// ---------------- variables ------------ //
// Reglerstruct
typedef struct{
	int16_t controller_speed;   			// mm/s
	int16_t controller_desired_distance;	// mm
	int16_t wall_distance;					// mm
	int16_t front_distance;					// mm
	int16_t e;								// mm
	int16_t corrector;						// mm/s
	int16_t differential;
	int16_t integral;

//	int16_t start_decelerating				// mm

//	float thetaAlt;
//	int16_t dlAlt;
//	int16_t drAlt;
//	int16_t sensorAlt;

}controller_struct;

controller_struct controller;

uint8_t control_flag; // use in controllerHandler to check if abort criteria is met

// ---------------- prototypes ------------ //

void lre_controller_init();
void lre_controller_start();
void lre_controller_stop();
void lre_controller_leftWall(void);
void lre_controller_rightWall(void);
void lre_controller_bothWalls(void);

#endif /* LRE_CONTROLER_H_ */
