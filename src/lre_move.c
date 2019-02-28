/*
 * lre_move.c
 *
 *  Created on: 03.02.2019
 *      Author: Stefan
 */

#include "lre_move.h"
#include "lre_leds.h"
// -------------------- Abstracted Stuff (Move Stuff) -------------------------

#include "math.h"
#include "lre_stepper.h"
#include "lre_controler.h"

#define LRE_MOVE_DISTANCE_BETWEEN_WHEELS_MM	92	//distance between the middle of the wheels in mm
#define LRE_MOVE_DEFAULT_SPEED 60

// -------------------- functions -------------------------

//positive degree is a rotation to the left!
void lre_move_rotate(int16_t degree) {
	moveMode = MOVE_ACTIVE;
	int16_t distanceToTravel = degree * LRE_MOVE_DISTANCE_BETWEEN_WHEELS_MM
			* M_PI / 360.0;
	int8_t speed = LRE_MOVE_DEFAULT_SPEED;
	if (distanceToTravel < 0) {
		speed *= -1;
	}
	lre_stepper_setMaxDistance(distanceToTravel, STEPPER_RIGHT);
	lre_stepper_setMaxDistance(-distanceToTravel, STEPPER_LEFT);
	lre_stepper_setSpeed(speed, STEPPER_RIGHT);
	lre_stepper_setSpeed(-speed, STEPPER_LEFT);
	while (!lre_stepper_idle(STEPPER_BOTH)) {
		//wait till stepper is finished
	}
	moveMode = MOVE_IDLE;
}

void lre_move_distance(int16_t distance) {
	moveMode = MOVE_ACTIVE;
	if (distance > 0) {
		lre_stepper_setMaxDistance(distance, STEPPER_BOTH);
		lre_stepper_setSpeed(LRE_MOVE_DEFAULT_SPEED, STEPPER_BOTH);
	} else {
		lre_stepper_setMaxDistance(distance, STEPPER_BOTH);
		lre_stepper_setSpeed(-LRE_MOVE_DEFAULT_SPEED, STEPPER_BOTH);
	}
	while (!lre_stepper_idle(STEPPER_BOTH)) {
		//wait till stepper is finished
	}
	moveMode = MOVE_IDLE;
}

void lre_move_speed(int8_t speed) {
	moveMode = MOVE_ACTIVE;
	lre_stepper_setMaxDistance(0, STEPPER_BOTH);
	lre_stepper_setSpeed(speed, STEPPER_BOTH);
}

void lre_move_stop() {
	lre_stepper_stop(STEPPER_BOTH);
	moveMode = MOVE_IDLE;
	//TODO controller stop???
}

/* move_straight setzt die Parameter des controller_structs auf von aussen gewuenschte Werte
 *
 */

void lre_move_straight(int16_t speed, int16_t desired_distance,
		int16_t wall_distance, int16_t front_distance) {

	moveMode = MOVE_ACTIVE;
	nextCellVisible = NOT_VISIBLE;
	controller.controller_speed = speed;
	controller.controller_desired_distance = desired_distance;
	controller.wall_distance = wall_distance;
	controller.front_distance = front_distance;
	lre_controller_start();
}

uint8_t lre_move_idle()
{
	if (moveMode == MOVE_ACTIVE)
	{
		return FALSE;
	}
	return TRUE;
}

uint8_t lre_move_nextCellVisible()
{
	if (nextCellVisible)
	{
		return TRUE;
	}
	return FALSE;
}
