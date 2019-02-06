/*
 * lre_controler.c
 *
 *  Created on: 04.02.2019
 *      Author: Stefan
 */
#include "lre_controler.h"
// ---------------- control parameter (hand tuned) ------------ //
#define K_P 0.03
#define K_D 0
#define K_I 0

void lre_controller_leftWall(void)
{
	controller.error = (int16_t)mouse_distance[2] - controller.wall_distance;
	controller.corrector= K_P * controller.error + K_D * (controller.error-controller.differential) + K_I * (controller.error+controller.integral);
	controller.differential = controller.error;
	controller.integral=controller.integral + controller.error;

	//lre_stepper_setSpeed(controller.controller_speed-controller.error, STEPPER_LEFT, controller.controller_desired_distance);
	lre_stepper_setSpeed(controller.controller_speed+controller.error, STEPPER_RIGHT, controller.controller_desired_distance);
}

void lre_controller_rightWall(void)
{
	controller.error = (int16_t)mouse_distance[1] -controller.wall_distance;
	controller.corrector= K_P * controller.error + K_D * (controller.error-controller.differential)+ K_I * (controller.error+controller.integral);;
	controller.differential = controller.error;
	controller.integral=controller.integral + controller.error;

	lre_stepper_setSpeed(controller.controller_speed+controller.error, STEPPER_LEFT, controller.controller_desired_distance);
	//lre_stepper_setSpeed(controller.controller_speed-controller.error, STEPPER_RIGHT, controller.controller_desired_distance);
}

void lre_controller_bothWalls(void)
{	// wie controller_leftWall
	controller.error = (int16_t)mouse_distance[2] -controller.wall_distance;
	controller.corrector= K_P * controller.error + K_D * (controller.error-controller.differential)+ K_I * (controller.error+controller.integral);;
	controller.differential = controller.error;
	controller.integral=controller.integral + controller.error;

	//lre_stepper_setSpeed(controller.controller_speed-controller.error, STEPPER_LEFT, controller.controller_desired_distance);
	lre_stepper_setSpeed(controller.controller_speed+controller.error, STEPPER_RIGHT, controller.controller_desired_distance);
}
