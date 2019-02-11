/*
 * lre_controler.c
 *
 *  Created on: 04.02.2019
 *      Author: Stefan
 */
#include "lre_controler.h"

// ---------------- control parameter (hand tuned) ------------ //
#define K_P 0.1
#define K_D 0
#define K_I 0

void lre_controller_leftWall(void)
{
	controller.error = (int16_t)mouse_distance[2] - controller.wall_distance;
	controller.corrector= K_P * controller.error + K_D * (controller.error-controller.differential) + K_I * (controller.error+controller.integral);
	controller.differential = controller.error;
	controller.integral=controller.integral + controller.error;

	//lre_stepper_setSpeed(controller.controller_speed-controller.error, STEPPER_LEFT, controller.controller_desired_distance);
	lre_stepper_setSpeed(controller.controller_speed+controller.corrector, STEPPER_RIGHT, controller.controller_desired_distance);
}

void lre_controller_rightWall(void)
{
	controller.error = (int16_t)mouse_distance[1] -controller.wall_distance;
	controller.corrector= K_P * controller.error + K_D * (controller.error-controller.differential)+ K_I * (controller.error+controller.integral);;
	controller.differential = controller.error;
	controller.integral=controller.integral + controller.error;

	lre_stepper_setSpeed(controller.controller_speed+controller.corrector, STEPPER_LEFT, controller.controller_desired_distance);
	//lre_stepper_setSpeed(controller.controller_speed-controller.error, STEPPER_RIGHT, controller.controller_desired_distance);
}

void lre_controller_bothWalls(void)
{	// wie controller_leftWall
	controller.error = (int16_t)mouse_distance[2] -controller.wall_distance;
	controller.corrector= K_P * controller.error + K_D * (controller.error-controller.differential)+ K_I * (controller.error+controller.integral);;
	controller.differential = controller.error;
	controller.integral=controller.integral + controller.error;

	//lre_stepper_setSpeed(controller.controller_speed-controller.error, STEPPER_LEFT, controller.controller_desired_distance);
	lre_stepper_setSpeed(controller.controller_speed+controller.corrector, STEPPER_RIGHT, controller.controller_desired_distance);
}

uint8_t SensorMagic( int8_t mouseSide)
{
	// Variables
	uint8_t height;
	int8_t dl;
	int8_t dr;
	int16_t momentpol2Mouse;
	int16_t momentpol2Sensor;
	int8_t angleMomentpol2Sensor;
	int8_t DeltaYSensor;
	int8_t DeltaYRad;
	int8_t SensorCorrect;
	int16_t Wandabschnitt;
	float beta;
	float theta;

// Geometrie der Roboterbewegung

	dl = lre_stepper_getMovedDistance(STEPPER_LEFT) - controller.dlAlt;
	dr = lre_stepper_getMovedDistance(STEPPER_RIGHT) - controller.drAlt;
	theta = (dl-dr)/LRE_CONTROLER_BREITE;
	momentpol2Mouse = LRE_CONTROLER_BREITE*dr/(dl-dr);
	momentpol2Sensor = sqrt(square((momentpol2Mouse+LRE_CONTROLER_BREITE+LRE_CONTROLER_YSENSOR))+ square(LRE_CONTROLER_XSENSOR));
	angleMomentpol2Sensor = asin(LRE_CONTROLER_XSENSOR/momentpol2Sensor);
	DeltaYSensor = momentpol2Sensor*cos(theta+controller.thetaAlt - angleMomentpol2Sensor) - (momentpol2Mouse+LRE_CONTROLER_BREITE+LRE_CONTROLER_YSENSOR);
	DeltaYRad = (momentpol2Mouse+LRE_CONTROLER_BREITE)*(cos(theta+controller.thetaAlt) - 1);

// Sensorwerte Korrigieren
	SensorCorrect = controller.sensorAlt - DeltaYSensor;

// Höhe berechnen
	Wandabschnitt = sqrt(square(SensorCorrect) + square(mouse_distance[mouseSide]) - 2* mouse_distance[mouseSide]*SensorCorrect * cos(theta));
	beta = acos((square(SensorCorrect)+square(Wandabschnitt)-square(mouse_distance[mouseSide]))/(2* SensorCorrect * mouse_distance[mouseSide]));
	height = SensorCorrect*sin(beta) + DeltaYSensor + DeltaYRad;

	controller.thetaAlt = controller.thetaAlt + theta;
	controller.dlAlt = controller.dlAlt + dl;
	controller.drAlt = controller.drAlt + dl;
	controller.sensorAlt = SensorCorrect;
	return height;
}

int square(int b)
{
    int z;
    z = b*b;
    return(z);
}
