/*
 * lre_controler.c
 *
 *  Created on: 04.02.2019
 *      Author: Stefan
 */
#include "lre_controler.h"


// ---------------- control parameter (hand tuned) ------------ //
#define K_P 10
#define K_D 0
#define K_I 0

void lre_controller_leftWall(void)
{
	controller.e = mouse_distance[1] - controller.wall_distance;
	controller.integral=controller.integral + controller.e;
	controller.corrector= K_P * controller.e + K_D * (controller.e-controller.differential) + K_I * controller.integral;
	controller.differential = controller.e;

	controller.corrector = actuating_variable_restiction(controller.corrector, controller.controller_speed);

	lre_stepper_setSpeed(controller.controller_speed - controller.corrector, STEPPER_LEFT);
	lre_stepper_setSpeed(controller.controller_speed + controller.corrector, STEPPER_RIGHT);
}

void lre_controller_rightWall(void)
{
	controller.e = mouse_distance[2] - controller.wall_distance;
	controller.integral=controller.integral + controller.e;
	controller.corrector= K_P * controller.e + K_D * (controller.e-controller.differential) + K_I * controller.integral;
	controller.differential = controller.e;

	controller.corrector = actuating_variable_restiction(controller.corrector, controller.controller_speed);

	lre_stepper_setSpeed(controller.controller_speed + controller.corrector, STEPPER_LEFT);
	lre_stepper_setSpeed(controller.controller_speed - controller.corrector, STEPPER_RIGHT);
}

void lre_controller_bothWalls(void)
{	// wie controller_leftWall
	controller.e = mouse_distance[1] - controller.wall_distance;
	controller.integral=controller.integral + controller.e;
	controller.corrector= K_P * controller.e + K_D * (controller.e-controller.differential) + K_I * controller.integral;
	controller.differential = controller.e;

	controller.corrector = actuating_variable_restiction(controller.corrector, controller.controller_speed);

	lre_stepper_setSpeed(controller.controller_speed - controller.corrector, STEPPER_LEFT);
	lre_stepper_setSpeed(controller.controller_speed + controller.corrector, STEPPER_RIGHT);
}

/* This function restricts the corrector, so the max stepper speed will not be exceeded
 *
 * - Only checks in positive direction, because we will not drive backwards with controller on
 *
 * */
int16_t actuating_variable_restiction(int16_t corrector, int16_t speed)
{
	if (corrector + speed > STEPPER_MAX_SPEED)
	{
		return STEPPER_MAX_SPEED - speed;
	}
	else
	{
		return corrector;
	}
}

//uint8_t SensorMagic( int8_t mouseSide)
//{
//	// Variables
//	uint8_t height;
//	int8_t dl;
//	int8_t dr;
//	int16_t momentpol2Mouse;
//	int16_t momentpol2Sensor;
//	int8_t angleMomentpol2Sensor;
//	int8_t DeltaYSensor;
//	int8_t DeltaYRad;
//	int8_t SensorCorrect;
//	int16_t Wandabschnitt;
//	float beta;
//	float theta;
//
//// Geometrie der Roboterbewegung
//
//	dl = lre_stepper_getMovedDistance(STEPPER_LEFT) - controller.dlAlt;
//	dr = lre_stepper_getMovedDistance(STEPPER_RIGHT) - controller.drAlt;
//	theta = (dl-dr)/LRE_CONTROLER_BREITE;
//	momentpol2Mouse = LRE_CONTROLER_BREITE*dr/(dl-dr);
//	momentpol2Sensor = sqrt(square((momentpol2Mouse+LRE_CONTROLER_BREITE+LRE_CONTROLER_YSENSOR))+ square(LRE_CONTROLER_XSENSOR));
//	angleMomentpol2Sensor = asin(LRE_CONTROLER_XSENSOR/momentpol2Sensor);
//	DeltaYSensor = momentpol2Sensor*cos(theta+controller.thetaAlt - angleMomentpol2Sensor) - (momentpol2Mouse+LRE_CONTROLER_BREITE+LRE_CONTROLER_YSENSOR);
//	DeltaYRad = (momentpol2Mouse+LRE_CONTROLER_BREITE)*(cos(theta+controller.thetaAlt) - 1);
//
//// Sensorwerte Korrigieren
//	SensorCorrect = controller.sensorAlt - DeltaYSensor;
//
//// Höhe berechnen
//	Wandabschnitt = sqrt(square(SensorCorrect) + square(mouse_distance[mouseSide]) - 2* mouse_distance[mouseSide]*SensorCorrect * cos(theta));
//	beta = acos((square(SensorCorrect)+square(Wandabschnitt)-square(mouse_distance[mouseSide]))/(2* SensorCorrect * mouse_distance[mouseSide]));
//	height = SensorCorrect*sin(beta) + DeltaYSensor + DeltaYRad;
//
//	controller.thetaAlt = controller.thetaAlt + theta;
//	controller.dlAlt = controller.dlAlt + dl;
//	controller.drAlt = controller.drAlt + dl;
//	controller.sensorAlt = SensorCorrect;
//	return height;
//}
//
//int square(int b)
//{
//    int z;
//    z = b*b;
//    return(z);
//}
