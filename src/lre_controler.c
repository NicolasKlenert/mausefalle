/*
 * lre_controler.c
 *
 *  Created on: 04.02.2019
 *      Author: Stefan
 */
#include "lre_controler.h"
#include "lre_leds.h"
#include "lre_execution_time.h"
#include "labyrinth.h"
#include "lre_usart.h"
#include "stdio.h"


// ---------------- control parameter (hand tuned) ------------ //
#define K_P 0.5
#define K_D 0
#define K_I 0

// -------------------- (Controller Stuff) -------------------------
#define LRE_MOVE_CONTROLLER_FREQ 50	// Frequency of Controller in Hz
#define LRE_MOVE_TIMER_FREQ 10000 		// Frequency of TIM7 in Hz (minimum 734 Hz!!!)
#define LRE_MOVE_TIMER_PERIOD (uint16_t)(( LRE_MOVE_TIMER_FREQ / LRE_MOVE_CONTROLLER_FREQ ) - 1 )	// Period of TIM7

// -------------------- function prototypes ------------------------
int16_t actuating_variable_restiction(int16_t corrector, int16_t speed);
//uint8_t SensorMagic(int8_t mouseSide);
//int square(int b);

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
	else if (speed - corrector < 1)
	{
		return speed - 1;
	}
	else
	{
		return corrector;
	}
}

// Timer fuer die Regelung initialisieren
void lre_controller_init() {
	// RCC enable TIM7
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
	// RCC enable Sysconfig for interrupts
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	// TIM init struct
	TIM_TimeBaseInitTypeDef timerInitStruct;
	timerInitStruct.TIM_ClockDivision = 0;
	timerInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStruct.TIM_Period = LRE_MOVE_TIMER_PERIOD;
	timerInitStruct.TIM_Prescaler = SystemCoreClock / LRE_MOVE_TIMER_FREQ - 1;
	timerInitStruct.TIM_RepetitionCounter = 0;

	// TIM init
	TIM_TimeBaseInit(TIM7, &timerInitStruct);

	TIM_Cmd(TIM7, DISABLE);
	TIM_SetCounter(TIM7, 0);
	TIM_ClearITPendingBit(TIM7, TIM_IT_Update);

	// TIM7 enable update interrupt
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);

	// NVIC config and enable
	NVIC_InitTypeDef nvicTIM7;
	nvicTIM7.NVIC_IRQChannel = TIM7_IRQn;
	nvicTIM7.NVIC_IRQChannelCmd = ENABLE;
	nvicTIM7.NVIC_IRQChannelPriority = 2;	// can be 0 to 3
	NVIC_Init(&nvicTIM7);
}

// Timer fuer die Regelung anschalten
void lre_controller_start() {

	TIM_SetCounter(TIM7, 0);
	TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	TIM_Cmd(TIM7, ENABLE);
	control_flag = FALSE;
	controller.e = 0;
	controller.differential = 0;
	controller.integral = 0;

//	controller.thetaAlt = 0;
//	controller.dlAlt = 0;
//	controller.drAlt = 0;
//	controller.sensorAlt = controller.wall_distance;
}

// Timer fuer die Regelung abschalten
void lre_controller_stop() {
	// Disable TIM7
	TIM_Cmd(TIM7, DISABLE);
	lre_move_stop();
	control_flag = TRUE;
}

// Regelungsroutine Timer handler
void TIM7_IRQHandler(void) {
	lre_execution_time_tic();
	char str[50];

	// check which interrupt occurred
	if (SET == TIM_GetITStatus(TIM7, TIM_IT_Update)) {
		uint8_t rightWall = FALSE;
		uint8_t leftWall = FALSE;
		// get moved distance (average of both steppers)
		int16_t movedDistanceLeft = lre_stepper_getMovedDistance(STEPPER_LEFT);
		int16_t movedDistanceRight = lre_stepper_getMovedDistance(STEPPER_RIGHT);
		int16_t movedDistance = (abs(movedDistanceLeft)
				+ abs(movedDistanceRight)) / 2;

//		sprintf(str, "%d %d", movedDistanceLeft,movedDistanceRight);
//		send_usart_string(str);

		// check if mouse is to close to front wall
		if ( mouse_distance[0] < controller.front_distance)
		{
			lre_controller_stop();
			//TODO show that an error happened
		}

		//abort criteria because the desired distance is past
		else if (( movedDistance > controller.controller_desired_distance)
				&& (controller.controller_desired_distance != 0))
		{
			lre_controller_stop();
		}
		else
		{
			// ckeck if mouse sees a wall on the right or left or front
			if ( mouse_distance[2] <= (2 * controller.wall_distance))// ab 2 mal Wandabstand wird keine Wand erkannt.
			{
				rightWall = TRUE;
				lre_ledOn(ledRight);
			}
			else lre_ledOff(ledRight);

			if (mouse_distance[1] <= (2 * controller.wall_distance))// ab 2 mal Wandabstand wird keine Wand erkannt.
			{
				leftWall = TRUE;
				lre_ledOn(ledLeft);
			}
			else lre_ledOff(ledLeft);

			if (mouse_distance[0] <= (2 * controller.wall_distance))// ab 2 mal Wandabstand wird keine Wand erkannt.
			{
				lre_ledOn(ledUp);
			}
			else lre_ledOff(ledUp);

			// Control algorithm depending on which wall it sees

			// MODE 1: NO Wall
			if ((leftWall == FALSE) && (rightWall == FALSE)) {
				lre_stepper_setSpeed(controller.controller_speed, STEPPER_BOTH);
			}

			// MODE 2: only left Wall
			if ((leftWall == TRUE) && (rightWall == FALSE)) {
				lre_controller_leftWall();
				//send_usart_string("Linke Wand");
			}

			// MODE 3: only right Wall
			if ((leftWall == FALSE) && (rightWall == TRUE)) {
				lre_controller_rightWall();

				//send_usart_string("Rechte Wand");
			}

			// MODE 4: both Wall
			if ((leftWall == TRUE) && (rightWall == TRUE)) {
				lre_controller_bothWalls();
			}
		}
		// check if sensors are already looking into the next cell
//		if (( controller.controller_desired_distance - movedDistance ) < DISTANCE_VISIBLE)
//		{
//			nextCellVisible = VISIBLE;
//		}
		// reset interrupt pending bit
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	}
	lre_execution_time_toc();
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
