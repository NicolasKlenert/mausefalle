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

#define LRE_MOVE_DISTANCE_BETWEEN_WHEELS_MM	88	//distance between the middle of the wheels in mm
#define LRE_MOVE_DEFAULT_SPEED 60
// -------------------- (ControlLer Stuff) -------------------------
#define LRE_MOVE_CONTROLLER_FREQ 50	// Frequency of Controller in Hz
#define LRE_MOVE_TIMER_FREQ 10000 		// Frequency of TIM7 in Hz (minimum 734 Hz!!!)
#define LRE_MOVE_TIMER_PERIOD (uint16_t)(( LRE_MOVE_TIMER_FREQ / LRE_MOVE_CONTROLLER_FREQ ) - 1 )	// Period of TIM7

controller_struct controller = { 0, 0, 0, 0, 0, 0, 0 };

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
	lre_stepper_setSpeed(speed, STEPPER_RIGHT, distanceToTravel);
	lre_stepper_setSpeed(-speed, STEPPER_LEFT, -distanceToTravel);
	while (!lre_stepper_idle(STEPPER_BOTH)) {
		//wait till stepper is finished
	}
	moveMode = MOVE_IDLE;
}

void lre_move_distance(int16_t distance) {
	moveMode = MOVE_ACTIVE;
	if (distance > 0) {
		lre_stepper_setSpeed(LRE_MOVE_DEFAULT_SPEED, STEPPER_BOTH, distance);
	} else {
		lre_stepper_setSpeed(-LRE_MOVE_DEFAULT_SPEED, STEPPER_BOTH, distance);
	}
	while (!lre_stepper_idle(STEPPER_BOTH)) {
		//wait till stepper is finished
	}
	moveMode = MOVE_IDLE;
}

void lre_move_speed(int8_t speed) {
	moveMode = MOVE_ACTIVE;
	lre_stepper_setSpeed(speed, STEPPER_BOTH, 0);
}

void lre_move_stop() {
	lre_stepper_stop(STEPPER_BOTH);
	moveMode = MOVE_IDLE;
}

// -------------------- CONTROLLER STUFF ---------------------

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

	// TIM start
	TIM_Cmd(TIM7, ENABLE);

	// TIM7 enable update interrupt
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);

	// NVIC config and enable
	NVIC_InitTypeDef nvicTIM7;
	nvicTIM7.NVIC_IRQChannel = TIM7_IRQn;
	nvicTIM7.NVIC_IRQChannelCmd = DISABLE;
	nvicTIM7.NVIC_IRQChannelPriority = 1;	// can be 0 to 3
	NVIC_Init(&nvicTIM7);

}

// Timer fuer die Regelung anschalten
void lre_controller_start() {
	// Enable TIM7 IRQn
	NVIC_EnableIRQ(TIM7_IRQn);
	moveMode = MOVE_ACTIVE;
	control_flag = FALSE;
	controller.error = 0;
	controller.differential = 0;
	controller.integral = 0;
	controller.thetaAlt = 0;
	controller.dlAlt = 0;
	controller.drAlt = 0;
	controller.sensorAlt = controller.wall_distance;

}
// Timer fuer die Regelung abschalten
void lre_controller_stop() {
	// Disable TIM7 IRQn (interrupt handler will not be called anymore)
	NVIC_DisableIRQ(TIM7_IRQn);
	lre_move_stop();

	control_flag = TRUE;

}

/* move_straight setzt die Parameter des controller_structs auf von aussen gewuenschte Werte
 *
 */

void lre_move_straight(int16_t speed, int16_t desired_distance,
		int16_t wall_distance, int16_t front_distance) {
	//TODO: think about starting the controller here
	controller.controller_speed = speed;
	controller.controller_desired_distance = desired_distance;
	controller.wall_distance = wall_distance;
	controller.front_distance = front_distance;
	lre_controller_start();
}

// Regelungsroutine Timer handler
void TIM7_IRQHandler(void) {
	// check which interrupt occurred
	if (SET == TIM_GetITStatus(TIM7, TIM_IT_Update)) {
		int rightWall = FALSE;
		int leftWall = FALSE;

		if ((int16_t) mouse_distance[0] > controller.front_distance) // check if mouse is to close to wall
		{
			// ckeck if mouse sees a wall on the right or left
			if ((int16_t) mouse_distance[1] <= (2 * controller.wall_distance))// ab 2 mal Wandabstand wird keine Wand erkannt.
					{
				rightWall = TRUE;
			}

			if ((int16_t) mouse_distance[2] <= (2 * controller.wall_distance))// ab 2 mal Wandabstand wird keine Wand erkannt.
					{
				leftWall = TRUE;
			}

			// Control algorithm depending on witch wall it sees

			// MODE 1: NO Wall
			if ((leftWall == FALSE) && (rightWall == FALSE)) {
				lre_stepper_setSpeed(controller.controller_speed, STEPPER_BOTH,
						controller.controller_desired_distance);
			}

			// MODE 2: only left Wall
			if ((leftWall == TRUE) && (rightWall == FALSE)) {
				lre_controller_leftWall();
			}

			// MODE 3: only right Wall
			if ((leftWall == FALSE) && (rightWall == TRUE)) {
				lre_controller_rightWall();
			}

			// MODE 4: both Wall
			if ((leftWall == TRUE) && (rightWall == TRUE)) {
				lre_controller_bothWalls();
			}

		} else {
			control_flag = TRUE;
		}

		//abort criteria because the desired distance is past
		if ((abs(lre_stepper_getMovedDistance(STEPPER_RIGHT))
				> controller.controller_desired_distance)
				&& (controller.controller_desired_distance != 0)) {
			control_flag = TRUE; //abort criteria because the desired distance is past
		}

		if (control_flag == TRUE) {
			lre_controller_stop();
		}
		// reset interrupt pending bit
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	}
}
