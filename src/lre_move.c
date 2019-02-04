/*
 * lre_move.c
 *
 *  Created on: 03.02.2019
 *      Author: Stefan
 */

#include "lre_move.h"

// -------------------- Abstracted Stuff (Move Stuff) -------------------------

#include "math.h"
#include "lre_stepper.h"

#define LRE_MOVE_DISTANCE_BETWEEN_WHEELS_MM	100	//distance between the middle of the wheels in mm
#define LRE_MOVE_DEFAULT_SPEED 60

//positive degree is a rotation to the left!
void lre_move_rotate(int8_t degree){
	moveMode = MOVE_ACTIVE;
	int8_t distanceToTravel = degree * LRE_MOVE_DISTANCE_BETWEEN_WHEELS_MM * M_PI / 180.0;
	int8_t speed = LRE_MOVE_DEFAULT_SPEED;
	if(distanceToTravel < 0){
		speed *= -1;
	}
	lre_stepper_setSpeed(speed,STEPPER_RIGHT,distanceToTravel);
	lre_stepper_setSpeed(-speed,STEPPER_LEFT,-distanceToTravel);
	while(!lre_stepper_idle(STEPPER_BOTH)){
		//wait till stepper is finished
	}
	moveMode = MOVE_IDLE;
}

void lre_move_distance(int16_t distance){
	moveMode = MOVE_ACTIVE;
	if(distance > 0){
		lre_stepper_setSpeed(LRE_MOVE_DEFAULT_SPEED, STEPPER_BOTH, distance);
	}else{
		lre_stepper_setSpeed(-LRE_MOVE_DEFAULT_SPEED, STEPPER_BOTH, distance);
	}
	while(!lre_stepper_idle(STEPPER_BOTH)){
			//wait till stepper is finished
	}
	moveMode = MOVE_IDLE;
}

void lre_move_speed(int8_t speed){
	moveMode = MOVE_ACTIVE;
	lre_stepper_setSpeed(speed, STEPPER_BOTH, 0);
}

void lre_move_stop(){
	lre_stepper_stop(STEPPER_BOTH);
	moveMode = MOVE_IDLE;
}


// -------------------- CONTROLLER STUFF ---------------------


#define LRE_MOVE_CONTROLLER_FREQ 100	// Frequency of Controller in Hz
#define LRE_MOVE_TIMER_FREQ 10000 		// Frequency of TIM7 in Hz (minimum 734 Hz!!!)
#define LRE_MOVE_TIMER_PERIOD (uint16_t)(( LRE_MOVE_TIMER_FREQ / LRE_MOVE_CONTROLLER_FREQ ) - 1 )	// Period of TIM7

// Reglerstruct
typedef struct{
	int16_t controller_speed;   	// mm/s
	int16_t controller_desired_distance;	// mm
	int16_t wall_distance;			// mm
}controller_struct;

// variables
controller_struct controller = {
		0,
		0,
		0};

// Timer fuer die Regelung initialisieren
void lre_controller_init()
{
	// RCC enable TIM6
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
	// RCC enable Sysconfig for interrupts
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);

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
	nvicTIM7.NVIC_IRQChannelPriority = 3;	// can be 0 to 3
	NVIC_Init(&nvicTIM7);

}

// Timer fuer die Regelung anschalten
void lre_controller_start()
{
	// Enable TIM7 IRQn
	NVIC_EnableIRQ(TIM7_IRQn);
}
// Timer fuer die Regelung abschalten
void lre_controller_stop()
{
	// Disable TIM7 IRQn (interrupt handler will not be called anymore)
	NVIC_DisableIRQ(TIM7_IRQn);
}

/* move_straight setzt die Parameter des controller_structs auf von aussen gewuenschte Werte
 *
 */

void lre_move_straight(int16_t speed, int16_t desired_distance, int16_t wall_distance)
{
	//TODO: think about starting the controller here
	controller.controller_speed = speed;
	controller.controller_desired_distance = desired_distance;
	controller.wall_distance = wall_distance;
}

// Regelungsroutine Timer handler
void TIM7_IRQHandler(void)
{
	// check which interrupt occured
	if(SET == TIM_GetITStatus(TIM7, TIM_IT_Update))
	{
		// reset interrupt pending bit
		TIM_ClearITPendingBit(TIM17, TIM_IT_Update);

		// TODO: set flag to start control loop

	}
}
