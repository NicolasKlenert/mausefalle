/*
 * stepper.h
 *
 *  Created on: 14.01.2019
 *      Author: JoBire
 */

//TODO invert left stepper direction

#ifndef LRE_STEPPER_H_
#define LRE_STEPPER_H_

// includes
#include "stm32f0xx.h"
#include <stdlib.h>

// defines
#define STEPPER_RIGHT	0b01		// belongs to TIM16
#define STEPPER_LEFT	0b10		// belongs to TIM17
#define STEPPER_BOTH	0b11

#define ACCELERATION (float)50.0			// acceleration in mm/s^2

#define STEPPER_MAX_SPEED 90		// max speed in mm/s

// function prototypes
void lre_stepper_init(void);
uint8_t lre_stepper_idle(uint8_t stepper_x);
void lre_stepper_setSpeed(int8_t speed_mm_s, uint8_t stepper_x);
void lre_stepper_setMaxDistance(int16_t max_distance, uint8_t stepper_x);
int16_t lre_stepper_getMovedDistance(uint8_t stepper_x);
void lre_stepper_stop(uint8_t stepper_x);

#endif /* LRE_STEPPER_H_ */
