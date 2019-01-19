/*
 * stepper.h
 *
 *  Created on: 14.01.2019
 *      Author: JoBire
 */

#ifndef LRE_STEPPER_H_
#define LRE_STEPPER_H_

// includes
#include "stm32f0xx.h"
#include <stdlib.h>

// defines
#define STEPPER_RIGHT (uint8_t)0		// belongs to TIM16
#define STEPPER_LEFT (uint8_t)1			// belongs to TIM17

// function prototypes
void lre_stepper_init(void);
void lre_stepper_setSpeed(int8_t speed_mm_s, uint8_t stepper_x);
int16_t lre_stepper_getMovedDistance(uint8_t stepper_x);

#endif /* LRE_STEPPER_H_ */
