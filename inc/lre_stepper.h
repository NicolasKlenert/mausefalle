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

// defines
#define TIMER_FREQ 1000000

// function prototypes
void lre_stepper_init(void);
void lre_stepper1_setStepFreq(uint16_t stepFreq);
void lre_stepper2_setStepFreq(uint16_t stepFreq);
void lre_stepper1_step(int32_t steps);
void lre_stepper2_step(int32_t steps);

#endif /* LRE_STEPPER_H_ */
