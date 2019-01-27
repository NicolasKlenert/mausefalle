/*
 * lre_execution_time.h
 *
 *  Created on: 27.01.2019
 *      Author: JoBire
 */

#ifndef LRE_EXECUTION_TIME_H_
#define LRE_EXECUTION_TIME_H_


// includes
#include "stm32f0xx.h"

// variables
uint32_t execution_time;

// function prototypes
void lre_execution_time_init();
void lre_execution_time_tic();
void lre_execution_time_toc();

#endif /* LRE_EXECUTION_TIME_H_ */
