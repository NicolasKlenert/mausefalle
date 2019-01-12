/*
 * mm_systickTimer.h
 *
 *  Created on: 28.01.2017
 *      Author: fi
 */

#ifndef LRE_WAIT_H_
#define LRE_WAIT_H_

#include "stm32f0xx.h"

volatile uint32_t lre_wait_systickTime; // contains the system uptime since last initiation in ms

void lre_wait_init(void);
void lre_wait(uint32_t waitTime);

#endif /* LRE_WAIT_H_ */
