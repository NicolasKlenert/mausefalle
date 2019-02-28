/*
 * test.h
 *
 *  Created on: 24.12.2018
 *      Author: Nicol
 */

#ifndef TEST_H_
#define TEST_H_

#include "stm32f0xx.h"
#include "stm32f072b_discovery.h"

void createFakeLabyrinth();
//tests if labyrinth has a border around it (and if you can go to the middle with all 4 corners)
uint16_t acceptableLabyrinth();
uint16_t testPathFinding();
uint16_t testQueue();


#endif /* TEST_H_ */
