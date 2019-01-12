/*
 * labyrinth.h
 *
 *  Created on: 24.12.2018
 *      Author: Nicolas Klenert
 */

#ifndef LABYRINTH_H_
#define LABYRINTH_H_

#include "stm32f0xx.h"
#include "stm32f072b_discovery.h"

#define numRows			7
#define numCols			7
#define goal			24
#define TRUE			1
#define FALSE			0
#define DIR_NORTH		0
#define DIR_EAST		1
#define DIR_SOUTH		2
#define DIR_WEST		3

/**
 * cells are 4bit-bitflags.
 * The first bit indicates if the upper path is open (1) or not (0).
 * The second one is for the right side, after that the bottom and left side.
 * Example: 0b1001 stands for a cell which has walls on the east and south.
 *
 * The array is the listing of the cells. We start to count at the upper left corner and count towards the left
 * example: a 3x3 maze has an array with ids:
 * 0,1,2
 * 3,4,5
 * 6,7,8
 *
 * The defualt labyrinth is a labyrinth with walls everywhere
 */
volatile uint16_t cells[numRows*numCols];

uint16_t getPath(uint16_t start, uint16_t aim, uint16_t *arr);
void setWall(uint16_t id,uint16_t direction);
void setGate(uint16_t id,uint16_t direction);
void setWalls(uint16_t id,uint16_t bitflag);
void setGates(uint16_t id,uint16_t bitflag);
uint16_t hasWall(uint16_t id,uint16_t direction);
uint16_t hasGate(uint16_t id,uint16_t direction);
uint16_t getCellId(uint16_t id,uint16_t direction);
uint16_t inverseDirection(uint16_t direction);
uint16_t rotateDirection(uint16_t direction, uint16_t times);


#endif /* LABYRINTH_H_ */
