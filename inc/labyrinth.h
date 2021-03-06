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
#define ROOM_WIDTH      200

#define leftWallChar '|'
#define upperWallChar '_'
#define bottomWallChar '_'
#define rightWallChar '|'
#define upperLeftCornerChar ' '
#define upperRightCornerChar ' '
#define bottomLeftCornerChar '|'
#define bottomRightCornerChar '|'
#define heightRoom 3
#define widthRoom 5

//arr has to be ((numCols*widthRoom)+2)*(numRows*heightRoom) big
void printLabyrinth(char *arr);
void labyrinth_init();
uint8_t getPath(uint16_t start, uint16_t aim, uint16_t *arr, uint8_t length);
void setWall(uint16_t id,uint8_t direction);
void setGate(uint16_t id,uint8_t direction);
void setWalls(uint16_t id,uint8_t bitflag);
void setGates(uint16_t id,uint8_t bitflag);
uint8_t hasWall(uint16_t id,uint8_t direction);
uint8_t hasGate(uint16_t id,uint8_t direction);
void setVisited(uint16_t id);
uint8_t isVisited(uint16_t id);
int16_t getCellId(int16_t id,uint8_t direction);
uint8_t inverseDirection(uint8_t direction);
uint8_t rotateDirection(uint8_t direction, uint8_t times);
void createFakeLabyrinth();


#endif /* LABYRINTH_H_ */
