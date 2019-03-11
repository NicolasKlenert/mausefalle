/*
 * mouse.h
 *
 *  Created on: 28.12.2018
 *      Author: Nicol
 */

#ifndef MOUSE_H_
#define MOUSE_H_

#include "stm32f0xx.h"
#include "stm32f072b_discovery.h"

#define SPEED_MAPPING 50
#define SPEED_RUNNING 70

uint16_t mouse_status;
uint16_t mouse_distance[3];	// vorne, links, rechts
//float mouse_sensor_time[3];

static volatile int16_t mouse_position;
static volatile int16_t mouse_direction;
volatile int16_t mouse_start_position;
volatile int16_t mouse_start_direction;
volatile int16_t mouse_aim;

void mouse_init();
uint8_t mouse_findPath(uint16_t aim, uint16_t *arr, uint8_t length);
void mouse_setGates(uint16_t directionRelativeToMouse, uint16_t length);
void mouse_setFrontGates(uint16_t length);
void mouse_setLeftGates(uint16_t length);
void mouse_setRightGates(uint16_t length);
void mouse_setStatus(uint16_t status);
uint8_t identifyGates(uint16_t distance);
void mouse_mapAll(uint16_t position, uint16_t direction);
void mouse_executeMove(int16_t rotation);
void mouse_setPosition(uint16_t position);
uint16_t mouse_getPosition(void);
void mouse_setDirection(uint16_t position);
uint16_t mouse_getDirection(void);
void mouse_Run(uint16_t* arr, uint8_t length, uint8_t position_in_path);

#define MOUSE_STANDBY 			0
#define MOUSE_CRITICAL_ERROR 	1

#endif /* MOUSE_H_ */
