/*
 * mouse.c
 *
 *  Created on: 28.12.2018
 *      Author: Nicolas Klenert
 *
 */

#include "mouse.h"
#include "labyrinth.h"
#include "lre_led_status.h"

void mouse_init(){
	mouse_status = 0;
}

uint16_t mouse_findPath(uint16_t aim, uint16_t *arr){
	return getPath(mouse_position,aim,arr);
}

void mouse_setGates(uint16_t directionRelativeToMouse, uint16_t length){
	uint16_t tempPos = mouse_position;
	uint16_t globalDirection = rotateDirection(mouse_direction,directionRelativeToMouse);
	for (int i = 0; i < length; i++){
		setGate(tempPos,globalDirection);
		tempPos = getCellId(tempPos,globalDirection);
		setGate(tempPos,inverseDirection(globalDirection));
	}
}

void mouse_setFrontGates(uint16_t length){
	mouse_setGates(0,length);
}

void mouse_setLeftGates(uint16_t length){
	mouse_setGates(3,length);
}

void mouse_setRightGates(uint16_t length){
	mouse_setGates(1,length);
}

void mapAll(){
	//function to map the labyrinth.
	//It searches till all cells are visited. The cells most adjacent to the mouse are chosen first.

}

void mouse_setStatus(uint16_t status){
	mouse_status = status;
	led_status_counter = 0;
}
