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
#include "lre_move.h"

void mouse_init(){
	mouse_status = 0;
}

uint16_t mouse_findPath(uint16_t aim, uint16_t *arr, uint8_t length){
	return getPath(mouse_position,aim,arr,length);
}
/*
 * @param length: how many gates to set in this direction
 * */
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

void mouse_mapAll(uint16_t start_direction, uint16_t start_position){
	//function to map the labyrinth.
	//It searches till all cells are visited. The cells most adjacent to the mouse are chosen first.

	/* Algorithm:
	 *
	 * 1. Check where the walls are (sensors) and set Gates
	 * 2. Mark the current cell as visited
	 * 3. Decide which cell to visit next
	 * 4. Make the move a) straight or b) rotate
	 * 5. Change a) position or b) direction
	 *
	 * Repeat these steps until arriving at goal, then get the shortest way back to start
	 *
	 * */

	// variables
	uint8_t direction_right = 0;	// global direction relative to mouse right
	uint8_t direction_straight = 0;	// global direction relative to mouse straight
	uint8_t direction_left = 0;		// global direction relative to mouse left
	uint16_t nextCell = 0;			// id of next Cell to visit
	int16_t rotation = 0;			// degrees to turn

	// first set the starting direction and position
	mouse_position = start_position;
	mouse_direction = start_direction;

	// stay in this loop until arriving at the goal (middle of the labyrinth)
	while (mouse_position != goal)
	{
		/* -------------------- Set Gates ------------------- */
		// check walls to the front
		mouse_setFrontGates( (uint16_t) (mouse_distance[0] / ROOM_WIDTH) );
		// check walls to the left
		mouse_setLeftGates( (uint16_t) (mouse_distance[1] / ROOM_WIDTH) );
		// check walls to the right
		mouse_setRightGates( (uint16_t) (mouse_distance[2] / ROOM_WIDTH) );

		/* -------------------- Mark visited ------------------- */
		setVisited(mouse_position);

		/* -------------------- Decide which cell to visit next ------------------- */
		direction_right = rotateDirection(mouse_direction, DIR_EAST);
		direction_straight = rotateDirection(mouse_direction, DIR_NORTH);
		direction_left = rotateDirection(mouse_direction, DIR_WEST);

		if (hasGate(mouse_position, direction_right))
		{
			rotation = -90;
			mouse_position = getCellId(mouse_position, direction_right);	// go right
			mouse_direction = direction_right;
		}
		else if (hasGate(mouse_position, direction_straight))
		{
			rotation = 0;
			mouse_position = getCellId(mouse_position, direction_straight);	// go straight
			mouse_direction = direction_straight;
		}
		else if (hasGate(mouse_position, direction_left))
		{
			rotation = 90;
			mouse_position = getCellId(mouse_position, direction_left);	// go left
			mouse_direction = direction_left;
		}
		else
		{
			rotation = 180;
			mouse_position = getCellId(mouse_position, inverseDirection(mouse_direction));	// go back
			mouse_direction = inverseDirection(mouse_direction);
		}

		/* -------------------- Make the move ------------------- */

	}
}

void mouse_executeMove(int16_t rotation)
{
	if (rotation == 0)
	{
		lre_move_straight(SPEED_MAPPING, ROOM_WIDTH, THRESHOLD_SITE, THRESHOLD_FRONT);
	}
	else
	{
		lre_move_rotate(rotation);
		while(!lre_move_idle())
		{
			// wait
		}
		lre_move_straight(SPEED_MAPPING, ROOM_WIDTH, THRESHOLD_SITE, THRESHOLD_FRONT);
	}
}

void mouse_setStatus(uint16_t status){
	mouse_status = status;
	led_status_counter = 0;
}
