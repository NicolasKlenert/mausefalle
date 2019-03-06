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
	uint8_t direction_left = 0;		// global direction relative to mouse left
	uint8_t direction_back = 0;		// global direction relative to mouse back
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
		direction_left = rotateDirection(mouse_direction, DIR_WEST);
		direction_back = rotateDirection(mouse_direction, DIR_SOUTH);

		if (hasGate(mouse_position, direction_right))	// go right
		{
			rotation = -90;
			mouse_position = getCellId(mouse_position, direction_right);	// update mouse position
			mouse_direction = direction_right;								// update mouse direction
		}
		else if (hasGate(mouse_position, mouse_direction))	// go straight
		{
			rotation = 0;
			mouse_position = getCellId(mouse_position, mouse_direction);	// update mouse position
		}
		else if (hasGate(mouse_position, direction_left))	// go left
		{
			rotation = 90;
			mouse_position = getCellId(mouse_position, direction_left);		// update mouse position
			mouse_direction = direction_left;								// update mouse direction
		}
		else											// go back
		{
			rotation = 180;
			mouse_position = getCellId(mouse_position, direction_back);		// update mouse position
			mouse_direction = direction_back;								// update mouse direction
		}

		/* -------------------- Make the move ------------------- */
		mouse_executeMove(rotation);
	}
}

/* mouse will run along the path that is provided in arr
 * @param arr: pointer to array where the path is saved
 * @param length: length of array
 *
 * */
void mouse_Run(uint16_t* arr, uint8_t length)
{

}

void mouse_executeMove(int16_t rotation)
{
	// if there is no rotation, try to make the next move immediately (fluent driving on a straight path)
	if (rotation == 0)
	{
		if (!lre_move_idle())
		{
			// just alter distance if mouse is still moving
			lre_move_straight_alter_distance(ROOM_WIDTH);
		}
		else
		{
			// new move if last move is already complete
			lre_move_straight(SPEED_MAPPING, ROOM_WIDTH, THRESHOLD_SITE, THRESHOLD_FRONT);
		}
	}
	else
	{
		while (!lre_move_idle())	// Wait for the previous move to finish
		{
			// wait
		}
		lre_move_rotate(rotation);
		while (!lre_move_idle())		// wait for the rotation to finish
		{
			// wait
		}
		lre_move_straight(SPEED_MAPPING, ROOM_WIDTH, THRESHOLD_SITE, THRESHOLD_FRONT);
//		lre_move_distance(ROOM_WIDTH);
	}
	while (!lre_move_idle() && !lre_move_nextCellVisible())
	{
		// wait till next cell is visible or the move is completed
	}
}

void mouse_setStatus(uint16_t status){
	mouse_status = status;
	led_status_counter = 0;
}
