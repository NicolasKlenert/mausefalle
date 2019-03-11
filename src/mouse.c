/*
 * mouse.c
 *
 *  Created on: 28.12.2018
 *      Author: Nicolas Klenert
 *
 */

#include "mouse.h"
#include "lre_wait.h"
#include "labyrinth.h"
#include "lre_led_status.h"
#include "lre_move.h"
#include "lre_stepper.h"
#include "lre_usart.h"
#include "stdio.h"
#include "lre_leds.h"
#include "lre_controler.h"

void mouse_init(){
	mouse_status = 0;
}

uint8_t mouse_findPath(uint16_t aim, uint16_t *arr, uint8_t length){
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

void mouse_setWall(uint16_t directionRelativeToMouse){
	uint16_t globalDirection = rotateDirection(mouse_direction,directionRelativeToMouse);
	setWall(mouse_position, globalDirection);
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

void mouse_setFrontWall(){
	mouse_setWall(0);
}

void mouse_setLeftWall(){
	mouse_setWall(3);
}

void mouse_setRightWall(){
	mouse_setWall(1);
}

void mouse_mapAll(uint16_t position, uint16_t direction){
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
	uint16_t direction_right = 0;	// global direction relative to mouse right
	uint16_t direction_left = 0;		// global direction relative to mouse left
	uint16_t direction_back = 0;		// global direction relative to mouse back
	int16_t rotation = 0;			// degrees to turn
	uint16_t new_position = 0;
	uint16_t new_direction = 0;

	// set initial position and direcrtion
	mouse_setPosition(position);
	mouse_setDirection(direction);

	// stay in this loop until arriving at the goal (middle of the labyrinth)
	while (mouse_position != goal)
	{

		/* -------------------- Set Gates ------------------- */
		if (mouse_distance[0] > 120)
		{
			mouse_setFrontGates(1);
		}
		else
		{
			mouse_setFrontWall();
		}

		if (mouse_distance[1] > 120)
		{
			mouse_setLeftGates(1);
		}
		else
		{
			mouse_setLeftWall();
		}

		if (mouse_distance[2] > 120)
		{
			mouse_setRightGates(1);
		}
		else
		{
			mouse_setRightWall();
		}

		// check walls to the front
//		mouse_setFrontGates( (uint16_t) (mouse_distance[0] / ROOM_WIDTH) );
//		// check walls to the left
//		mouse_setLeftGates( (uint16_t) (mouse_distance[1] / ROOM_WIDTH) );
//		// check walls to the right
//		mouse_setRightGates( (uint16_t) (mouse_distance[2] / ROOM_WIDTH) );

		/* -------------------- Mark visited ------------------- */
		setVisited(mouse_getPosition());

		/* -------------------- Decide which cell to visit next ------------------- */
		direction_right = rotateDirection(mouse_getDirection(), DIR_EAST);
		direction_left = rotateDirection(mouse_getDirection(), DIR_WEST);
		direction_back = rotateDirection(mouse_getDirection(), DIR_SOUTH);

		// TEST
		char str[50];
		sprintf(str,"Cell %d, Direction %d, Vorne: %d",mouse_getPosition(), mouse_getDirection(), mouse_distance[0]);
		send_usart_string(str);
		char arr[((numCols*widthRoom)+2)*(numRows*heightRoom)] = {0};
		printLabyrinth(arr);
		send_usart_string(arr);
//		lre_wait(1000);

		if (hasGate(mouse_getPosition(), direction_right))	// go right
		{
			rotation = -90;
			new_position = getCellId(mouse_getPosition(), direction_right);	// update mouse position
			new_direction = direction_right;								// update mouse direction
		}
		else if (hasGate(mouse_position, mouse_direction))	// go straight
		{
			rotation = 0;
			new_position = getCellId(mouse_getPosition(), mouse_getDirection());	// update mouse position
			new_direction = mouse_getDirection();									// same direction
		}
		else if (hasGate(mouse_position, direction_left))	// go left
		{
			rotation = 90;
			new_position = getCellId(mouse_getPosition(), direction_left);	// update mouse position
			new_direction = direction_left;								// update mouse direction
		}
		else											// go back
		{
			rotation = 180;
			new_position = getCellId(mouse_getPosition(), direction_back);		// update mouse position
			new_direction = direction_back;									// update mouse direction
		}

		/* -------------------- Make the move ------------------- */

		mouse_executeMove(rotation);

		/* ------------------- Check previous move -----------*/
		if (controller.controller_movedDistance < 130 && controller.controller_movedDistance > 10)
		{
			lre_wait(100);
			send_usart_string("Move wasn't complete! Going same distance back!");
			// mouse stopped due to an error or because of a front wall, move wasn't finished
			lre_move_distance(-controller.controller_movedDistance);
			while(!lre_move_idle())
			{
				// wait
			}
			//lre_move_straight(SPEED_MAPPING, ROOM_WIDTH - controller.controller_movedDistance, THRESHOLD_SITE, THRESHOLD_FRONT); // go backwards
		}
		else if (controller.controller_movedDistance <= 10)
		{
			send_usart_string("Move wasn't complete! Going 20mm back!");
			lre_move_distance(-20);
			while(!lre_move_idle())
			{
				// wait
			}
		}
		else	// move was successful, update position an direction
		{
			mouse_setPosition(new_position);
			mouse_setDirection(new_direction);
		}
	}
	send_usart_string("Yippiiieee!! I made it! Going back now...");
}

/* mouse will run along the path that is provided in arr
 * @param arr: pointer to array where the path is saved
 * @param length: length of array
 *
 * */
void mouse_Run(uint16_t* arr, uint8_t length, uint8_t position_in_path)
{
	uint8_t counter = position_in_path + 1;
	int16_t rotation = 0;
	uint16_t new_position = 0;
	uint16_t new_direction = 0;
	uint16_t direction_right = 0;	// global direction relative to mouse right
	uint16_t direction_left = 0;		// global direction relative to mouse left
	uint16_t direction_back = 0;
	uint16_t id_right = 0;
	uint16_t id_straight = 0;
	uint16_t id_left = 0;
	uint16_t id_back = 0;

	while (counter < length)
	{
		char str[50];
		sprintf(str,"Cell %d, Direction %d",mouse_getPosition(), mouse_getDirection());
		send_usart_string(str);

		direction_right = rotateDirection(mouse_getDirection(), DIR_EAST);
		direction_left = rotateDirection(mouse_getDirection(), DIR_WEST);
		direction_back = rotateDirection(mouse_getDirection(), DIR_SOUTH);
		id_right = getCellId(mouse_getPosition(), direction_right);
		id_left = getCellId(mouse_getPosition(), direction_left);
		id_straight = getCellId(mouse_getPosition(), mouse_getDirection());
		id_back = getCellId(mouse_getPosition(), direction_back);

		if ( id_right == arr[counter] )
		{
			rotation = -90;
			new_position = id_right;	// update mouse position
			new_direction = direction_right;
		}
		else if ( id_left == arr[counter] )
		{
			rotation = 90;
			new_position = id_left;	// update mouse position
			new_direction = direction_left;
		}
		else if ( id_straight == arr[counter] )
		{
			rotation = 0;
			new_position = id_straight;	// update mouse position
			new_direction = mouse_getDirection();
		}
		else if ( id_back == arr[counter] )
		{
			rotation = 180;
			new_position = id_back;	// update mouse position
			new_direction = direction_back;
		}
		else
		{
			send_usart_string("Help! I'm lost!!!");
			while (1)
			{
				//
			}
		}
		mouse_executeMove(rotation);

		/* ----------------- Check if move was complete --------------- */

		if (controller.controller_movedDistance < 100)
		{
			send_usart_string("Move wasn't complete, going back!");
			// mouse stopped due to an error or because of a front wall, move wasn't finished
			lre_move_distance(-controller.controller_movedDistance); // go backwards
		}
		else	// move was successful, update position an direction
		{
			mouse_setPosition(new_position);
			mouse_setDirection(new_direction);
			counter++;
		}
	}
}

void mouse_executeMove(int16_t rotation)
{
	// if there is no rotation, try to make the next move immediately (fluent driving on a straight path)
	if (rotation == 0)
	{
		lre_move_straight(SPEED_MAPPING, ROOM_WIDTH, THRESHOLD_SITE, THRESHOLD_FRONT);
	}
	else
	{
		lre_move_rotate(rotation);
		while (!lre_move_idle())		// wait for the rotation to finish
		{
			// wait
		}
		lre_move_straight(SPEED_MAPPING, ROOM_WIDTH, THRESHOLD_SITE, THRESHOLD_FRONT);
	}
	while (!lre_move_idle())
	{
		// wait till the move is completed
	}
	lre_ledToggle(ledDown);
}

void mouse_setStatus(uint16_t status){
	mouse_status = status;
	led_status_counter = 0;
}

void mouse_setPosition(uint16_t position)
{
	if (position < 49)
	{
		mouse_position = position;
	}
	else
	{
		send_usart_string("Tried to set invalid position!");
		while (1)
		{
			//
		}
	}
}
uint16_t mouse_getPosition(void)
{
	if (mouse_position < 49)
	{
		return mouse_position;
	}
	else
	{
		send_usart_string("Mouse position is invalid!");
		while (1)
		{
			//
		}
	}
}
void mouse_setDirection(uint16_t direction)
{
	if (mouse_direction < 4)
	{
		mouse_direction = direction;
	}
	else
	{
		send_usart_string("Tried to set invalid direction!");
		while (1)
		{
			//
		}
	}
}
uint16_t mouse_getDirection(void)
{
	if (mouse_direction < 4)
	{
		return mouse_direction;
	}
	else
	{
		send_usart_string("Mouse direction is invalid!");
		while (1)
		{
			//
		}
	}
}
