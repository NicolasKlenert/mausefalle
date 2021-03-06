/*
 * communication.c
 *
 *  Created on: 23.01.2019
 *      Author: JoBire
 */

#include <lre_communication.h>
// includes
#include "cmd.h"
#include "lre_stepper.h"
#include "lre_leds.h"
#include "lre_queue.h"
#include "lre_usart.h"
#include "lre_move.h"
#include "mouse.h"
#include "main.h"
#include "lre_controler.h"
#include "labyrinth.h"
/* argv[0] == "name"
 * argv[1] == "first argument"
 * ...
 *
 */


/* Hier m�ssen alle cmd_add Befehle rein
 *
 * */
void lre_communication_init()
{
	cmd_init();
	cmd_add("mv", &lre_move);
	cmd_add("tm", &lre_telemetrie);
	cmd_add("mz", &lre_maze_com);
	cmd_add("ma", &lre_maneuver);

}


/* Hier sollen die Funktionen aus der communication
 *
 */

//
void lre_maneuver(int argc, char **argv)
{
	// park: "ma pk front_distance reverse_distance"
	if ( (argv[1][0] == 'p') && (argv[1][1] == 'k') )
	{
		int16_t distance1 = cmd_str2Num(argv[2], (uint8_t)10);
		int16_t distance2 = cmd_str2Num(argv[3], (uint8_t)10);
		lre_move_straight(50, 0, 1, distance1);
		while(moveMode == MOVE_ACTIVE)
		{
			//
		}
		lre_move_rotate(180);
		while(moveMode == MOVE_ACTIVE)
		{
			//
		}
		lre_move_distance(-distance2);
		//lre_move_straight(-30, 0, 1, distance2);


	}
	// wall: "ma wl walldistance"
	if ( (argv[1][0] == 'w') && (argv[1][1] == 'l') )
	{
		//int16_t distance1 = cmd_str2Num(argv[2], (uint8_t)10);

		//lre_move_straight(50, 2000, distance1, 45);
		lre_move_distance(2000);
	}
	// corner: "ma co"
	if ( (argv[1][0]=='c') && (argv[1][1]=='o') )
	{
		// check distance
		int16_t mindis = (int16_t) mouse_distance[2] > mouse_distance[1] ? mouse_distance[1] : mouse_distance[2];
		lre_move_straight(50, 0, 1, mindis);
		while(moveMode == MOVE_ACTIVE)
		{
			//
		}
		lre_move_rotate(-90);
		while(moveMode == MOVE_ACTIVE)
		{
			//
		}
		lre_move_distance(700);
	}

	// map all: "ma map position direction"
	if ( (argv[1][0] == 'm' ) && (argv[1][1] == 'a') && (argv[1][2] == 'p') )
	{
		mouse_start_position = cmd_str2Num(argv[2], (uint8_t)10);
		mouse_start_direction = cmd_str2Num(argv[3], (uint8_t)10);
		flag_mapAll = TRUE;
	}

	// run: "ma run position direction aim"
	if ( (argv[1][0] == 'r' ) && (argv[1][1] == 'u') && (argv[1][2] == 'n') )
	{
		mouse_start_position = cmd_str2Num(argv[2], (uint8_t)10);
		mouse_start_direction = cmd_str2Num(argv[3], (uint8_t)10);
		mouse_aim = cmd_str2Num(argv[4], (uint8_t)10);
		flag_run = TRUE;
	}
}

void lre_telemetrie(int argc, char **argv)
{	// Telemetrie Ultra-sonic sensors
	if ((argv[1][0]== 0x75) && (argv[1][1]== 0x73)) // first letter u, second letter s in hex
	{
		char str[75];
		char distance1[24];
		char distance2[24];
		char distance3[24];
		hreadable_floats(mouse_distance[0],distance1);
		hreadable_floats(mouse_distance[1],distance2);
		hreadable_floats(mouse_distance[2],distance3);
		sprintf(str,"Entfernungen: %s; %s; %s;",distance1,distance2,distance3);
		send_usart_string(str);
	}
	// Telemetrie Position
	if ((argv[1][0]== 0x70) && (argv[1][1]== 0x73)) // first letter p, second letter s in hex
	{
	}
	// Telemetrie Rotation
	if ((argv[1][0]== 0x68) && (argv[1][1]== 0x64)) // first letter h, second letter d in hex
	{
	}

}


void lre_move(int argc, char **argv)
{


	// move line
	if ((argv[1][0]== 0x6C) && (argv[1][1]== 0x6E)) // first letter l, second letter n in hex
	{
		int16_t distance_line = cmd_str2Num(argv[2], (uint8_t)10);
		lre_move_straight(60, distance_line, THRESHOLD_SITE, THRESHOLD_FRONT);

	}
	// move speed
	if ((argv[1][0]== 0x73) && (argv[1][1]== 0x70)) // first letter s, second letter p in hex
	{
		int8_t speed = cmd_str2Num(argv[2], (uint8_t)10);
		lre_move_speed(speed);
	}
	// move stop
	if ((argv[1][0]== 0x73) && (argv[1][1]== 0x74)) // first letter s, second letter t in hex
	{
		lre_controller_stop();
	}

	// move distance
	if ((argv[1][0]== 0x64) && (argv[1][1]== 0x73)) // first letter d, second letter s in hex
	{
		int32_t distance_desired = cmd_str2Num(argv[2], (uint8_t)10);
		lre_move_distance(distance_desired);
	}
	// move rotate
	if ((argv[1][0]== 0x72) && (argv[1][1]== 0x74)) // first letter r, second letter t in hex
	{
		int32_t degree = cmd_str2Num(argv[2], (uint8_t)10);
		lre_move_rotate(degree);
	}
}

void lre_maze_com(int argc, char **argv)
{	//  maze
	if ((argv[1][0]== 0x75) && (argv[1][1]== 0x73)) // first letter u, second letter s in hex
	{
	}
	// maze
	if ((argv[1][0]== 0x70) && (argv[1][1]== 0x73)) // first letter u, second letter s in hex
	{

	}
	// maze
	if ((argv[1][0]== 0x68) && (argv[1][1]== 0x64)) // first letter u, second letter s in hex
	{

	}
}

