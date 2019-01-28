/*
 * communication.c
 *
 *  Created on: 23.01.2019
 *      Author: JoBire
 */

#include <lre_communication.h>
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
}


/* Hier sollen die Funktionen aus der communication
 *
 */

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
		lre_ledToggle(ledRight);
	}
	// Telemetrie Rotation
	if ((argv[1][0]== 0x68) && (argv[1][1]== 0x64)) // first letter h, second letter d in hex
	{
		lre_ledToggle(ledRight);
	}

}


void lre_move(int argc, char **argv)
{
	lre_ledOn(ledRight);
	// move speed
	if ((argv[1][0]== 0x73) && (argv[1][1]== 0x70)) // first letter s, second letter p in hex
	{
		uint32_t speed = cmd_str2Num(argv[2], (uint8_t)10);
		lre_stepper_setSpeed((uint8_t)speed, STEPPER_BOTH, 0);
	}
	// move stop
	if ((argv[1][0]== 0x73) && (argv[1][1]== 0x74)) // first letter s, second letter t in hex
	{
		lre_stepper_stop(STEPPER_BOTH);
	}

	// move distance
	if ((argv[1][0]== 0x64) && (argv[1][1]== 0x73)) // first letter d, second letter s in hex
	{
		int32_t distance_desired = cmd_str2Num(argv[2], (uint8_t)10);
		lre_stepper_setSpeed(SPEED, STEPPER_BOTH, distance_desired);
	}
	// move rotate
		if ((argv[1][0]== 0x73) && (argv[1][1]== 0x74)) // first letter s, second letter t in hex
		{
				lre_stepper_stop(STEPPER_BOTH);
		}
}

void lre_maze_com(int argc, char **argv)
{	//  maze
	if ((argv[1][0]== 0x75) && (argv[1][1]== 0x73)) // first letter u, second letter s in hex
	{
		lre_ledToggle(ledUp);
	}
	// maze
	if ((argv[1][0]== 0x70) && (argv[1][1]== 0x73)) // first letter u, second letter s in hex
	{
		lre_ledToggle(ledUp);
	}
	// maze
	if ((argv[1][0]== 0x68) && (argv[1][1]== 0x64)) // first letter u, second letter s in hex
	{
		lre_ledToggle(ledUp);
	}

}

