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
}


/* Hier sollen die Funktionen aus der communication
 *
 */
void lre_move(int argc, char **argv)
{	// move speed
	if ((argv[1][0]== 0x73) && (argv[1][1]== 0x70)) // first letter s, second letter p in hex
		{
		uint32_t speed = cmd_str2Num(argv[2], (uint8_t)10);
		lre_stepper_setSpeed((uint8_t)speed, STEPPER_LEFT);
		lre_stepper_setSpeed((uint8_t)speed, STEPPER_RIGHT);
	}
	// move stop
		if ((argv[1][0]== 0x73) && (argv[1][1]== 0x74)) // first letter s, second letter t in hex
			{
			lre_stepper_stop();
		}

	// move distance --> quick and dirty
		if ((argv[1][0]== 0x64) && (argv[1][1]== 0x73)) // first letter d, second letter s in hex
			{
			//uint32_t distance_desired = cmd_str2Num(argv[2], (uint8_t)10);
			uint32_t distance_desired = 1000;
			int16_t moved_distance=0;
			while(moved_distance  < distance_desired)
			{

				lre_stepper_setSpeed((uint8_t)SPEED, STEPPER_LEFT);
				lre_stepper_setSpeed((uint8_t)SPEED, STEPPER_RIGHT);
				lre_wait(1000);
				moved_distance = lre_stepper_getMovedDistance(STEPPER_RIGHT);
			}


			lre_stepper_stop();
		}


}



