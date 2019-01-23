/*
 * communication.c
 *
 *  Created on: 23.01.2019
 *      Author: JoBire
 */
#include "cmd.h"


/* argv[0] == "name"
 * argv[1] == "first argument"
 * ...
 *
 */


void move(int argc, char **argv)
{
 return;
}

/* Hier müssen alle cmd_add Befehle rein
 *
 * */
void communication_init()
{
	cmd_add("mv", &move);
}
