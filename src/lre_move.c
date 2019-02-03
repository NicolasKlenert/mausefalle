/*
 * lre_move.c
 *
 *  Created on: 03.02.2019
 *      Author: Stefan
 */



// Reglerstruct
typedef struct{
	int16_t controller_speed;   	// mm/s
	int16_t controller_desired_distance;	// mm
	int16_t wall_distance;			// mm
}controller_struct;

// variables
controller_struct controller = {
		0,
		0,
		0};

// Timer fuer die Regelung initialisieren
void lre_controller_init()
{

}

// Timer fuer die Regelung anschalten
void lre_controller_start()
{

}
// Timer fuer die Regelung abschalten
void lre_controller_stop()
{

}

/* move_straight setzt die Parameter des controller_structs auf von aussen gewuenschte Werte
 *
 */

void move_straight(int16_t speed, int16_t desired_distance, int16_t wall_distance)
{
	controller.controller_speed = speed;
	controller.controller_desired_distance = desired_distance;
	controller.wall_distance = wall_distance;
}

// Regelungsroutine Timer handler
