/**
  ******************************************************************************
  * @file    main.c
  * @author  Nicolas Klenert
  * @version V1.0
  * @date    01-December-2018
  * @brief   Default main function.
  ******************************************************************************
*/


#include <lre_communication.h>
#include "stm32f0xx.h"
#include "stm32f072b_discovery.h"
#include "stdio.h"
#include "lre_led_status.h"
#include "lre_wait.h"
#include "lre_sensor.h"
#include "lre_stepper.h"
#include "lre_usart.h"
#include "lre_queue.h"
#include "lre_move.h"
#include "lre_controler.h"
#include "mouse.h"
#include "main.h"
#include "labyrinth.h"
#include "test.h"
#include "lre_leds.h"

void error(char* string){
	mouse_setStatus(MOUSE_CRITICAL_ERROR);
	led_status_show();
	send_usart_string(string);
}

int main(void){
	// init lre_leds
	lre_leds_init();
	//init waiter
	lre_wait_init();
	// init controller
	lre_controller_init();
	//init usart
	lre_usart_init();
	// init stepper
	lre_stepper_init();
	// init communication
	lre_communication_init();
	//init sensors
	lre_sensor_init();

	labyrinth_init();

	// start stuff
	lre_sensor_start();

//	testPathFinding();
//	uint16_t test = testQueue();
//	char arr[((numCols*widthRoom)+2)*(numRows*heightRoom)] = {0};
//	printLabyrinth(arr);
//	lre_wait(5000);
//	send_usart_string(arr);

		while(1){
			lre_wait(1000);
			char str[75] = "connection online";
			send_usart_string(str);
			char distance[80];
			sprintf(distance, "Vorne: %d; Links: %d; Rechts: %d", mouse_distance[0], mouse_distance[1], mouse_distance[2]);
			send_usart_string(distance);
		}
}

/*   Testprogramm vom 24.01.19 17:19 Uhr
 *
 	//init usart
	usart_init();
	//init waiter
	lre_wait_init();
	//init gyro
//	lre_gyro_init();
	//init sensors
	lre_sensor_init();
	lre_sensor_start();
	cmd_init();
	cmd_add("stepper_set_speed", &stepperSetSpeed);
	cmd_add("stepper_stop", &stepperStop);
	lre_stepper_init();
//	lre_stepper_setSpeed(20, STEPPER_LEFT);

	while(1){
		lre_wait(400);
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

//	led_status_init();
//	mouse_setStatus(MOUSE_CRITICAL_ERROR);
//	led_status_show();

 */


