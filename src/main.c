/**
  ******************************************************************************
  * @file    main.c
  * @author  Nicolas Klenert
  * @version V1.0
  * @date    01-December-2018
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f0xx.h"
#include "stm32f072b_discovery.h"
#include "stdio.h"
#include "lre_led_status.h"
#include "lre_wait.h"
#include "lre_gyro.h"
#include "lre_sensor.h"

#include "lre_usart.h"
#include "lre_queue.h"
#include "mouse.h"
#include "main.h"

int main(void){
	//init usart
	usart_init();
	//init waiter
	lre_wait_init();
	//init gyro
//	lre_gyro_init();
	//init sensors
	lre_sensor_init();
	lre_sensor_start();

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
}


