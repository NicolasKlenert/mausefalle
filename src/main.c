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
	//init leds
	lre_leds_init();

	while(1){
		lre_wait(400);
		lre_ledToggle(ledAll);
		send_usart_string("test");
	}

//	led_status_init();
//	mouse_setStatus(MOUSE_CRITICAL_ERROR);
//	led_status_show();
}


