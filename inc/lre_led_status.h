/*
 * lre_led_status.h
 *
 *  Created on: 11.01.2019
 *      Author: Nicolas Klenert
 */

#ifndef LRE_LED_STATUS_H_
#define LRE_LED_STATUS_H_

/*
 * lre_led_status.c
 *
 *  Created on: 11.01.2019
 *      Author: Nicolas Klenert
 *
 *  This file should be used to set the status of the mouse.
 *  It uses the intern Timer 14.
 */

#include "lre_leds.h"
#include "lre_led_status.h"
#include "mouse.h"

uint16_t led_status_counter;

void led_status_init();
void led_status_show();

#endif /* LRE_LED_STATUS_H_ */
