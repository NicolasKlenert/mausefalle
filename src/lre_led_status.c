/*
 * lre_led_status.c
 *
 *  Created on: 11.01.2019
 *      Author: Nicolas Klenert
 *
 *  This file makes the status of the mouse visible.
 *  It uses the intern Timer 3, so we are able to dim the lights with MPW.
 */

#include "lre_leds.h"
#include "lre_led_status.h"
#include "mouse.h"

void led_status_init(){
	lre_leds_init();
	mouse_init();
	//set timer so we can use interrupts to change the leds accordingly
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIM_TimeBaseInitTypeDef timerInitStruct;
	timerInitStruct.TIM_ClockDivision = 0;
	timerInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStruct.TIM_Period = 250 - 1;
	timerInitStruct.TIM_Prescaler =	SystemCoreClock / 1000 - 1;	//1000 Hz
	timerInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &timerInitStruct);
	TIM_ITConfig(TIM3,TIM_IT_Update, ENABLE);
	NVIC_EnableIRQ(TIM3_IRQn);
}

void led_status_show(){
	TIM_Cmd(TIM3,ENABLE);
	led_status_counter = 0;
}

void TIM3_IRQHandler(void){
	//uint16_t led_status_counter;
	switch(mouse_status){
	case MOUSE_CRITICAL_ERROR:
		lre_ledToggle(ledAll);
		break;
	case MOUSE_STANDBY:
	default:
		if(led_status_counter %2 == 0){
			lre_ledOn(ledUp | ledDown);
			lre_ledOff(ledLeft | ledRight);
		}else{
			lre_ledOn(ledLeft | ledRight);
			lre_ledOff(ledUp | ledDown);
		}
	}
//	lre_ledToggle(ledUp);
	led_status_counter++;
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
}
