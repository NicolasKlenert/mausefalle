/*
 * lre_leds.h
 *
 *  Created on: Nov 1, 2015
 *      Author: M. Lehmann
 *
 *      LED abstraction driver for STM32F072 Discovery Board
 */

#include "stm32f0xx_gpio.h"

#define ledUp   GPIO_Pin_6
#define ledDown   GPIO_Pin_7
#define ledLeft   GPIO_Pin_8
#define ledRight   GPIO_Pin_9
#define ledAll  (ledUp | ledDown | ledRight | ledLeft)

#define ledPort      GPIOC

#define pushed  1
#define not_pushed      0

void systickInit(uint32_t tickFreq);
void SysTick_Handler(void);
void waitForSysTicks(uint32_t ticks);
void lre_leds_init(void);
void lre_ledToggle(uint16_t led);
void lre_ledOn(uint16_t led);
void lre_ledOff(uint16_t led);
uint8_t lre_userButton(void);
void lre_userButton_init(void);
