/*
 * lre_leds.c
 *
 *  Created on: Nov 1, 2015
 *      Author: M. Lehmann
 *
 *      LED abstraction driver for STM32F072 Discovery Board
 */

#include "lre_leds.h"

/**
 * @brief initiates the 4 Pins which are connected to the LEDs
 * @note designed for the STM32F072Discovery Board
 */
void lre_leds_init(void) {
	// activate peripheral clock module for GPIO_C
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

	// create init structure
	GPIO_InitTypeDef gpio_initStruct;

	/* write values to the init structure */
	// use pins as output
	gpio_initStruct.GPIO_Mode = GPIO_Mode_OUT;
	// pins will directly connect either to GND (0) or to VCC (1) (Push Pull configuration)
	gpio_initStruct.GPIO_OType = GPIO_OType_PP;
	// use pins 6 to 9
	gpio_initStruct.GPIO_Pin = (ledUp | ledDown | ledRight | ledLeft);
	// deactivate internal Pullup or Pulldown resistor
	gpio_initStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	// set pins to maximum speed
	gpio_initStruct.GPIO_Speed = GPIO_Speed_50MHz;

	// initiate the GPIO with the init structure
	GPIO_Init(ledPort, &gpio_initStruct);
}

/**
 * @brief toggles the values of "led"
 * @param led: can be either ledUp, ledDown, ledLeft, ledRight
 */
void lre_ledToggle(uint16_t led) {
	uint16_t portValue;
	portValue = GPIO_ReadOutputData(ledPort);
	portValue = portValue ^ led;
	GPIO_Write(ledPort, portValue);
}

/**
 * @brief switches "led" on
 * @param led: can be either ledUp, ledDown, ledLeft, ledRight
 */
void lre_ledOn(uint16_t led) {
	GPIO_SetBits(ledPort, led);
}

/**
 * @brief switches "led" off
 * @param led: can be either ledUp, ledDown, ledLeft, ledRight
 */
void lre_ledOff(uint16_t led) {
	GPIO_ResetBits(ledPort, led);
}

uint8_t lre_userButton(void) {
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)) {
		return pushed;
	} else {
		return not_pushed;
	}
}

void lre_userButton_init(void) {
	// activate peripheral clock module for GPIO_A
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	// create init structure
	GPIO_InitTypeDef gpio_initStruct;

	/* write values to the init structure */
	// use pins as output
	gpio_initStruct.GPIO_Mode = GPIO_Mode_IN;
	// use pin 0
	gpio_initStruct.GPIO_Pin = GPIO_Pin_0;
	// deactivate internal Pullup or Pulldown resistor
	gpio_initStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	// set pins to maximum speed
	gpio_initStruct.GPIO_Speed = GPIO_Speed_50MHz;

	// initiate the GPIO with the init structure
	GPIO_Init(GPIOA, &gpio_initStruct);
}
