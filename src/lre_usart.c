/*
 * lre_usart.c
 *
 *  Created on: 24.12.2018
 *      Author: Nicolas Klenert
 */

#include "stm32f0xx.h"
#include "stm32f072b_discovery.h"
#include "stdio.h"
#include "lre_usart.h"

void hreadable_floats(float number, char *buf){
	char tmpSign = (number < 0) ? '-' : '+';
	float tmpVal = (number < 0) ? -number: number;
	int tmpInt = tmpVal;
	float tmpFrac = tmpVal - tmpInt;
	int tmpLast = tmpFrac * 10000; //4 places after comma
	sprintf(buf, "%c%3d.%04d",tmpSign,tmpInt,tmpLast);
}

void revieve_usart_commands(char str[]){
	//TODO: write some logic behind it, so a laptop can command the mouse
}

void send_usart_string(char str[]){
	for (int i = 0; str[i] != '\0'; i++){
		send_usart_letter(str[i]);
	}
	send_usart_letter('\r');
	send_usart_letter('\n');
}

void send_usart_letter(char letter){
	//TODO: use pushing/flagging method instead how the main thread
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE)
		== RESET)
		{
		// wait for the transmitter to be empty
		}
	USART_SendData(USART3, letter);
}

void usart_init(){
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	GPIO_InitTypeDef gpioInitStruct;
	gpioInitStruct.GPIO_Mode = GPIO_Mode_AF;
	gpioInitStruct.GPIO_OType = GPIO_OType_PP;
	gpioInitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	gpioInitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpioInitStruct.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_Init(GPIOC, &gpioInitStruct);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_1);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	//Initialise usart
	USART_InitTypeDef usartInitStruct;
	usartInitStruct.USART_BaudRate = 115200;
	usartInitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usartInitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usartInitStruct.USART_Parity = USART_Parity_No;
	usartInitStruct.USART_StopBits = USART_StopBits_1;
	usartInitStruct.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART3, &usartInitStruct);
	USART_Cmd(USART3, ENABLE);
}


