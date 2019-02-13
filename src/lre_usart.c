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
#include "cmd.h"
#include "lre_led_status.h"

char stringToSend[1000];
int16_t head = 0;
int16_t tail = 0;	//we only have the update function for the timer6

void hreadable_floats(float number, char *buf){
	char tmpSign = (number < 0) ? '-' : '+';
	float tmpVal = (number < 0) ? -number: number;
	int tmpInt = tmpVal;
	float tmpFrac = tmpVal - tmpInt;
	int tmpLast = tmpFrac * 10000; //4 places after comma
	sprintf(buf, "%c%3d.%04d",tmpSign,tmpInt,tmpLast);
}

void send_usart_string(char str[]){
	for (int i = 0; str[i] != '\0'; i++){
		stringToSend[head] = str[i];
		head = (head+1)%1000;
	}
	stringToSend[head] = '\r';
	head = (head+1)%1000;
	stringToSend[head] = '\n';
	head = (head+1)%1000;
	TIM_Cmd(TIM6, ENABLE);
}

void lre_usart_init(){
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
	// Enable UART RX not empty interrupt
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	// Enable NVIC for UART3 interrupt
	NVIC_InitTypeDef nvicUsart3;
	nvicUsart3.NVIC_IRQChannel = USART3_4_IRQn;
	nvicUsart3.NVIC_IRQChannelCmd = ENABLE;
	nvicUsart3.NVIC_IRQChannelPriority = 3;	// can be 0 to 3
	NVIC_Init(&nvicUsart3);
	//initialise timer 6 to help sending strings
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	TIM_TimeBaseInitTypeDef timerInitStruct;
	timerInitStruct.TIM_ClockDivision = 0;
	timerInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStruct.TIM_Period = 1;	//1000 - 1 length of string
	timerInitStruct.TIM_Prescaler = SystemCoreClock / (115200 - 1);	//115200 baudrate
	timerInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM6, &timerInitStruct);
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
	NVIC_InitTypeDef nvicUsartSend;
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
	TIM_ClearITPendingBit(TIM6, TIM_IT_Update);		//interrupt is called in the beginning. even if timer is not activated yet
	//WHY? How can you fix it? clearing the flag does not help.
}

/** Interrupt Handler for USART3
 *
 *
 */
void USART3_4_IRQHandler(void)
{
	if ( SET == USART_GetITStatus(USART3, USART_IT_RXNE) )
	{
		cmd_handler();
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
	}
}

/**	Interrupt for sending Strings
 *
 */
void TIM6_DAC_IRQHandler(void){
	if(!USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET){
		//send letter
		char letter = stringToSend[tail];//TIM_GetCounter(TIM6);
		tail = (tail+1)%1000;
		if(head == tail){
			TIM_Cmd(TIM6, DISABLE);
		}
		USART_SendData(USART3, letter);
	}else{
		//-> no one heard the letter....repeating? or ignoring?
//		mouse_setStatus(MOUSE_CRITICAL_ERROR);
//		led_status_show();
	}
	TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
}
