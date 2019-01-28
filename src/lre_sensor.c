/*
 * lre_sensor.c
 *
 *  Created on: 13.01.2019
 *      Author: Nicolas Klenert
 */

#include "stm32f0xx.h"
#include "stm32f072b_discovery.h"
#include "mouse.h"
#include "lre_sensor.h"

#define TRIGGERLENGTH	100

void lre_sensor_init(){
	//initialise echo gpio
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_InitTypeDef gpio_initStruct;
	gpio_initStruct.GPIO_Mode = GPIO_Mode_IN;
	gpio_initStruct.GPIO_OType = GPIO_OType_PP;	//not important
	gpio_initStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_15;
	gpio_initStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpio_initStruct.GPIO_Speed = GPIO_Speed_50MHz; //falls störungen vorhanden sind, drossel auf 2MHz
	GPIO_Init(GPIOA, &gpio_initStruct);

	//configure EXTI
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,
	ENABLE);
	EXTI_InitTypeDef extiInitStruct;
	extiInitStruct.EXTI_Line = EXTI_Line4| EXTI_Line5 | EXTI_Line15;
	//TODO: it could be that every EXTI_Line has to be initialized separately
	extiInitStruct.EXTI_LineCmd = ENABLE;
	extiInitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	extiInitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_Init(&extiInitStruct);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource4|EXTI_PinSource5 |EXTI_PinSource15);
	//TODO: here you also have to change the PINsource, if every line has to be configured separately
	NVIC_EnableIRQ(EXTI4_15_IRQn);

	//configure Timer
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	TIM_TimeBaseInitTypeDef timerInitStruct;
	timerInitStruct.TIM_ClockDivision = 0;
	timerInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStruct.TIM_Period = 60000 - 1;
	timerInitStruct.TIM_Prescaler = SystemCoreClock / (1000000 - 1);
	timerInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &timerInitStruct);

	//initialise echo gpio
	//gpioa bus already active
	gpio_initStruct.GPIO_Mode = GPIO_Mode_AF;
	gpio_initStruct.GPIO_OType = GPIO_OType_PP;	//not important
	gpio_initStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
	gpio_initStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpio_initStruct.GPIO_Speed = GPIO_Speed_50MHz; //falls störungen vorhanden sind, drossel auf 2MHz
	GPIO_Init(GPIOA, &gpio_initStruct);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_2);
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);?
	TIM_OCInitTypeDef ocInitStruct;
	ocInitStruct.TIM_OCIdleState = TIM_OCIdleState_Set;
	ocInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Set;
	ocInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	ocInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	ocInitStruct.TIM_OCNPolarity = TIM_OCNPolarity_High;
	ocInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	ocInitStruct.TIM_OutputNState = TIM_OutputNState_Disable;
	ocInitStruct.TIM_Pulse = 0;
	//by the starthandler: acivate the next pmw with TIM_SetComparex(TIM3, 10)
	//AND let the current one stall with TIM_SetComparex(TIM3, 0)
	TIM_OC1Init(TIM1, &ocInitStruct);
	TIM_OC2Init(TIM1, &ocInitStruct);
	TIM_OC3Init(TIM1, &ocInitStruct);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	mouse_distance[0] = 0;
	mouse_distance[1] = 0;
	mouse_distance[2] = 0;
}

void lre_sensor_start(){
	TIM_SetCompare1(TIM1, TRIGGERLENGTH);
	TIM_Cmd(TIM1, ENABLE);
}

void EXTI4_15_IRQHandler(void){
	if(EXTI_GetITStatus(EXTI_Line4) == SET){
		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4) == SET){
			//steigende Flanke
			startTime[0] = TIM_GetCounter(TIM1);
			TIM_SetCompare1(TIM1,0);
			TIM_SetCompare2(TIM1,TRIGGERLENGTH);
		}else{
			//fallende Flanke
			endTime[0] = TIM_GetCounter(TIM1);
			mouse_distance[0] = (endTime[0] - startTime[0])*0.017; //[cm]
		}
		EXTI_ClearITPendingBit(EXTI_Line4);
	}else if(EXTI_GetITStatus(EXTI_Line5) == SET){
		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == SET){
			//steigende Flanke
			startTime[1] = TIM_GetCounter(TIM1);
			TIM_SetCompare2(TIM1,0);
			TIM_SetCompare3(TIM1,TRIGGERLENGTH);
		}else{
			//fallende Flanke
			endTime[1] = TIM_GetCounter(TIM1);
			mouse_distance[1] = (endTime[1] - startTime[1])*0.017; //[cm]
		}
		EXTI_ClearITPendingBit(EXTI_Line5);
	}else{
		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15) == SET){
			//steigende Flanke
			startTime[2] = TIM_GetCounter(TIM1);
			TIM_SetCompare3(TIM1,0);
			TIM_SetCompare1(TIM1,TRIGGERLENGTH);
		}else{
			//fallende Flanke
			endTime[2] = TIM_GetCounter(TIM1);
			mouse_distance[2] = (endTime[2] - startTime[2])*0.017; //[cm]
		}
		EXTI_ClearITPendingBit(EXTI_Line15);
	}
}
