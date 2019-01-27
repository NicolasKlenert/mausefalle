/*
 * lre_execution_time.c
 *
 *  Created on: 27.01.2019
 *      Author: JoBire
 */

#include "lre_execution_time.h"

void lre_execution_time_init()
{
	// RCC enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	// TIM init struct
	TIM_TimeBaseInitTypeDef timerInitStruct;
	timerInitStruct.TIM_ClockDivision = 0;
	timerInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStruct.TIM_Period = 0xFFFFFFFF;
	timerInitStruct.TIM_Prescaler = SystemCoreClock / 1000000 - 1;	// TIM freq = 1MHz
	timerInitStruct.TIM_RepetitionCounter = 0;

	// TIM init
	TIM_TimeBaseInit(TIM2, &timerInitStruct);

	// TIM start
	TIM_Cmd(TIM2, ENABLE);

	execution_time = 0;
}

/* start measuring time
 *
 * */
void lre_execution_time_tic()
{
	TIM_SetCounter(TIM2, (uint32_t)0);
}

/* Store elapsed time since calling the tic function in a global variable
 * Time is measured in microseconds
 *
 * */
void lre_execution_time_toc()
{
	execution_time = TIM_GetCounter(TIM2);
}
