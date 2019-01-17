/*
 * stepper.c
 *
 *  Created on: 14.01.2019
 *      Author: JoBire
 */

#include "lre_stepper.h"

// typedef
typedef struct{
	uint16_t step_table[8];
	uint16_t reset_mask;
	uint8_t counter;
	int32_t planned_steps;
	int32_t current_step;
}stepper_struct;

// variables
stepper_struct stepper1 = {{
		0b000100,
		0b001100,
		0b001000,
		0b011000,
		0b010000,
		0b110000,
		0b100000,
		0b100100 },
		0b1111111111000011,
		0,
		0,
		0};
stepper_struct stepper2 = {{
		0b0001000000,
		0b0011000000,
		0b0010000000,
		0b0110000000,
		0b0100000000,
		0b1100000000,
		0b1000000000,
		0b1001000000 },
		0b1111110000111111,
		0,
		0,
		0};

// function prototypes
uint8_t increment_counter(uint8_t counter);
uint8_t decrement_counter(uint8_t counter);

void lre_stepper_init(void)
{
	// RCC enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17, ENABLE);

	// TIM init struct
	TIM_TimeBaseInitTypeDef timerInitStruct;
	timerInitStruct.TIM_ClockDivision = 0;
	timerInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStruct.TIM_Period = 0xFFFF;	// will be changed by lre_stepperx_setSpeed()
	timerInitStruct.TIM_Prescaler = SystemCoreClock / TIMER_FREQ - 1;
	timerInitStruct.TIM_RepetitionCounter = 0;

	// TIM init
	TIM_TimeBaseInit(TIM16, &timerInitStruct);
	TIM_TimeBaseInit(TIM17, &timerInitStruct);

	// TIM start
	TIM_Cmd(TIM16, ENABLE);
	TIM_Cmd(TIM17, ENABLE);

	// TIM IT init
	TIM_ITConfig(TIM16, TIM_IT_Update, ENABLE);
	TIM_ITConfig(TIM17, TIM_IT_Update, ENABLE);

	// NVIC config and enable
	NVIC_InitTypeDef nvicTIM16;
	nvicTIM16.NVIC_IRQChannel = TIM16_IRQn;
	nvicTIM16.NVIC_IRQChannelCmd = ENABLE;
	nvicTIM16.NVIC_IRQChannelPriority = 1;	// can be 0 to 3
	NVIC_Init(&nvicTIM16);

	NVIC_InitTypeDef nvicTIM17;
	nvicTIM16.NVIC_IRQChannel = TIM17_IRQn;
	nvicTIM16.NVIC_IRQChannelCmd = ENABLE;
	nvicTIM16.NVIC_IRQChannelPriority = 1;	// can be 0 to 3
	NVIC_Init(&nvicTIM17);

	// GPIO init
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	GPIO_InitTypeDef gpio_initStruct;
	gpio_initStruct.GPIO_Mode = GPIO_Mode_OUT;
	gpio_initStruct.GPIO_OType = GPIO_OType_PP;
	gpio_initStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 |
			GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	gpio_initStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpio_initStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &gpio_initStruct);

	// Set stepper1 and stepper2 to start position
	GPIO_SetBits(GPIOB, GPIO_Pin_2 | GPIO_Pin_6);
}

/* Stellt die Timer Periode so ein, dass die gewünschte Step-Frequenz erreicht wird
 * ACHTUNG: Hier wird unter Umständen stark gerundet!
 *
 * @Param stepFreq: desired step frequency in Hz
 *
 * */
void lre_stepper1_setStepFreq(uint16_t stepFreq)
{
	TIM_SetAutoreload(TIM16, TIMER_FREQ / stepFreq - 1);
}

/* Stellt die Timer Periode so ein, dass die gewünschte Step-Frequenz erreicht wird
 * ACHTUNG: Hier wird unter Umständen stark gerundet!
 * @Param stepFreq: desired step frequency in Hz
 * */
void lre_stepper2_setStepFreq(uint16_t stepFreq)
{
	TIM_SetAutoreload(TIM17, TIMER_FREQ / stepFreq - 1);
}

/* Diese Funktion übergibt dem Schrittmotor eine Anzahl an auszuführenden Schritten
 *
 * ACHTUNG: Wird eine neue Schrittfolge übergeben bevor die alte vollständig ausgeführt wurde,
 * beginnt der Schrittmotor mit der neuen Schrittfolge und verwirft die Alte
 *
 * */
void lre_stepper1_step(int32_t steps)
{
	stepper1.planned_steps = steps;
	stepper1.current_step = 0;
}

/* Diese Funktion übergibt dem Schrittmotor eine Anzahl an auszuführenden Schritten
 *
 * ACHTUNG: Wird eine neue Schrittfolge übergeben bevor die alte vollständig ausgeführt wurde,
 * beginnt der Schrittmotor mit der neuen Schrittfolge und verwirft die Alte
 *
 * */
void lre_stepper2_step(int32_t steps)
{
	stepper2.planned_steps = steps;
	stepper2.current_step = 0;
}

void stepper_nextStep(stepper_struct *stepper)
{
	uint16_t portValue = 0;

	if (stepper->current_step < stepper->planned_steps)
	{
		stepper->counter = increment_counter(stepper->counter);
		stepper->current_step++;
	}
	else if (stepper->current_step > stepper->planned_steps)
	{
		stepper->counter = decrement_counter(stepper->counter);
		stepper->current_step--;
	}
	else
	{
		return;
	}
	portValue = GPIO_ReadOutputData(GPIOB);	// read old GPIOB Data
	portValue &= stepper->reset_mask;	// reset stepper 1 Pins
	portValue |= stepper->step_table[stepper->counter];	// set new Pins
	GPIO_Write(GPIOB, portValue);
}

void TIM16_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM16, TIM_IT_Update);
	stepper_nextStep(&stepper1);
}

void TIM17_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM17, TIM_IT_Update);
	stepper_nextStep(&stepper2);
}

uint8_t increment_counter(uint8_t counter)
{
	if (counter == 7)
	{
		return 0;
	}
	else
	{
		return ++counter;
	}
}

uint8_t decrement_counter(uint8_t counter)
{
	if (counter == 0)
	{
		return 7;
	}
	else
	{
		return --counter;
	}
}
