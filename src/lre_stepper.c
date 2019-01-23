/*
 * stepper.c
 *
 *  Created on: 14.01.2019
 *      Author: JoBire
 */

#include "lre_stepper.h"

// defines
#define TIMER_FREQ 1000000			// Frequency of the stepper timers TIM16 & TIM17
#define ACCELERATION (float)30.0			// acceleration in mm/s^2
#define WHEEL_DIAMETER (float)80.0			// Wheel diameter in mm
#define STEPS_PER_MM (float)( 4096.0 / (3.14159265359 * WHEEL_DIAMETER) )
#define ACC_STEPS_PER_SEC (int16_t)( ACCELERATION * STEPS_PER_MM )		// acceleration in steps/s^2

// typedef
typedef struct{
	uint16_t step_table[8];
	uint16_t reset_mask;
	uint8_t counter;
	int32_t current_step;
	int16_t step_freq;			// steps/s
	int16_t desired_step_freq;	// steps/s
}stepper_struct;

// variables
stepper_struct stepper_left = {{
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
		0,
		0};
stepper_struct stepper_right = {{
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
		0,
		0};

// function prototypes
void stepper_nextStep(stepper_struct *stepper);
void stepper_acceleration_ramp(TIM_TypeDef *tim, stepper_struct *stepper);
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
	timerInitStruct.TIM_Period = 0xFFFF;
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
	nvicTIM17.NVIC_IRQChannel = TIM17_IRQn;
	nvicTIM17.NVIC_IRQChannelCmd = ENABLE;
	nvicTIM17.NVIC_IRQChannelPriority = 1;	// can be 0 to 3
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

void lre_stepper_stop(void)
{
	stepper_right.desired_step_freq = 0;	// set desired step freq to 0
	stepper_right.step_freq = 0;			// set step freq to 0
	TIM_SetAutoreload(TIM16, 0xFFFF);		// set the Timer period to maximum

	stepper_left.desired_step_freq = 0;
	stepper_left.step_freq = 0;
	TIM_SetAutoreload(TIM17, 0xFFFF);		// set the Timer period to maximum
}

/* Updates the desired step frequency in the stepper struct
 *
 * @param speed: desired speed in mm/s (-90 ... 90)
 * @param stepper_x: STEPPER_RIGHT or STEPPER_LEFT
 *
 * */
void lre_stepper_setSpeed(int8_t speed_mm_s, uint8_t stepper_x)
{
	if (stepper_x == STEPPER_RIGHT)
	{
		stepper_right.desired_step_freq = (int16_t)( speed_mm_s * STEPS_PER_MM );	// convert the speed to a step frequency
	}
	else if (stepper_x == STEPPER_LEFT)
	{
		stepper_left.desired_step_freq = (int16_t)( -speed_mm_s * STEPS_PER_MM );	// negativ so steppers turn in same direction
	}
}

int16_t lre_stepper_getMovedDistance(uint8_t stepper_x)
{
	if (stepper_x == STEPPER_RIGHT)
	{
		return (uint16_t)stepper_right.current_step / STEPS_PER_MM;
	}
	else if (stepper_x == STEPPER_LEFT)
	{
		return (uint16_t)-stepper_left.current_step / STEPS_PER_MM;		// negativ because left stepper ist inverted
	}
	return 0;
}

void stepper_nextStep(stepper_struct *stepper)
{
	uint16_t portValue = 0;

	if (stepper->step_freq > 0)
	{
		stepper->counter = increment_counter(stepper->counter);
		stepper->current_step++;
	}
	else if (stepper->step_freq < 0)
	{
		stepper->counter = decrement_counter(stepper->counter);
		stepper->current_step--;
	}
	else
	{
		return;
	}
	portValue = GPIO_ReadOutputData(GPIOB);	// read old GPIOB Data
	portValue &= stepper->reset_mask;	// reset stepper Pins
	portValue |= stepper->step_table[stepper->counter];	// set new Pins
	GPIO_Write(GPIOB, portValue);	// write new Pins
}

void stepper_acceleration_ramp(TIM_TypeDef *tim, stepper_struct *stepper)
{
	int16_t max_freq_inc = 0;
	uint32_t timer_period = 0;
	uint32_t timer_period_old = tim->ARR;	// read the value of the autoreload register

	// calculate the maximum step frequency change based on the current timer period and the desired acceleration
	// 1 is added so the step_freq can increase above ACC_STEPS_PER_SEC
	max_freq_inc = ( ACC_STEPS_PER_SEC * ( timer_period_old + 1 ) ) / TIMER_FREQ + 1;
	// check if the difference is lower than the maximum step frequency change
	if (abs(stepper->desired_step_freq - stepper->step_freq) < max_freq_inc)
	{
		stepper->step_freq = stepper->desired_step_freq;
	}
	else
	{
		// acceleration
		if (stepper->desired_step_freq > stepper->step_freq)
		{
			stepper->step_freq += max_freq_inc;
		}
		// deceleration
		else
		{
			stepper->step_freq -= max_freq_inc;
		}
	}
	// if step_freq equals 0, set the longest possible timer period
	if (stepper->step_freq == 0)
	{
		timer_period = 0xFFFF;
	}
	else
	{
		timer_period = TIMER_FREQ / abs( stepper->step_freq ) - 1;
	}
	TIM_SetAutoreload(tim, timer_period);		// set the new Timer period
}

void TIM16_IRQHandler(void)
{
	// check which interrupt event occurred
	if (SET == TIM_GetITStatus(TIM16, TIM_IT_Update))
	{
		// reset ITPendingBit
		TIM_ClearITPendingBit(TIM16, TIM_IT_Update);
		// perform next step
		stepper_nextStep(&stepper_right);
		// check if the step frequency has to be changed
		if (stepper_right.step_freq != stepper_right.desired_step_freq)
		{
			stepper_acceleration_ramp(TIM16, &stepper_right);		// change the frequency according to acceleration
		}
	}
}

void TIM17_IRQHandler(void)
{
	// check which interrupt event occurred
	if (SET == TIM_GetITStatus(TIM17, TIM_IT_Update))
	{
		// reset ITPendingBit
		TIM_ClearITPendingBit(TIM17, TIM_IT_Update);
		// perform next step
		stepper_nextStep(&stepper_left);
		// check if the step frequency has to be changed
		if (stepper_left.step_freq != stepper_left.desired_step_freq)
		{
			stepper_acceleration_ramp(TIM17, &stepper_left);		// change the frequency according to acceleration
		}
	}
}

/* This function increments a counter keeping it between 0 and 7
 *
 * */
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

/* This function decrements a counter keeping it between 0 and 7
 *
 * */
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
