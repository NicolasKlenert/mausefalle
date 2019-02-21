/*
 * stepper.c
 *
 *  Created on: 14.01.2019
 *      Author: JoBire
 */
#include "main.h"
#include "lre_stepper.h"
#include "lre_execution_time.h"
#include "lre_usart.h"
#include "lre_wait.h"
#include "math.h"

// defines
#define TIMER_FREQ 1000000			// Frequency of the stepper timers TIM16 & TIM17
#define WHEEL_DIAMETER (float)81.5			// Wheel diameter in mm
#define STEPS_PER_MM (float)( 4096.0 / (M_PI * WHEEL_DIAMETER) )
#define ACC_STEPS_PER_SEC (int16_t)( ACCELERATION * STEPS_PER_MM )		// acceleration in steps/s^2

// typedef
typedef struct{
	uint16_t step_table[8];
	uint16_t reset_mask;
	int8_t counter;
	int32_t current_step;
	int16_t step_freq;			// steps/s
	int16_t desired_step_freq;	// steps/s
	int16_t max_distance;
	uint8_t active;
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
		0,
		0,
		0};

// function prototypes
void stepper_nextStep(stepper_struct *stepper);
void stepper_acceleration_ramp(TIM_TypeDef *tim, stepper_struct *stepper);

void lre_stepper_init(void)
{
	uint16_t portValue;

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

	// TIM IT init
	TIM_ITConfig(TIM16, TIM_IT_Update, ENABLE);
	TIM_ITConfig(TIM17, TIM_IT_Update, ENABLE);

	// NVIC config and enable
	NVIC_InitTypeDef nvicTIM16;
	nvicTIM16.NVIC_IRQChannel = TIM16_IRQn;
	nvicTIM16.NVIC_IRQChannelCmd = ENABLE;
	nvicTIM16.NVIC_IRQChannelPriority = 0;	// can be 0 to 3
	NVIC_Init(&nvicTIM16);

	NVIC_InitTypeDef nvicTIM17;
	nvicTIM17.NVIC_IRQChannel = TIM17_IRQn;
	nvicTIM17.NVIC_IRQChannelCmd = ENABLE;
	nvicTIM17.NVIC_IRQChannelPriority = 0;	// can be 0 to 3
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

	/* Set stepper1 and stepper2 to start position (step zero)
	 * do this by starting with step 8 and decrement down
	 * to zero for both steppers */

	portValue = GPIO_ReadOutputData(GPIOB);	// read old GPIOB Data

	for (int8_t i = 7; i >= 0; i--)
	{
		portValue &= stepper_right.reset_mask;	// reset stepper right Pins
		portValue &= stepper_left.reset_mask;	// reset stepper left Pins
		portValue |= stepper_right.step_table[i];	// set new Pins right
		portValue |= stepper_left.step_table[i];	// set new Pins left
		GPIO_Write(GPIOB, portValue);	// write new Pins
		lre_wait(10);		// equals a step freq of 100 Hz
	}
}

//returns state of stepper. (used to know if stepper is disabled)
uint8_t lre_stepper_idle(uint8_t stepper_x){
	if((stepper_x & STEPPER_RIGHT) && stepper_right.active){
		return FALSE;
	}
	if ((stepper_x & STEPPER_LEFT) && stepper_left.active){
		return FALSE;
	}
	return TRUE;
}

void lre_stepper_stop(uint8_t stepper_x)
{
	if(stepper_x & STEPPER_RIGHT){
		stepper_right.desired_step_freq = 0;	// set desired step freq to 0
		stepper_right.step_freq = 0;			// set step freq to 0
		stepper_right.current_step = 0;
		//disable stepper
		stepper_right.active = FALSE;
		TIM_SetAutoreload(TIM16, 0xFFFF);		// set the Timer period to maximum
		TIM_Cmd(TIM16, DISABLE);
	}
	if(stepper_x & STEPPER_LEFT){
		stepper_left.desired_step_freq = 0;
		stepper_left.step_freq = 0;
		stepper_left.current_step = 0;
		//disable stepper
		stepper_left.active = FALSE;
		TIM_SetAutoreload(TIM17, 0xFFFF);		// set the Timer period to maximum
		TIM_Cmd(TIM17, DISABLE);
	}
}

/* Updates the desired step frequency in the stepper struct
 *
 * @param speed: desired speed in mm/s (-90 ... 90)
 * @param stepper_x: STEPPER_RIGHT or STEPPER_LEFT or STEPPER_BOTH
 * @param max_distance: the maximal distance traveling is allowed in [mm]. If 0 is given, then is it seen as infinity
 *
 * */
void lre_stepper_setSpeed(int8_t speed_mm_s, uint8_t stepper_x)
{
	if (stepper_x & STEPPER_RIGHT)
	{
		// TIM start
		TIM_Cmd(TIM16, ENABLE);
		stepper_right.desired_step_freq = (int16_t)( -speed_mm_s * STEPS_PER_MM );	// negativ so steppers turn in same direction
		stepper_right.active = TRUE;
	}
	if (stepper_x & STEPPER_LEFT)
	{
		// TIM start
		TIM_Cmd(TIM17, ENABLE);
		stepper_left.desired_step_freq = (int16_t)( speed_mm_s * STEPS_PER_MM );
		stepper_left.active = TRUE;
	}
}

/* Updates the desired distance in the stepper struct
 *
 * @param max_distance: the maximal distance traveling is allowed in [mm]. If 0 is given, then is it seen as infinity
 * @param stepper_x: STEPPER_RIGHT or STEPPER_LEFT or STEPPER_BOTH
 *
 * */
void lre_stepper_setMaxDistance(int16_t max_distance, uint8_t stepper_x)
{
	if (stepper_x & STEPPER_RIGHT)
	{
		stepper_right.max_distance = max_distance;
	}
	if (stepper_x & STEPPER_LEFT)
	{
		// TIM start
		stepper_left.max_distance = max_distance;
	}
}

int16_t lre_stepper_getMovedDistance(uint8_t stepper_x)
{
	if (stepper_x & STEPPER_RIGHT)
	{
		return -stepper_right.current_step / STEPS_PER_MM;	// negativ because right stepper is inverted
	}
	if (stepper_x & STEPPER_LEFT)
	{
		return +stepper_left.current_step / STEPS_PER_MM;
	}
	send_usart_string("get_moved_distance kann den stepper nicht zuordnen");
	return 0;
}

int8_t decrement_modulo(int8_t number, int8_t modulo){
	if(number == 0){
		return modulo -1;
	}
	return --number;
}

void stepper_nextStep(stepper_struct *stepper)
{
	uint16_t portValue = 0;

	if (stepper->step_freq > 0)
	{
		stepper->counter++;
		stepper->counter = stepper->counter%8;
		stepper->current_step++;
	}
	else if (stepper->step_freq < 0)
	{
		stepper->counter = decrement_modulo(stepper->counter, 8);
		stepper->current_step--;
	}
	else
	{
		return;
	}
//	char string[50];
//	sprintf(string, "%d",stepper->counter);
//	send_usart_string(string);
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
		// TODO this could be a problem when going through zero and hitting zero exactly
		TIM_Cmd(tim, DISABLE);
		stepper->active = FALSE;
	}
	else
	{
		timer_period = TIMER_FREQ / abs( stepper->step_freq ) - 1;
	}
	TIM_SetAutoreload(tim, timer_period);		// set the new Timer period
}

/* Execution time: 5us ... 15us
 *
 * */
void TIM16_IRQHandler(void)
{
	// check which interrupt event occurred
	if (SET == TIM_GetITStatus(TIM16, TIM_IT_Update))
	{
		if(abs(lre_stepper_getMovedDistance(STEPPER_RIGHT)) >= abs(stepper_right.max_distance) && stepper_right.max_distance != 0){
			// already traveled max_distance
			lre_stepper_stop(STEPPER_RIGHT);
		}else{
			// perform next step
			stepper_nextStep(&stepper_right);
			// check if the step frequency has to be changed
			if (stepper_right.step_freq != stepper_right.desired_step_freq)
				{
				stepper_acceleration_ramp(TIM16, &stepper_right);		// change the frequency according to acceleration
			}
		}
		// reset ITPendingBit
		TIM_ClearITPendingBit(TIM16, TIM_IT_Update);
	}
}

/* Execution time: 5us ... 15us
 *
 * */
void TIM17_IRQHandler(void)
{
	// check which interrupt event occurred
	if (SET == TIM_GetITStatus(TIM17, TIM_IT_Update))
	{
		if(abs(lre_stepper_getMovedDistance(STEPPER_LEFT)) >= abs(stepper_left.max_distance) && stepper_left.max_distance != 0){
			// already traveled max_distance
			lre_stepper_stop(STEPPER_LEFT);
		}else{
			// perform next step
			stepper_nextStep(&stepper_left);
			// check if the step frequency has to be changed
			if (stepper_left.step_freq != stepper_left.desired_step_freq)
			{
				stepper_acceleration_ramp(TIM17, &stepper_left);		// change the frequency according to acceleration
			}
		}
		// reset ITPendingBit
		TIM_ClearITPendingBit(TIM17, TIM_IT_Update);
	}
}
