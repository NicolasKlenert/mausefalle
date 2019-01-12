/*
 * lre_wait.c
 *
 *  Created on: Nov 1, 2017
 *      Author: M. Lehmann
 *
 *      Wait function for STM32F0
 */

#include <lre_wait.h>

volatile uint32_t lre_wait_systickTime; // contains the time since last initiation in ms

/**
  * @brief  initiates the system tick timer to 1000 ticks per second
  * @param  None
  * @retval None
  */
void lre_wait_init(void) {
    // reset systick time
    lre_wait_systickTime = 0;
    // activate systick timer to count 1000 times per second
    SysTick_Config(SystemCoreClock / 1000);
}

/**
 * @brief  SysTick interrupt handler.
 */
void SysTick_Handler(void) {
    // increase system ticks. The 32-bit register overflows after 49.71 days
    lre_wait_systickTime++;
}

/**
  * @brief  waits for the given number of milliseconds. Does nothing while waiting
  * @param  waitTime: wait time in ms
  * @retval None
  */
void lre_wait(uint32_t waitTime) {
	// reset timer to zero
	SysTick->VAL = 0;
    // add the desired waiting time to the actual system time
    waitTime = lre_wait_systickTime + waitTime;

    // wait until the SysTick interrupt handler has decreased to 0
    while (waitTime > lre_wait_systickTime) {
        // do nothing
    }
}
