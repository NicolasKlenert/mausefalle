/*
 * lre_move.h
 *
 *  Created on: 03.02.2019
 *      Author: Stefan
 */

#ifndef LRE_MOVE_H_
#define LRE_MOVE_H_


// function prototypes
void lre_controller_init();
void lre_controller_start();
void lre_controller_stop();
void move_straight(int16_t speed, int16_t distance, int16_t wallDistance);
#endif /* LRE_MOVE_H_ */
