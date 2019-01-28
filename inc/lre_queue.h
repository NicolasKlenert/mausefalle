/*
 * lre_queue.h
 *
 *  Created on: 24.12.2018
 *      Author: Nicol
 */

#ifndef LRE_QUEUE_H_
#define LRE_QUEUE_H_

/*
 * lre_queue.c is a simple implementation of a queue (yeah there is no standard queue in c)
 *
 *  Created on: 24.12.2018
 *      Author: Nicolas Klenert
 */

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

#ifndef TRUE
#define TRUE		1
#define FALSE		0
#endif

struct Queue{
	uint16_t front, rear, size;
	uint16_t capacity;
	uint16_t* arr;
};

struct Queue* createQueue(uint16_t capacity);
uint16_t isFull(struct Queue* queue);
uint16_t isEmpty(struct Queue* queue);
uint16_t enqueue(struct Queue* queue, uint16_t item);
uint16_t dequeue(struct Queue* queue);
uint16_t touch(struct Queue* queue);

#endif /* LRE_QUEUE_H_ */
