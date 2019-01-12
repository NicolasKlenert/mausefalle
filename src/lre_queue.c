/*
 * lre_queue.c is a simple implementation of a queue (yeah there is no standard queue in c)
 *
 *  Created on: 24.12.2018
 *      Author: Nicolas Klenert
 */

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include "lre_queue.h"

#define TRUE		1
#define FALSE		0

struct Queue* createQueue(uint16_t capacity){
	//dynamic memory allocation
	struct Queue* queue = (struct Queue*) malloc(sizeof(struct Queue));
	queue->capacity = capacity;
	queue->front = queue->size = 0;
	queue->rear = capacity-1;
	queue->arr = (uint16_t*) malloc(queue->capacity * sizeof(uint16_t));
	return queue;
}

uint16_t isFull(struct Queue* queue){
	return (queue->size == queue->capacity);
}

uint16_t isEmpty(struct Queue* queue){
	return (queue->size == 0);
}

//enqueue an item. Return True if queue has enough capacity. otherwise it returns false
uint16_t enqueue(struct Queue* queue, uint16_t item){
	if(isFull(queue)) return FALSE;
	//we use a circle method, so we dont have to reallocate all items after a dequeue
	queue->rear = (queue->rear +1) % queue->capacity;
	queue->size = queue->size + 1;
	queue->arr[queue->rear] = item;
	return TRUE;
}

uint16_t dequeue(struct Queue* queue){
	if(isEmpty(queue)) return 0b1111111111111111;	//max unsigned int
	uint16_t item = queue->arr[queue->front];
	queue->front = (queue->front +1) % queue->capacity;
	queue->size = queue->size -1;
	return item;
}

//method to look at the next item without dequeuing it
uint16_t touch(struct Queue* queue){
	if(isEmpty(queue)) return 0b1111111111111111;	//max unsigned int
	return queue->arr[queue->front];
}
