/*
 * labyrinth.c
 *
 *  Created on: 24.12.2018
 *      Author: Nicolas Klenert
 */

#include "lre_queue.h"
#include "labyrinth.h"

void setWalls(uint16_t id, uint16_t bitflag){
	cells[id] = ~(~cells[id]|bitflag);
}

void setGates(uint16_t id,uint16_t bitflag){
	cells[id] = cells[id] | bitflag;
}

void setWall(uint16_t id,uint16_t direction){
	setWalls(id,1 << direction);
}

void setGate(uint16_t id,uint16_t direction){
	setGates(id,1 << direction);
}
uint16_t hasWall(uint16_t id,uint16_t direction){
	return !hasGate(id,direction);
}
uint16_t hasGate(uint16_t id,uint16_t direction){
	return cells[id] & (1 << direction);
}

uint16_t getCellId(uint16_t id,uint16_t direction){
	switch (direction) {
		case 0:
			return id - numCols;
			break;
		case 1:
			return id +1;
			break;
		case 2:
			return id + numCols;
			break;
		case 3:
			return id -1;
		default:
			return id;
	}
}

uint16_t inverseDirection(uint16_t direction){
	return rotateDirection(direction,2);
}

uint16_t rotateDirection(uint16_t direction, uint16_t times){
	return (direction+times)%4;
}

uint16_t getNeighbours(uint16_t id, uint16_t *arr){
	uint16_t counter = 0;
	if(hasGate(id,DIR_NORTH)){//cells[id] & 0b0001
		arr[counter] = id - numCols;
		counter++;
	}
	if(hasGate(id,DIR_EAST)){//cells[id] & 0b0010
		arr[counter] = id + 1;
		counter++;
	}
	if(hasGate(id,DIR_SOUTH)){ //cells[id] & 0b0100
			arr[counter] = id + numCols;
			counter++;
		}
	if(hasGate(id,DIR_WEST)){//cells[id] & 0b1000
		arr[counter] = id - 1;
		counter++;
	}
	return counter;
}

//IMPORTANT: the path is save on the end of the buffer array!
uint16_t getPath(uint16_t start, uint16_t aim, uint16_t *arr){
	//create queue and an visited array to make a BFS
	uint16_t size = numRows*numCols;
	if(sizeof(arr)/sizeof(arr[0]) < size){
		//length of buffer array has to be at least as much as the number of cells
		return FALSE;
	}
	struct Queue* queue = createQueue(size);
	uint16_t visited[size];
	uint16_t nuller = size+1;
	for(int i = 0; i < size; i++){
		visited[i] = nuller;
	}
	//put start in the queue and start loop
	enqueue(queue,start);
	visited[start] = start;

	uint16_t breaking = FALSE;
	uint16_t item;
	uint16_t neighbours[4];
	while(!isEmpty(queue) && !breaking){
		item = dequeue(queue);
		getNeighbours(item,neighbours);
		for (int i = 0; neighbours[i] != nuller; i++){
				if(visited[neighbours[i]] == nuller){
					visited[neighbours[i]] = item;
					if (neighbours[i] == aim){
						breaking = TRUE;
						break;
					}
					enqueue(queue,neighbours[i]);
				}
			}
		//empty neighbours so we can use it in the next iteration
		for (int i = 0; neighbours[i] != nuller; i++){
			neighbours[i] = nuller;
		}
	}

	//go through the visited array till we find the start again
	uint16_t counter = sizeof(arr)/sizeof(arr[0]);
	arr[counter] = aim;
	item = aim;
	while(item != start && counter >= 0){
		item = visited[item];
		counter--;
		arr[counter] = item;
	}
	return sizeof(arr)/sizeof(arr[0]) - counter;
}

