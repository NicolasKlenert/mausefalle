/*
 * labyrinth.c
 *
 *  Created on: 24.12.2018
 *      Author: Nicolas Klenert
 */

#include "lre_queue.h"
#include "labyrinth.h"
#include "math.h"

/**
 * cells are 8bit-bitflags.
 * The last bit indicates if the upper path is open (1) or not (0).
 * The one before is for the right side, before that the bottom and left side.
 * Example: 0b1001 stands for a cell which has walls on the east and south.
 *
 * The first bit says if the mouse had visited the room or if it is unkown.
 *
 * The array is the listing of the cells. We start to count at the upper left corner and count towards the left
 * example: a 3x3 maze has an array with ids:
 * 0,1,2
 * 3,4,5
 * 6,7,8
 *
 * The default labyrinth is a labyrinth with walls everywhere
 */
uint8_t cells[numRows*numCols];

void labyrinth_init(){
	for(uint16_t i = 0; i < numRows*numCols; i++){
		cells[i] = 0;
	}
}

/**
 *  _ _
 * |   |
 *     | -> mitte ist n/2 aufgerundet '\r''\n'
 * |___|
 */

uint8_t indexIsMid(uint8_t index, int max){
	return ceil(max/2) == index;
}

char printChar(uint8_t i, uint8_t j, uint16_t id){
	if(i == 0){
		if(j == 0){
			return upperLeftCornerChar;
		}else if(j == widthRoom -1){
			return upperRightCornerChar;
		}else if(indexIsMid(j,widthRoom)){
			if(hasGate(id,DIR_NORTH)){
				return ' ';
			}
			if(!isVisited(id)){
				return '?';
			}
		}
		return upperWallChar;
	}else if(i == heightRoom -1){
		if(j == 0){
			return bottomLeftCornerChar;
		}else if(j == widthRoom -1){
			return bottomRightCornerChar;
		}else if(indexIsMid(j,widthRoom)){
			if(hasGate(id,DIR_SOUTH)){
				return ' ';
			}
			if(!isVisited(id)){
				return '?';
			}
		}
		return bottomWallChar;
	}
	if(j == 0){
		if(indexIsMid(i,heightRoom)){
			if(!isVisited(id)){
				return '?';
			}
			if(hasGate(id,DIR_WEST)){
				return ' ';
			}
		}
		return leftWallChar;
	}else if(j == widthRoom -1){
		if(indexIsMid(i,heightRoom)){
			if(!isVisited(id)){
				return '?';
			}
			if(hasGate(id,DIR_EAST)){
				return ' ';
			}
		}
		return rightWallChar;
	}
	return ' ';
}

void printRoom(uint16_t id, char *arr){
	uint16_t x = (id%numCols)*widthRoom;
	uint16_t y = (id/numCols)*heightRoom;
	for(uint8_t i = 0; i < heightRoom; i++){
		for(uint8_t j = 0; j < widthRoom; j++){
			uint16_t index = (x+j)+((y+i)*((numCols*widthRoom)+2));
			arr[index] = printChar(i,j,id);
		}
	}

}

//arr has to be ((numCols*widthRoom)+2)*(numRows*heightRoom) big
void printLabyrinth(char *arr){
	for(uint16_t id = 0; id < numCols * numRows; id++){
		printRoom(id,arr);
	}
	uint16_t lineWidth = (numCols * widthRoom)+2;
	for(uint16_t i = 1; i <= numRows*heightRoom; i++){
		arr[(i*lineWidth)-2] = '\r';
		arr[(i*lineWidth)-1] = '\n';
	}
}

void setWalls(uint16_t id, uint8_t bitflag){
	cells[id] = ~(~cells[id]|bitflag);
}

void setGates(uint16_t id,uint8_t bitflag){
	cells[id] = cells[id] | bitflag;
}

void setWall(uint16_t id,uint8_t direction){
	setWalls(id,1 << direction);
}

void setGate(uint16_t id,uint8_t direction){
	setGates(id,1 << direction);
}

void setVisited(uint16_t id){
	cells[id] = cells[id] | (1 << 7);
}

uint8_t isVisited(uint16_t id){
	return cells[id] & (1 << 7);
}

uint8_t hasWall(uint16_t id,uint8_t direction){
	return !hasGate(id,direction);
}
uint8_t hasGate(uint16_t id,uint8_t direction){
	return cells[id] & (1 << direction);
}

uint16_t getCellId(uint16_t id,uint8_t direction){
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

uint8_t inverseDirection(uint8_t direction){
	return rotateDirection(direction,2);
}

uint8_t rotateDirection(uint8_t direction, uint8_t times){
	return (direction+times)%4;
}

uint8_t getNeighbours(uint16_t id, uint16_t *arr){
	uint8_t counter = 0;
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
uint8_t getPath(uint16_t start, uint16_t aim, uint16_t *arr){
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

	uint8_t breaking = FALSE;
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

void createFakeLabyrinth(){
	cells[0] = 0b0110;
	cells[1] = 0b1010;
	cells[2] = 0b1010;
	cells[3] = 0b1000;
	cells[4] = 0b0100;
	cells[5] = 0b0010;
	cells[6] = 0b1100;

	cells[7] = 0b0101;
	cells[8] = 0b0110;
	cells[9] = 0b1010;
	cells[10] = 0b1000;
	cells[11] = 0b0111;
	cells[12] = 0b1110;
	cells[13] = 0b1001;

	cells[14] = 0b0011;
	cells[15] = 0b1101;
	cells[16] = 0b0110;
	cells[17] = 0b1010;
	cells[18] = 0b1101;
	cells[19] = 0b0011;
	cells[20] = 0b1100;

	cells[21] = 0b0110;
	cells[22] = 0b1101;
	cells[23] = 0b0011;
	cells[24] = 0b1000;
	cells[25] = 0b0001;
	cells[26] = 0b0110;
	cells[27] = 0b1001;

	cells[28] = 0b0101;
	cells[29] = 0b0101;
	cells[30] = 0b0100;
	cells[31] = 0b0110;
	cells[32] = 0b1010;
	cells[33] = 0b1001;
	cells[34] = 0b0100;

	cells[35] = 0b0101;
	cells[36] = 0b0011;
	cells[37] = 0b1101;
	cells[38] = 0b0101;
	cells[39] = 0b0010;
	cells[40] = 0b1010;
	cells[41] = 0b1101;

	cells[42] = 0b0001;
	cells[43] = 0b0010;
	cells[44] = 0b1011;
	cells[45] = 0b1011;
	cells[46] = 0b1010;
	cells[47] = 0b1010;
	cells[48] = 0b1001;
}

