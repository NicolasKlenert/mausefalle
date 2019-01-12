/*
 * test.c
 *
 *  Created on: 24.12.2018
 *      Author: Nicolas Klenert
 */

#include <test.h>
#include <labyrinth.h>
#include <lre_queue.h>

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

//tests if labyrinth has a border around it (and if you can go to the middle with all 4 corners)
uint16_t acceptableLabyrinth(){
	return FALSE;
}

uint16_t testPathFinding(){
	//create labyrinth
	createFakeLabyrinth();
	//path from upper left corner: 0,7,14,15,22,29,36,37,44,45,38,31,32,33,26,27,20,19,12,11,18,17,16,23,24
	uint16_t correct[25] = {0,7,14,15,22,29,36,37,44,45,38,31,32,33,26,27,20,19,12,11,18,17,16,23,24};
	uint16_t path[numCols*numRows];
	uint16_t count = getPath(0,25,path);
	for(int i = 0; i < 25; ++i){
		if(path[count+i] != correct[i]) return FALSE;
	}
	return TRUE;
}

uint16_t testQueue(){
	struct Queue* queue = createQueue(4);
	if(!dequeue(queue)) return FALSE;
	enqueue(queue,1);
	enqueue(queue,2);
	enqueue(queue,3);
	enqueue(queue,4);
	if(!enqueue(queue,5)) return FALSE;
	if(dequeue(queue) != 4) return FALSE;
	if(dequeue(queue) != 3) return FALSE;
	return TRUE;
}

void testAll(){
	if(!testQueue()) printf("\n Queue funktioniert nicht richtig! \n");
	if(!testPathFinding()) printf("\n BFS funktioniert nicht! \n");

}
