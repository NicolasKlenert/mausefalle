/*
 * test.c
 *
 *  Created on: 24.12.2018
 *      Author: Nicolas Klenert
 */

#include <test.h>
#include <labyrinth.h>
#include <lre_queue.h>

//tests if labyrinth has a border around it (and if you can go to the middle with all 4 corners)
uint16_t acceptableLabyrinth(){
	return FALSE;
}

uint16_t testPathFinding(){
	//create labyrinth
	createFakeLabyrinth();
	// set all cells visited
	for (int i = 0; i < 49; i++)
	{
		setVisited(i);
	}
	//path from upper left corner: 0,7,14,15,22,29,36,37,44,45,38,31,32,33,26,27,20,19,12,11,18,17,16,23,24
	uint16_t correct[25] = {0,7,14,15,22,29,36,37,44,45,38,31,32,33,26,27,20,19,12,11,18,17,16,23,24};
	uint16_t path[numCols*numRows] = {0};
	uint16_t count = getPath(0, goal, path, numCols*numRows);
	for(int i = 0; i < 25; ++i){
		if(path[count+i] != correct[i]) return FALSE;
	}
	return TRUE;
}

uint16_t testQueue(){
	struct Queue* queue = createQueue(49);
	if(!dequeue(queue)) return FALSE;
	enqueue(queue,1);
	enqueue(queue,2);
	enqueue(queue,3);
	enqueue(queue,4);
//	if(enqueue(queue,5)) return FALSE;
	if(dequeue(queue) != 1) return FALSE;
	if(dequeue(queue) != 2) return FALSE;
	return TRUE;
}

void testAll(){
	if(!testQueue()) printf("\n Queue funktioniert nicht richtig! \n");
	if(!testPathFinding()) printf("\n BFS funktioniert nicht! \n");

}
