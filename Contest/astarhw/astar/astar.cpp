#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/mman.h>
#include "astar_path.h"
#include "omapl138_gpiofuncs.h"
#include "pQueue.h"
#include "../../sharedmem_com/sharedmem.h"
#define MAP_SIZE 4096UL
#define MAP_MASK (MAP_SIZE - 1)
#define DELAYTIME 100000

// functions declared at bottom of this file
int mygetch(void);
void gs_killapp(int s);
//int init_server(void);
//void sd_signal_handler_IO (int status);
//int run_server(void);
//void writefloats_sharedmem_5times(void);

sharedmemstruct* myshared;  // pointer to shared memory stucture used to communicate between Linux and the OMAPL138's DSP

volatile GPIOregs* mygpio;  // pointer to GPIO registers so Linux can control GPIO Pins
							//int TCPIPtransmissionerror = 0;		// Initialize transmission error count
							//int TCPIPbeginnewdata = 0;
							//int TCPIPdatasize = 0;
							//int numTXChars = 0;

							//char tmpbuf[LINUX_COMSIZE];

							//long tcpipcount = 0;

							//#define INBUFFSIZE (256)
							//#define OUTBUFFSIZE (INBUFFSIZE + 2)
							//volatile char TCPIPMessageArray[INBUFFSIZE];
							//volatile char tempReadbackToCleanCache[INBUFFSIZE];
							//volatile int tempReadbackSize;

							//volatile unsigned int delaycount = 0;

int fd;  // variables used to map to physical memory addresses
void *map_base;
off_t target;

//char inbuf[INBUFFSIZE];  // TCPIP input and output buffers
//unsigned char outbuf[OUTBUFFSIZE];
//int accepted_skt;

//float DSPFloats[NUM_FLOATS_FROM_LINUX_TO_DSP];  // temporay variables used in the communication with DSP core    
//volatile float tempFloats[NUM_FLOATS_FROM_LINUX_TO_DSP];
//int firsttime = 1;

/***** Astar Variables *****/
int RobotRow = 0;
int RobotCol = 0;
int DestRow = 0;
int DestCol = 0;


char map[176] = 		//16x11
{ '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
'x', 'x', 'x', 'x', '0', '0', '0', 'x', 'x', 'x', 'x',
'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0' };
/*{   '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
'0', '0', '0', '0', '0', '0', '0', 'x', 'x', 'x', 'x',
'0', '0', '0', '0', '0', '0', '0', 'x', '0', 'x', '0',
'x', 'x', '0', '0', '0', '0', '0', 'x', 'x', 'x', 'x',
'0', 'x', '0', '0', '0', '0', '0', '0', '0', '0', '0',
'x', 'x', 'x', 'x', '0', '0', '0', '0', '0', '0', '0',
'0', 'x', '0', 'x', '0', '0', '0', '0', '0', '0', '0',
'0', 'x', 'x', 'x', '0', '0', '0', '0', '0', 'x', 'x',
'0', '0', '0', '0', '0', '0', '0', '0', '0', 'x', '0',
'0', '0', '0', '0', '0', '0', '0', '0', '0', 'x', 'x',
'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
'x', 'x', 'x', 'x', '0', '0', '0', 'x', 'x', 'x', 'x',
'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'   }; */


//global variables necessary for the algorithm
node_t neighbors[8];		//used to get a list of neighboring nodes
int currDist = 0;	//distance traveled from starting point
int pathLen = 0;			//used to keep track of number of points in reconstructed path
int pathRow[400];			//reconstructed path in reverse order
int pathCol[400];
char map[176];
int mapRowSize = 16;
int mapColSize = 11;
dictElem_t nodeTrack[176];		//to track location of nodes in the heap, and to track parents of nodes technically only needs to be mapRowSize*mapColsize long
heap_t openSet, closedSet;

//preconditions: rowCurr, colCurr, rowGoal, and colGoal are feasible locations in the matrix
//returns the distance between two points as the sum of the row and column differences "Manhattan" distance on a square grid
int heuristic(int rowCurr, int colCurr, int rowGoal, int colGoal)
{
	return 1000 * sqrtf((abs(rowGoal - rowCurr))*(abs(rowGoal - rowCurr)) + (abs(colGoal - colCurr))*(abs(colGoal - colCurr)); // change later
}

//assumes that canTravel is called with neighboring points of a valid starting point
//precondition: row, col must be valid indices for a matrix
//returns 0 for false and 1 for true
//(row, col) is considered an unacceptable traveling point if it is represented as an 'x' in map, or if it is out of bounds of the map
int canTravel(int row, int col)
{
	//check for out of bounds
	int mapIdx = row*mapColSize + col;		//map data stored as 1d array
	if (mapIdx >= (mapRowSize*mapColSize) || mapIdx < 0)		//index out of bounds
		return 0;		//0 for cannot travel
	else if (col >= mapColSize || col < 0)			//too many columns, will result in a location in the next row
		return 0;
	else if (map[mapIdx] == 'x')	//wall reached, cannot travel
		return 0;
	else
		return 1;
}

//parameters rowCurr and colCurr must be valid starting points
//returns the number of neighbors which are viable traveling points
//fills the node_t array called neighbors, with neighboring nodes to which the path can travel
//note: not using diagonal neighbors here, so there are only for possibilities for neighboring nodes
int getNeighbors(int rowCurr, int colCurr)
{
	node_t nodeToAdd;
	int numNeighbors = 0;
	if (canTravel(rowCurr - 1, colCurr) == 1)	//can travel up
	{
		nodeToAdd.row = rowCurr - 1;
		nodeToAdd.col = colCurr;
		neighbors[numNeighbors] = nodeToAdd;
		numNeighbors++;
	}
	if (canTravel(rowCurr, colCurr - 1) == 1)	//can travel left
	{
		nodeToAdd.row = rowCurr;
		nodeToAdd.col = colCurr - 1;
		neighbors[numNeighbors] = nodeToAdd;
		numNeighbors++;
	}
	if (canTravel(rowCurr, colCurr + 1) == 1)	//can travel right
	{
		nodeToAdd.row = rowCurr;
		nodeToAdd.col = colCurr + 1;
		neighbors[numNeighbors] = nodeToAdd;
		numNeighbors++;
	}
	if (canTravel(rowCurr + 1, colCurr) == 1)	//can travel down
	{
		nodeToAdd.row = rowCurr + 1;
		nodeToAdd.col = colCurr;
		neighbors[numNeighbors] = nodeToAdd;
		numNeighbors++;
	}
	if (canTravel(rowCurr + 1, colCurr + 1) == 1)	//can travel upright
	{
		nodeToAdd.row = rowCurr + 1;
		nodeToAdd.col = colCurr + 1;
		neighbors[numNeighbors] = nodeToAdd;
		numNeighbors++;
	}
	if (canTravel(rowCurr + 1, colCurr - 1) == 1)	//can travel upleft
	{
		nodeToAdd.row = rowCurr + 1;
		nodeToAdd.col = colCurr - 1;
		neighbors[numNeighbors] = nodeToAdd;
		numNeighbors++;
	}
	if (canTravel(rowCurr - 1, colCurr + 1) == 1)	//can travel bottomright
	{
		nodeToAdd.row = rowCurr - 1;
		nodeToAdd.col = colCurr + 1;
		neighbors[numNeighbors] = nodeToAdd;
		numNeighbors++;
	}
	if (canTravel(rowCurr - 1, colCurr - ) == 1)	//can travel bottomleft
	{
		nodeToAdd.row = rowCurr - 1;
		nodeToAdd.col = colCurr - 1;
		neighbors[numNeighbors] = nodeToAdd;
		numNeighbors++;
	}

	return numNeighbors;
}

//helper function called inside astar to return the path from the goal point back to the starting point
//the list of points is put into the array pathRow and pathCol in reverse order, pathLen is the number of inserted points
void reconstructPath(int rowEnd, int colEnd, dictElem_t nodeTrack[])
{
	pathLen = 0;		//global variable, reset so start inserting at beginning of path array
	int currRow = rowEnd;
	int currCol = colEnd;
	while (currRow != 400 && currCol != 400)
	{
		//while node is not null "400", put it into path starting at end point
		pathRow[pathLen] = currRow;  //global array for Row
		pathCol[pathLen] = currCol;  //global array for Column
		pathLen++;
		//	printf("currPath: (%d, %d), %d\n", pathRow[pathLen-1], pathCol[pathLen-1], pathLen);
		int nodeTrackIdx = currRow*mapColSize + currCol;
		currRow = nodeTrack[nodeTrackIdx].parentRow;
		currCol = nodeTrack[nodeTrackIdx].parentCol;
		//	printf("next location: (%d, %d), %d\n", currRow, currCol, pathLen);
	}
	//	printf("done with reconstruction\n");
}

//path planning algorithm
//parameters rowStart, colStart, rowEnd, and colEnd must be valid locations
//they must be both within the indices of the matrix size and must not be points where barriers ('x') exist
int astar(int rowStart, int colStart, int rowEnd, int colEnd)
{
	//pseudo code instruction: initialize open and closed sets
	//initialize a dictionary to keep track of parents of nodes for easy reconstruction and for keeping track of 
	//  heap indexes for easy retrieval from heaps
	//set the values of the dictionary, open and closed sets to be zero 
	// so that no old values sitting in memory will produce weird results
	int resetNodeCnt;
	for (resetNodeCnt = 0; resetNodeCnt<176; resetNodeCnt++)
	{
		nodeTrack[resetNodeCnt].heapId = ' ';
		nodeTrack[resetNodeCnt].heapIdx = 0;
		nodeTrack[resetNodeCnt].parentRow = 0;
		nodeTrack[resetNodeCnt].parentCol = 0;
	}

	startHeap(&openSet);
	startHeap(&closedSet);
	currDist = 0;
	node_t start;

	/* initialize a start node to be the starting position
	since this is the start node, it should have zero distance traveled from the start, the normal predicted distance to the goal,
	and the total distance is the sum of the distance traveled from the start and the distance to the goal.*/
	/* update the dictionary, use row and column location as a key in the dictionary,
	use 400 for null parent values and don't forget to indicate which heap, open or closed set, the node is being placed*/
	/* put starting node on open set*/

	start.row = rowStart;
	start.col = colStart;
	start.distTravelFromStart = 0;
	start.distToGoal = heuristic(rowStart, colStart, rowEnd, colEnd);
	start.totalDist = start.distTravelFromStart + start.distToGoal;
	int startIdx = (start.row*mapColSize) + start.col;
	(nodeTrack[startIdx]).heapId = 'o';		//o for open set
	nodeTrack[startIdx].parentRow = 400;	//use 400 as NULL, if 400, know reached beginning in reconstruction
	nodeTrack[startIdx].parentCol = 400;	//no parent value = 400 since out of bounds
	if (rowStart == rowEnd && colStart == colEnd)		//if start point is the end point, don't do anything more!!!
		return 2;
	push(start, &openSet, nodeTrack, mapColSize); // put start node on the openSet

	char goalFound = 'f'; //false

						  /*while open set not empty and goal not yet found*/
	while (openSet.numElems > 0 && goalFound != 't')
	{

		/*find the node with the least total distance (f_score) on the open list, call it q (called minDistNode in code)*/
		node_t minDistNode = peek(openSet);	//find node with least total cost, make that the current node

											/*set the current distance to the current node's distance traveled from the start.*/
											//1.  currDist is set to q's distance traveled from the Start.  Explain why this could be different from the Manhattan distance to the Start position 
											//    This question is just asking for comments.
		currDist = minDistNode.distTravelFromStart;

		(nodeTrack[(minDistNode.row*mapColSize) + minDistNode.col]).heapId = 'r';		//r for removed from any set

																						//2.  pop q (which is currently the minimum) off which queue? 
																						// Choose one of these two lines of code
																						// IF the Openset
		pop(&openSet, nodeTrack, mapColSize);
		// IF closedSet 
		//pop(&closedSet, nodeTrack, mapColSize);

		/*generate q's 4 neighbors*/
		// 3.  Pass q's row and col to getNeighbors
		int numNeighbors = getNeighbors(minDistNode.row, minDistNode.col);	//get list of neighbors

																			/*for each neighbor*/
		int cnt = 0;
		for (cnt = 0; cnt<numNeighbors; cnt++)	//for each found neighbor
		{
			// 4. Just add comments here.  Find where the structure node_t is defined and inside commments here copy its definition for
			// viewing reference.  
			// All the answer for 4. will be commented.
			node_t next = neighbors[cnt];

			/*if neighbor is the goal, stop the search*/
			/*update the dictionary so this last node's parents are set to the current node*/
			if ((next.row == rowEnd) && (next.col == colEnd))		//if neighbor is the goal, found the end, so stop the search
			{
				// 5.  set current neighbor's parents.  Set parentRow to q's row.  Set parentCol to q's col since q is the parent of this neighbor
				(nodeTrack[next.row*mapColSize + next.col]).parentRow = minDistNode.row;	 //set goal node's parent position to current position
				(nodeTrack[next.row*mapColSize + next.col]).parentCol = minDistNode.col;
				goalFound = 't';
				break;
			}

			/*neighbor.distTravelFromStart (g) = q.distTravelFromStart + distance between neighbor and q which is always 1 when search just top left bottom right*/
			// 6.  Set this neighbor's distance traveled from the start.  Remember you have the variable "currDist" that is the distance of q to Start
			if ((abs(minDistNode.row - next.row) == 1) && (abs(minDistNode.row - next.row) == 1)) {
				next.distTravelFromStart = currDist + 1500;
			}
			else {
				next.distTravelFromStart = currDist + 1000;
				/*neighbor.distToGoal (h) = distance from goal to neighbor, heuristic function	(estimated distance to goal)*/
				// 7.  Pass the correct parameters to "heuristic" to calculate the distance this neighbor is from the goal.
				//  Remember that we have the variables rowEnd and colEnd which are the grid coordinates of the goal 
				next.distToGoal = heuristic(next.row, next.col, rowEnd, colEnd);

				/*neighbor.totalDist (f) = neighbor.distTravelFromStart + neighbor.distToGoal
				(total estimated distance as sum of distance traveled from start and distance to goal)*/
				// 8.  Find f, (totalDist) for this neighbor
				next.totalDist = next.distTravelFromStart + next.distToGoal;


				// 9.  Just comments for this question.
				// Explain in your own words (not copying the comments below) what the next 19 lines of C code are doing

				/*if a node with the same position as neighbor is in the OPEN list
				which has a lower total distance than neighbor, skip putting this neighbor onto the open set*/
				//check if node is on the open set already
				int nodeTrackIdx = (next.row*mapColSize) + next.col;
				char skipPush = 'f';
				if (nodeTrack[nodeTrackIdx].heapId == 'o')	//on the open set
				{
					int heapIndex = nodeTrack[nodeTrackIdx].heapIdx;
					node_t fromOpen = openSet.elems[heapIndex];
					if (fromOpen.totalDist <= next.totalDist)
						skipPush = 't';		//skip putting this node onto the openset, a better one already on there
				}

				/*if a node with the same position as neighbor is in the CLOSED list
				which has a lower f than neighbor, skip putting this neighbor onto the open set*/
				else if (nodeTrack[nodeTrackIdx].heapId == 'c')		//on closed set
				{
					int heapIndex = nodeTrack[nodeTrackIdx].heapIdx;
					node_t fromClosed = closedSet.elems[heapIndex];
					if (fromClosed.totalDist <= next.totalDist)
						skipPush = 't';		//skip putting this node onto the openset, already part of possible solution
				}

				/*if not skipping putting this node on the set, then push node onto the open set
				and update the dictionary to indicate the node is on the open set and set the parents of this node to the current node*/
				//(can't be an ordinary else to the things above because then it won't push onto the open set if already on open or closed set)
				if (skipPush != 't')
				{
					nodeTrack[nodeTrackIdx].heapId = 'o';		//mark in nodetrack that this is going onto the open set
					(nodeTrack[nodeTrackIdx]).parentRow = minDistNode.row;	 //set neighbor's parent position to current position
					(nodeTrack[nodeTrackIdx]).parentCol = minDistNode.col;
					//10.  push this neighbor on which queue? 
					// Choose one of these two lines of code
					// IF openSet
					push(next, &openSet, nodeTrack, mapColSize);
					// IF closedSet
					//push(next, &closedSet, nodeTrack, mapColSize);

				}
			}/* end for loop*/

			int nodeTrackIdx = minDistNode.row*mapColSize + minDistNode.col;
			nodeTrack[nodeTrackIdx].heapId = 'c';
			//11.  push q (current node) on which queue? 
			// Choose one of these two lines of code
			// IF openSet
			//push(minDistNode, &openSet, nodeTrack, mapColSize);
			// IF closedSet
			push(minDistNode, &closedSet, nodeTrack, mapColSize);

		}  /*end while loop*/

		   /*if a path was found from the start to the goal, then reconstruct the path*/
		if (goalFound == 't') {
			// 12.  Pass the correct varaibles to "reconstructPath" in order for it to fill in the global arrays pathRow, pathCol
			//     and integer pathLen.  Note that the path is in reverse order in pathRow and pathCol.
			reconstructPath(rowEnd, colEnd, nodeTrack);
			return 1;  // found a path return 1
		}
		return 0;  // Did not find a path  return zero;

	}

	int runAstar(int startRow, int startCol, int endRow, int endCol)
	{
		int astarfoundpath = 0;
		mapRowSize = 16;
		mapColSize = 11;

		int i, j;
		for (i = 0; i<mapRowSize; i++)
		{
			for (j = 0; j<mapColSize; j++) {
				map[i*mapColSize + j] = myshared->sharedAstarMap[i*mapColSize + j];
				printf("%c ", map[i*mapColSize + j]);
			}
			printf("\n");
		}
		printf("\n");
		if (startRow >= 0 && startRow<mapRowSize && startCol >= 0 && startCol<mapColSize && endRow >= 0 && endRow<mapRowSize && endCol >= 0 && endCol<mapColSize)		//if in bounds{
		{
			if (map[startRow*mapColSize + startCol] != 'x' && map[endRow*mapColSize + endCol] != 'x')		//make sure valid start and end points
			{
				astarfoundpath = astar(startRow, startCol, endRow, endCol);		//solve
				if (astarfoundpath == 1) {
					myshared->sharedPathLen = pathLen;
					printf("pathLen %d\n", myshared->sharedPathLen);
					for (i = 0; i < pathLen; i++)		//put solution on map
					{
						int mapIdx = pathRow[i] * mapColSize + pathCol[i];
						map[mapIdx] = '-';
						myshared->sharedPathRow[i] = pathRow[i]; //we added these lines after discussion with michal
						myshared->sharedPathCol[i] = pathCol[i];

						printf("%d, %d; %d, %d \n", pathRow[i], pathCol[i], (11 - pathRow[i]), (pathCol[i] - 5));		//Print Astar coordinates, Robot coordinates
					}

					//print map with solution
					for (i = 0; i<mapRowSize; i++)
					{
						for (j = 0; j<mapColSize; j++) {
							printf("%c ", map[i*mapColSize + j]);
							myshared->sharedAstarMap[i*mapColSize + j] = map[i*mapColSize + j]; //why do we need to send this info to DSP?? 
						}
						printf("\n");
					}
					return 1;
				}
				else {
					printf("\n\n\nNo Path Found\n\n\n");
					return 0;
				}
				return 1;

			}
			else
				printf("start or stop point not a navigable point: sR%d, sC%d, eR%d, eC%d \n", startRow, startCol, endRow, endCol);
		}
		else
			printf("at least one entered value not in bounds of map size: sR%d, sC%d, eR%d, eC%d \n", startRow, startCol, endRow, endCol);

		return 0;
	}


	/*
	* main()
	*   process command line input
	*/
	int main(int argc, char **argv)
	{
		int index = 0;
		char buffer[200];  // used by fgets to read character string typed by user.
		char mychar;
		int inputdone = 0;
		float tempfloat = 0;
		float DVel = 0;
		float turn = 0.0;

		int startRow = 0, startCol = 0, endRow = 0, endCol = 0, i = 0, j = 0;


		/* ****************************************************************/
		/*  Memory map to physical memory spaces of the GPIO control registers and shared memory space */
		if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) {
			printf("/dev/mem could not be opened.\n");
			exit(1);
		}
		else {
			printf("/dev/mem opened.\n");
		}
		fflush(stdout);


		target = 0x01E26000;
		/* Map one page for shared memory structure*/
		map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, target);
		if (map_base == (void *)-1) {
			printf("Memory map failed.\n");
		}
		else {
			printf("gpio Struct mapped at address %p.\n", map_base);
		}
		fflush(stdout);
		mygpio = (GPIOregs *)map_base;

		target = 0x80000000;
		/* Map one page for shared memory structure*/
		map_base = mmap(0, 3 * MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, target & ~MAP_MASK);
		if (map_base == (void *)-1) {
			printf("Memory map failed.\n");
		}
		else {
			printf("Shared Struct mapped at address %p.\n", map_base);
		}
		fflush(stdout);

		myshared = (sharedmemstruct *)map_base;
		/* ******************************************************************/

		GPIO_setOutput(mygpio, LVDATA_TO_LINUX_BANK, LVDATA_TO_LINUX_FLAG, OUTPUT_HIGH);  // Set = linux is ready for LV data from DSP
		GPIO_setOutput(mygpio, LVDATA_FROM_LINUX_BANK, LVDATA_FROM_LINUX_FLAG, OUTPUT_LOW);  // Clear = linux will write LV data for shared memory for DSP
		GPIO_setOutput(mygpio, DATA_TO_LINUX_BANK, DATA_TO_LINUX_FLAG, OUTPUT_HIGH);  // Set = linux is ready for data from DSP
		GPIO_setOutput(mygpio, DATA_FROM_LINUX_BANK, DATA_FROM_LINUX_FLAG, OUTPUT_LOW);  // Clear = linux will write data for shared memory for DSP

		GPIO_setOutput(mygpio, ASTAR_FAILED_BANK, ASTAR_FAILED_FLAG, OUTPUT_LOW); // Initially set all astar flags to 0. 
		GPIO_setOutput(mygpio, ASTAR_COMMAND_BANK, ASTAR_COMMAND_FLAG, OUTPUT_LOW);
		GPIO_setOutput(mygpio, ASTAR_DONE_BANK, ASTAR_DONE_FLAG, OUTPUT_LOW);


		// stay in this while loop receiving input from the user until user exits by selecting option e or v  

		printf("Setting signal handler...\n");
		signal(SIGKILL, gs_killapp);
		signal(SIGINT, gs_killapp);
		printf("...OK\n");

		while (!gs_exit) {
			gs_quit = 0;
			while (!gs_quit) {  // sit inside this while checking for new data from the DSP until TCPIP connection disconnected
				sched_yield();  // allow other processes to run if necessary

				if (GPIO_getOutput(mygpio, ASTAR_COMMAND_BANK, ASTAR_COMMAND_FLAG) == 1) { // DSP Command to ASTAR?
					GPIO_setOutput(mygpio, ASTAR_COMMAND_BANK, ASTAR_COMMAND_FLAG, OUTPUT_LOW); // Reset Astar Command Flag
					GPIO_setOutput(mygpio, ASTAR_FAILED_BANK, ASTAR_FAILED_FLAG, OUTPUT_LOW); // Clear Failure GPIO Flag

					printf("Got Astar Command %d \n", myshared->astarTrigger);

					//Update start and end locations

					RobotRow = myshared->sharedRobotRow;
					RobotCol = myshared->sharedRobotCol;
					DestRow = myshared->sharedDestRow;
					DestCol = myshared->sharedDestCol;

					if (runAstar(RobotRow, RobotCol, DestRow, DestCol) == 1) {
						GPIO_setOutput(mygpio, ASTAR_DONE_BANK, ASTAR_DONE_FLAG, OUTPUT_HIGH); // Notify DSP that Astar is completed...
						GPIO_setOutput(mygpio, ASTAR_FAILED_BANK, ASTAR_FAILED_FLAG, OUTPUT_LOW); // ... succesfully
					}
					else {
						GPIO_setOutput(mygpio, ASTAR_DONE_BANK, ASTAR_DONE_FLAG, OUTPUT_HIGH); // Notify DSP that Astar is completed...
						GPIO_setOutput(mygpio, ASTAR_FAILED_BANK, ASTAR_FAILED_FLAG, OUTPUT_HIGH); // ... but failed
						printf("astar unsuccessful");

					}

				}

			}
		}

		if (munmap((void*)mygpio, MAP_SIZE) == -1) {
			printf("Memory unmap failed.\n");
		}
		if (munmap((void*)myshared, MAP_SIZE) == -1) {
			printf("Memory unmap failed.\n");
		}
	}






	int mygetch(void)
	{
		struct termios oldt,
			newt;
		int ch;
		tcgetattr(STDIN_FILENO, &oldt);
		newt = oldt;
		newt.c_lflag &= ~(ICANON | ECHO);
		tcsetattr(STDIN_FILENO, TCSANOW, &newt);
		ch = getchar();
		tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
		return ch;
	}

	/*
	* gs_killapp()
	*   ends application safely
	*
	*/
	void gs_killapp(int s)
	{
		printf("\nTerminating\n");

		gs_quit = 1;
		gs_exit = 1;
		close(gs_coms_skt);
		return;
	}

