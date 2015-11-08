#include "Headers/PathPlanning.h"
#include "Headers/CudaErrorCheckFunctions.h"

#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <sys/time.h>

// Non-member function declarations
bool checkIfNodesAreToCloseToObstacles(aStarNode* nodes, int nrOfNodes, OpenGLvertex* obstacleSquaresOnDevice, int nrOfObstacles, float minDistanceToObstacle, float obstacleMatrixResolution);

// Hashtable:
int PathPlanning::HashTable::getIndex(float x, float y) {
	// I tested this hasher in matlab, it is based on Java's hashCode(), and it gives pretty even results with obstacle matrix resolution
	// between 0.01 and 0.2
	int hash =100*abs(x)+100*abs(y)*abs(y);
	return hash % HASH_TABLE_ENTRIES;
}

aStarNode* PathPlanning::HashTable::getAstarNode(float x, float y) {
	int arrayIndex = getIndex(x,y);
	HashBucket* bucketPointer = *(hashArray + arrayIndex);

	while(true) {
		if (bucketPointer==NULL) {break;} //No bucket was found matching x and y
		else if (bucketPointer->node.x==x && bucketPointer->node.y==y) {return &bucketPointer->node;}

		bucketPointer=bucketPointer->nextBucket;
	}
	return NULL;
}

PathPlanning::HashTable::HashTable() {
}

PathPlanning::HashTable::~HashTable() {
	clearHashTable();
}

void PathPlanning::HashTable::clearBucketList(HashBucket* bucket) {
	if (bucket==NULL) {return;}

	clearBucketList(bucket->nextBucket);
	delete bucket;
}

void PathPlanning::HashTable::clearHashTable() {
	// This function deletes the linked list entries of hashbuckets for each index (if any) that were allocated with new
	for (int i=0;i<HASH_TABLE_ENTRIES;i++) {clearBucketList(*(hashArray+i));}

	// Make all pointers in array null
	memset(hashArray,0,HASH_TABLE_ENTRIES*sizeof(HashBucket*));
}


aStarNode* PathPlanning::HashTable::addAstarNode(float x, float y) {
	aStarNode* addedNode = getAstarNode(x,y);
	if (addedNode != NULL) {return addedNode;}

	int arrayIndex = getIndex(x,y); // The index in the hashArray where the node is supposed to go

	// Get the latest bucket in the linked list for the specified index:
	HashBucket *bucketPointer = *(hashArray + arrayIndex), *prevBucket=NULL;
	while (bucketPointer!=NULL) {
		prevBucket = bucketPointer;
		bucketPointer = bucketPointer->nextBucket;
	}

	// Add bucket:
	bucketPointer=new HashBucket(); //Everything (including all the fields of the aStarNode) set to zero
	bucketPointer->node.x=x;
	bucketPointer->node.y=y;
	if (prevBucket!=NULL) {prevBucket->nextBucket=bucketPointer;}

	return &bucketPointer->node;
}

//Minheap:
PathPlanning::MinHeap::MinHeap() {
	currentNrOfNodesInHeap=0;
}

PathPlanning::MinHeap::~MinHeap() {
}

void PathPlanning::MinHeap::clearMinHeap() {
	currentNrOfNodesInHeap=0;
}

bool PathPlanning::MinHeap::addNode(aStarNode* node) {
	if (currentNrOfNodesInHeap==HEAP_SIZE) {return false;} // Return false if heap is full; meaning that no path can be generated

	currentNrOfNodesInHeap++;
	heapArray[currentNrOfNodesInHeap] = node;
	node->heapArrayIndex = currentNrOfNodesInHeap;

	bubbleNode(node);

	return true;
}

void PathPlanning::MinHeap::bubbleNode(aStarNode* node) {
	// Bubble up, see https://en.wikipedia.org/wiki/Binary_heap
	while (true) {
		// If parents heuristic is lower than mine, or if i already have the lowest, then i don't need to bubble
		if (node->heapArrayIndex==0 || heapArray[(node->heapArrayIndex-1)/2]->heuristic < heapArray[node->heapArrayIndex]->heuristic) {return;}

		// Switch position with parent
		heapArray[node->heapArrayIndex] = heapArray[(node->heapArrayIndex-1)/2]; // Put parent in my position in the heapArray
		heapArray[(node->heapArrayIndex-1)/2]=node; // Put me in my parents position
		node->heapArrayIndex = (node->heapArrayIndex-1)/2; // Update my heapArrayIndex
	}
}

aStarNode* PathPlanning::MinHeap::popNode() {
	if (currentNrOfNodesInHeap==0) {return NULL;}
	aStarNode *returnNode = heapArray[0], *tempNode;

	heapArray[0] = heapArray[currentNrOfNodesInHeap];
	heapArray[currentNrOfNodesInHeap]->heapArrayIndex=0;
	currentNrOfNodesInHeap--;

	int currentNodeIndex=0,childNodeIndex;
	double childHeuristics;
	while (true) {
		// Find the smallest of the two children and swap the current node with them if their heuristics are smaller than the current node
		if (2*currentNodeIndex +1 >= currentNrOfNodesInHeap) {break;} // currentNode has no children

		childNodeIndex = 2*currentNodeIndex+1;
		childHeuristics = heapArray[childNodeIndex]->heuristic;
		if (2*currentNodeIndex+2 < currentNrOfNodesInHeap && heapArray[2*currentNodeIndex+2]->heuristic < childHeuristics) {
			childNodeIndex=2*currentNodeIndex+2;
			childHeuristics=heapArray[2*currentNodeIndex+2]->heuristic;
		}
		if (heapArray[currentNodeIndex]->heuristic < childHeuristics) {break;} // No need to switch anything

		// Switch heapArray[currentNodeIndex] with heapArray[childNodeIndex]
		tempNode = heapArray[currentNodeIndex];
		heapArray[currentNodeIndex] = heapArray[childNodeIndex];
		heapArray[currentNodeIndex]->heapArrayIndex = currentNodeIndex;

		heapArray[childNodeIndex] = tempNode;
		heapArray[childNodeIndex]->heapArrayIndex = childNodeIndex;

		currentNodeIndex = childNodeIndex;
	}

	return returnNode;
}

// PathPlanning:
void PathPlanning::generatePath(float targetX, float targetY, OpenGLvertex* obstacleSquaresOnDevice, int nrOfObstacles, float minDistanceToObstacle, float obstacleMatrixResolution) {
	/* This function generates a path from {0,0} to targetX,targetY using a variant of A* tailored for producing vehicle paths
	 *
	 */
	hashTable->clearHashTable();
	minHeap->clearMinHeap();

	aStarNode *baseNode = hashTable->addAstarNode(0,0), *targetNode = hashTable->addAstarNode(targetX,targetY);
	while (true) {
		baseNode->isOnClosedSet=true;
		baseNode->isOnOpenSet=false;

		if (baseNode->x == targetNode->x && baseNode->y == targetNode->y) {return;} // Fix later

		for (int i=0;i<14;i++) {discoverNeighbor(baseNode,targetNode,i,obstacleSquaresOnDevice,nrOfObstacles,minDistanceToObstacle,obstacleMatrixResolution);}
		baseNode = minHeap->popNode();
	}
}

void PathPlanning::discoverNeighbor(aStarNode* baseNode, aStarNode* targetNode, int index, OpenGLvertex* obstacleSquaresOnDevice, int nrOfObstacles, float minDistanceToObstacle, float obstacleMatrixResolution) {
	// This function discovers a neighbor node to baseNode, looking in 8 directions forward, and 8 directions backward
	aStarNode* neighborNode;

	const float stepDistance=obstacleMatrixResolution; // The stepping distance between each point in the path
	double deltaAngle,directionAngle;
	switch(index) {
		// Going forward:
		case 0: {deltaAngle=0+(baseNode->pathIsReversingFromPrevNode ? M_PI : 0);break;}
		case 1: {deltaAngle=0.1+(baseNode->pathIsReversingFromPrevNode ? M_PI : 0);break;}
		case 2: {deltaAngle=0.2+(baseNode->pathIsReversingFromPrevNode ? M_PI : 0);break;}
		case 3: {deltaAngle=0.4+(baseNode->pathIsReversingFromPrevNode ? M_PI : 0);break;}
		case 4: {deltaAngle=-0.1+(baseNode->pathIsReversingFromPrevNode ? M_PI : 0);break;}
		case 5: {deltaAngle=-0.2+(baseNode->pathIsReversingFromPrevNode ? M_PI : 0);break;}
		case 6: {deltaAngle=-0.4+(baseNode->pathIsReversingFromPrevNode ? M_PI : 0);break;}
		// Reversing:
		case 7: {deltaAngle=0+(baseNode->pathIsReversingFromPrevNode ? 0 : M_PI);break;}
		case 8: {deltaAngle=0.1+(baseNode->pathIsReversingFromPrevNode ? 0 : M_PI);break;}
		case 9: {deltaAngle=0.2+(baseNode->pathIsReversingFromPrevNode ? 0 : M_PI);break;}
		case 10: {deltaAngle=0.4+(baseNode->pathIsReversingFromPrevNode ? 0 : M_PI);break;}
		case 11: {deltaAngle=-0.1+(baseNode->pathIsReversingFromPrevNode ? 0 : M_PI);break;}
		case 12: {deltaAngle=-0.2+(baseNode->pathIsReversingFromPrevNode ? 0 : M_PI);break;}
		case 13: {deltaAngle=-0.4+(baseNode->pathIsReversingFromPrevNode ? 0 : M_PI);break;}
	}
	directionAngle = baseNode->angleFromPreviosNodeInPath + deltaAngle;
	if (directionAngle < -M_PI) {directionAngle+=2*M_PI;}
	if (directionAngle > M_PI) {directionAngle-=2*M_PI;}

	// The coordinates of the neighbor node, the values rounded to match the obstacleMatrixResolution in the grid:
	float neighborX,neighborY;
	neighborX = baseNode->x + round(stepDistance * cos(directionAngle)/obstacleMatrixResolution)*obstacleMatrixResolution;
	neighborY = baseNode->y + round(stepDistance * sin(directionAngle)/obstacleMatrixResolution)*obstacleMatrixResolution;

	neighborNode = hashTable->addAstarNode(neighborX,neighborY); // Gets the neighborNode from the hashTable, or adds it if it hasn't been visited

	if (!neighborNode->isOnOpenSet && !neighborNode->isOnClosedSet) {
		// If this is true, the node has not been discovered yet, so before doing anything else, check if the node is too
		// close to any obstacle, and if so mark it as closed
		neighborNode->isOnClosedSet = checkIfNodesAreToCloseToObstacles(neighborNode,1,obstacleSquaresOnDevice,nrOfObstacles,minDistanceToObstacle,obstacleMatrixResolution);
	}
	if (neighborNode->isOnClosedSet) {return;}	// neighborNode is either on closed set, or to close to an obstacle, so ignore it

	// Calculate the new distance from start node and heuristics:
	float neighborDistanceFromStartNode;
	neighborDistanceFromStartNode = sqrt((baseNode->x-neighborNode->x)*(baseNode->x-neighborNode->x)+(baseNode->y-neighborNode->y)*(baseNode->y-neighborNode->y));
	if (index>6) {neighborDistanceFromStartNode *= 1.3;} //Increase the distance if the vehicle is reversing (punish reversing). A lot of modifications can be done here
	neighborDistanceFromStartNode += baseNode->distanceFromStartNode;

	float neighbourHeuristic;
	neighbourHeuristic = neighborDistanceFromStartNode;
	neighbourHeuristic += sqrt((neighborNode->x-targetNode->x)*(neighborNode->x-targetNode->x)+(neighborNode->y-targetNode->y)*(neighborNode->y-targetNode->y));

	if (!neighborNode->isOnOpenSet) {
		// Node has not been discovered yet
		neighborNode->isOnOpenSet=true;
		neighborNode->distanceFromStartNode =neighborDistanceFromStartNode;
		neighborNode->heuristic = neighbourHeuristic;
		neighborNode->angleFromPreviosNodeInPath = directionAngle;
		neighborNode->previousNodeInPath = baseNode;
		if (index<7) {neighborNode->pathIsReversingFromPrevNode=false;}
		else {neighborNode->pathIsReversingFromPrevNode=true;}

		minHeap->addNode(neighborNode);
	}
	else if (neighbourHeuristic < neighborNode->heuristic) {
		neighborNode->distanceFromStartNode = neighborDistanceFromStartNode;
		neighborNode->heuristic = neighbourHeuristic;
		neighborNode->angleFromPreviosNodeInPath = directionAngle;
		neighborNode->previousNodeInPath = baseNode;
		if (index<7) {neighborNode->pathIsReversingFromPrevNode=false;}
		else {neighborNode->pathIsReversingFromPrevNode=true;}

		minHeap->bubbleNode(neighborNode);
	}
}


PathPlanning::PathPlanning(VehicleState* vehicleState, Path* mainGPSpath) {
	this->vehicleState=vehicleState;
	this->mainGPSpath=mainGPSpath;

	this->hashTable = new HashTable();
	this->minHeap = new MinHeap();
	this->intraGPSpath = new Path();
}

PathPlanning::~PathPlanning() {
	delete hashTable;
	delete minHeap;
	delete intraGPSpath;
}


void PathPlanning::updatePathAndControlSignals(OpenGLvertex* obstacleSquares,int nrOfObstacles) {
	/* This function does the following:
	 *
	 * -Updates the pathInLocalCoords based on the new vehicle GPS position
	 * -Checks if the vehicle has passed the second GPS point in the path, and if then trims the path
	 * -Checks if any new obstacles are detected along the path, and if then replans the path
	 * -Sends the new control signals onto the CAN bus
	 */

	int nrOfPointsInPath=4;

	// Update pathInLocalCoords based on the new GPS data
	for (int i=0;i<nrOfPointsInPath;i++) {
		/* First calculate the difference (in meters) between vehicleState->currentGPSPosition and getGPSpointInPath(i) in
		 * lat and long. Call delta_lat y and delta_long x in a local coordinate system. Then remember that the heading of a
		 * vehicle is defined as the angle from true north to the longitudinal centerline of the vehicle. The task is now
		 * to rotate the point (x,y) clockwise by some angle so that the local x-axis now point in the same direction as the
		 * vehicle longitudinal centerline. This angle is vehicleState->currentHeading-M_PI/2. To understand why M_PI/2 is subtracted
		 * remember that a heading of 0 means the vehicle is traveling north. The local y axis will then align with the vehicle
		 * longitudinal centerline. We want the x axis to align, and that is why we have to rotate the point with -M_PI/2 clockwise
		 * The path will later be displayed overlaying the lidar data, so it is important that vehicleState->currentGPSposition
		 * gives the exact location of the lidar sensor origin (x=0,y=0,z=0).
		 */

		//(pathInLocalCoords + i)->x = rotateAndGetX(getLongDistanceBetweenPoints(vehicleState->currentGPSposition,getGPSpointInPath(i)),getLatDistanceBetweenPoints(vehicleState->currentGPSposition,getGPSpointInPath(i)),-(vehicleState->currentHeading-M_PI/2));
		//(pathInLocalCoords + i)->y = rotateAndGetY(getLongDistanceBetweenPoints(vehicleState->currentGPSposition,getGPSpointInPath(i)),getLatDistanceBetweenPoints(vehicleState->currentGPSposition,getGPSpointInPath(i)),-(vehicleState->currentHeading-M_PI/2));
	}




//	double currentPathAngle = getAngleBetweenPoints(getFirstPointInPath(),getSecondPointInPath());
//	float
//
//	if () {
//		currentPathIndex++;
//		if (currentPathIndex==pointsInPath) {currentPathIndex=0;}
//	}
}




// Non member functions:
__global__ void checkIfNodeIsToCloseToObstacle(aStarNode* nodesOnDevice, int nrOfNodes, OpenGLvertex* obstacleSquaresOnDevice, int nrOfObstacles, float minDistanceToObstacle, float obstacleMatrixResolution, bool* isToClosePointer) {
	/* Think of the function as a matrix, where nodes are the rows, and obstacles are the columns. For each node/obstacle pair, see if
	 * their distance is at least the min distance. To get the row and column index, row = index/columnSize, column = index - row*columnSize
	 */
	int myThreadID = blockIdx.x*blockDim.x+threadIdx.x, myNodeIndex = myThreadID/nrOfObstacles, myObstacleIndex = myThreadID-myNodeIndex*nrOfObstacles;
	float distance,nodeX,nodeY,obstacleX,obstacleY;

	if (myNodeIndex < nrOfNodes) {
		// The positions of the nodes and obstacles are the center distances of their respective square
		nodeX = (nodesOnDevice+myNodeIndex)->x + obstacleMatrixResolution/2;
		nodeY = (nodesOnDevice+myNodeIndex)->y + obstacleMatrixResolution/2;
		obstacleX = (obstacleSquaresOnDevice + 4*myObstacleIndex)->x +obstacleMatrixResolution/2;
		obstacleY = (obstacleSquaresOnDevice + 4*myObstacleIndex)->y +obstacleMatrixResolution/2;

		distance = sqrt((nodeX-obstacleX)*(nodeX-obstacleX)+(nodeY-obstacleY)*(nodeY-obstacleY));

		if (distance < minDistanceToObstacle) {*(isToClosePointer)=true;}
	}
}

bool checkIfNodesAreToCloseToObstacles(aStarNode* nodes, int nrOfNodes, OpenGLvertex* obstacleSquaresOnDevice, int nrOfObstacles, float minDistanceToObstacle, float obstacleMatrixResolution) {
	// Returns false if all nodes in nodes have a min distance of minDistance to all obstacles in obstacleSquaresOnDevice, otherwise returns true
	bool* isToClosePointer;
	CUDA_CHECK_RETURN(cudaMalloc((void**)&isToClosePointer,sizeof(bool)));
	CUDA_CHECK_RETURN(cudaMemset(isToClosePointer,0,sizeof(bool)));

	aStarNode* nodesOnDevice;
	CUDA_CHECK_RETURN(cudaMalloc((void**)&nodesOnDevice,sizeof(aStarNode)*nrOfNodes));

	int numberOfKernelBlocks = nrOfNodes*nrOfObstacles/256+1;
	checkIfNodeIsToCloseToObstacle<<<numberOfKernelBlocks,256>>>(nodesOnDevice,nrOfNodes,obstacleSquaresOnDevice,nrOfObstacles,minDistanceToObstacle,obstacleMatrixResolution,isToClosePointer);

	bool isToClose;
	CUDA_CHECK_RETURN(cudaMemcpy(&isToClose,isToClosePointer,sizeof(bool),cudaMemcpyDeviceToHost));
	CUDA_CHECK_RETURN(cudaFree(isToClosePointer));
	CUDA_CHECK_RETURN(cudaFree(nodesOnDevice));
	cudaDeviceSynchronize(); // TODO change to specific instead of global stream
	return isToClose;
}


//double getAngleBetweenPoints(OpenGLvertex* fromPoint,OpenGLvertex* toPoint) {
//	double dx = toPoint->x - fromPoint->x;
//	double dy = toPoint->y - fromPoint->y;
//	return atan2(dx,dy);
//}

float rotateAndGetX(float x, float y, double angle) {
	// Rotates the point by the angle counter clockwise and returns its new x coordinate
	return x * cos(angle) - y * sin(angle);
}

float rotateAndGetY(float x, float y, double angle) {
	// Rotates the point by the angle counter clockwise and returns its new y coordinate
	return x * sin(angle) + y * cos(angle);
}

float getLongDistanceBetweenPoints(GPSpoint* fromPoint, GPSpoint* toPoint) {
	// Returns the longitudinal distance in meters from fromPoint to toPoint
	int lengthOfDegreeLongInMeters = 57141;
	return lengthOfDegreeLongInMeters*(toPoint->longc - fromPoint->longc);
}

float getLatDistanceBetweenPoints(GPSpoint* fromPoint, GPSpoint* toPoint) {
	// Returns the latitudinal distance in meters from fromPoint to toPoint
	int lengthOfDegreeLatInMeters = 111398;
	return lengthOfDegreeLatInMeters*(toPoint->latc - fromPoint->latc);
}

//GPSpoint* PathPlanning::getGPSpointInPath(int index) {
//	int arrayIndex = index + currentPathIndex;
//	if (arrayIndex>=nrOfPointsInPath) {arrayIndex-=nrOfPointsInPath;}
//
//	return pathInGPScoords + arrayIndex;
//}

//aStarNodeListEntry* PathPlanning::getNeighborList(aStarNode* baseNode, float obstacleMatrixResolution) {
//	/*
//	 *
//	 *
//	 */
//
//	aStarNodeListEntry* neighborList = new aStarNodeListEntry();
//	neighborList->node = getNeighbor(baseNode,0,obstacleMatrixResolution);
//	for (int i=1;i<14;i++) {
//		neighborList->nextListEntry = new aStarNodeListEntry();
//		neighborList->nextListEntry->node = getNeighbor(baseNode,i,obstacleMatrixResolution);
//	}
//
//
//	return neighborList;
//}
//
//void PathPlanning::clearNeighborsList(aStarNodeListEntry* listEntry) {
//	if (listEntry->nextListEntry==NULL) {
//		delete listEntry;
//		return;
//	}
//
//	clearNeighborsList(listEntry->nextListEntry);
//	delete listEntry;
//}
