#include "Headers/PathPlanning.h"
#include "Headers/CudaErrorCheckFunctions.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h> // For memset
#include <errno.h>

#define LENGTH_OF_ONE_LONG_DEGREE_IN_METERS 57141
#define LENGTH_OF_ONE_LAT_DEGREE_IN_METERS 111398

namespace {
	// Non-member function declarations
	bool checkIfNodesAreToCloseToObstacles(const aStarNode* nodes, int nrOfNodes, const ObstaclePoint *obstacleSquaresOnGPU, int nrOfObstacles, float minDistanceToObstacle, float obstacleMatrixResolution);
	float rotateAndGetX(float x, float y, double angle);
	float rotateAndGetY(float x, float y, double angle);
}

// PathPlanning:
PathPlanning::PathPlanning(const VehicleState &vehicleState, const ObstaclePoint* obstacleSquaresOnGPU, const int &currentNrOfObstacles):
		vehicleState(vehicleState),
		obstacleSquaresOnGPU(obstacleSquaresOnGPU),
		currentNrOfObstacles(currentNrOfObstacles) {

	this->hashTable=new HashTable();
	this->minHeap=new MinHeap();

	this->macroPathGPS=NULL;
	this->microPathGPS=NULL;
	this->microPathXY=NULL;
	this->lengthOfMacroPath=0;
	this->lengthOfMicroPath=0;
	this->currentIndexInMacroPath=0;
	this->currentIndexInMicroPath=0;
}

PathPlanning::~PathPlanning() {
	delete hashTable;
	delete minHeap;
	delete[] macroPathGPS;
	delete[] microPathGPS;
	delete[] microPathXY;
}

void PathPlanning::updatePathAndControlSignals() const {
	// This is the main path-updating function that gets called once every iteration of the main control loop
	if (currentIndexInMacroPath == lengthOfMacroPath-1 || lengthOfMacroPath==0) {return;} // Vehicle has arrived on target

	// First translate the path from GPS coords to XY coords
	for (int i=currentIndexInMicroPath;i<lengthOfMicroPath;i++) {
		translateGPSpositionToXY(microPathGPS->latc,microPathGPS->longc,microPathXY[i]);
	}

	double currentPathAngle;
	float pathPointRotatedY,vehicleRotatedY;
	// The update currentIndexInMicroPath
	while(true) {
		//currentPathAngle=atan2(microPathXY[currentIndexInMicroPath].x
	}
}

bool PathPlanning::setMacroPath(const char* filePath) {
	// Clear old path first:
	delete[] macroPathGPS;
	currentIndexInMacroPath=0;
	lengthOfMacroPath=0;

	// This function is called by Input when a new path is loaded. The filepath to the pathfile is the parameter
	FILE *fp;
	char line[60];

	if ((fp = fopen(filePath,"rt"))  == NULL) {
		printf("%s%s\n","Unable to open path file: ", strerror(errno));
		return false;
	}

	while (fgets(line, 60, fp)) {if (*line!=35) {break;}} // Comment line, starting with '#'
	sscanf(line,"%d",&lengthOfMacroPath);

	macroPathGPS = new GPSposition[lengthOfMacroPath]();

	for (int i=0;i<lengthOfMacroPath;i++) {
		if (fgets(line,60,fp)==0) {
			printf("%s%s\n","Read error when reading from path file: ",strerror(errno));
			delete[] macroPathGPS;
			macroPathGPS=NULL;
			return false;
		}
		sscanf(line,"%lf %lf\n",&((macroPathGPS+i)->latc),&((macroPathGPS+i)->longc));
	}

	fclose(fp);
	return true;
}

bool PathPlanning::generateMicroPath(float targetX, float targetY, const ObstaclePoint* obstacleSquaresOnDevice, int nrOfObstacles, float minDistanceToObstacle, float obstacleMatrixResolution) {
	/* This function generates a path from {0,0} to targetX,targetY using a variant of A* tailored for producing vehicle paths
	 * returns true if a path was found, and false otherwise
	 */

	// First delete the current microGPSpath:
	delete[] microPathGPS;
	delete[] microPathXY;
	currentIndexInMicroPath=0;
	lengthOfMicroPath=0;
	hashTable->clearHashTable();
	minHeap->clearMinHeap();

	aStarNode *baseNode = hashTable->addAstarNode(0,0), *targetNode = hashTable->addAstarNode(targetX,targetY);
	while (true) {
		baseNode->isOnClosedSet=true;
		baseNode->isOnOpenSet=false;

		if (baseNode->x == targetNode->x && baseNode->y == targetNode->y) {break;} // Path found

		for (int i=0;i<14;i++) {
			if(!discoverNeighbor(baseNode,targetNode,i,obstacleSquaresOnDevice,nrOfObstacles,minDistanceToObstacle,obstacleMatrixResolution)) {
				return false; // Means that the minHeap is full, and cannot accept any more nodes
			}
		}
		baseNode = minHeap->popNode();
		if (baseNode==NULL) {return false;} // No path could be found
	}

	// Path found, now init microPathGPS
	const aStarNode* currentNode = targetNode;
	while((currentNode = currentNode->previousNodeInPath)!=NULL) {lengthOfMicroPath++;} // Find number of nodes in the path

	microPathGPS = new GPSposition[lengthOfMicroPath]();
	microPathXY = new PathPoint[lengthOfMicroPath]();

	currentNode = targetNode;
	for (int i=lengthOfMicroPath-1;i>=0;i--) {
		translateXYtoGPSposition(currentNode->x,currentNode->y,microPathGPS[i]);
		currentNode=currentNode->previousNodeInPath;
	}

	return true;
}

bool PathPlanning::discoverNeighbor(const aStarNode *baseNode, const aStarNode *targetNode, int index, const ObstaclePoint* obstacleSquaresOnDevice, int nrOfObstacles, float minDistanceToObstacle, float obstacleMatrixResolution) const {
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
	if (neighborNode->isOnClosedSet) {return true;}	// neighborNode is either on closed set, or to close to an obstacle, so ignore it

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

		if(!minHeap->addNode(neighborNode)) {return false;} // Minheap is full, path cannot be found
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
	return true;
}

void PathPlanning::translateXYtoGPSposition(float x, float y, GPSposition& target) const {
	/* The vehicle has the y axis pointing forward in the direction of travel. Rotate a point x
	 * and y counter clockwise by heading makes y align with true north, and x align with true east.
	 * Scale those and you get latc and longc
	 */
	float rotatedX = rotateAndGetX(x,y,vehicleState.currentHeading);
	float rotatedY = rotateAndGetY(x,y,vehicleState.currentHeading);

	target.latc = vehicleState.currentPosition.latc + rotatedY * (1.0/LENGTH_OF_ONE_LAT_DEGREE_IN_METERS);
	target.longc = vehicleState.currentPosition.longc + rotatedX * (1.0/LENGTH_OF_ONE_LONG_DEGREE_IN_METERS);
}

void PathPlanning::translateGPSpositionToXY(double latc, double longc, PathPoint& target) const {
	// See comment in above function
	float scaledY = (vehicleState.currentPosition.latc - latc) * LENGTH_OF_ONE_LAT_DEGREE_IN_METERS;
	float scaledX = (vehicleState.currentPosition.longc - longc) * LENGTH_OF_ONE_LONG_DEGREE_IN_METERS;

	target.x = rotateAndGetX(scaledX,scaledY,-vehicleState.currentHeading);
	target.y = rotateAndGetY(scaledX,scaledY,-vehicleState.currentHeading);
}

// Hashtable:
PathPlanning::HashTable::HashTable() {
}

PathPlanning::HashTable::~HashTable() {
	clearHashTable();
}

int PathPlanning::HashTable::getIndex(float x, float y) const {
	// I tested this hasher in matlab, it is based on Java's hashCode(), and it gives pretty even results with obstacle matrix resolution
	// between 0.01 and 0.2
	int hash =100*abs(x)+100*abs(y)*abs(y);
	return hash % HASH_TABLE_ENTRIES;
}

aStarNode* PathPlanning::HashTable::getAstarNode(float x, float y) const {
	int arrayIndex = getIndex(x,y);
	HashBucket* bucketPointer = *(hashArray + arrayIndex);

	while(true) {
		if (bucketPointer==NULL) {break;} //No bucket was found matching x and y
		else if (bucketPointer->node.x==x && bucketPointer->node.y==y) {return &bucketPointer->node;}

		bucketPointer=bucketPointer->nextBucket;
	}
	return NULL;
}

void PathPlanning::HashTable::clearBucketList(HashBucket* bucket) const {
	// Deletes every entry in the bucket linked-list
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


aStarNode *PathPlanning::HashTable::addAstarNode(float x, float y) {
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

namespace {
// Non member functions implementations:
	__global__ void checkIfNodeIsToCloseToObstacle(const aStarNode* nodesOnDevice, int nrOfNodes, const ObstaclePoint* obstacleSquaresOnDevice, int nrOfObstacles, float minDistanceToObstacle, float obstacleMatrixResolution, bool* isToClosePointer) {
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

	bool checkIfNodesAreToCloseToObstacles(const aStarNode* nodes, int nrOfNodes, const ObstaclePoint *obstacleSquaresOnDevice, int nrOfObstacles, float minDistanceToObstacle, float obstacleMatrixResolution) {
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

	float rotateAndGetX(float x, float y, double angle) {
		// Rotates the point by the angle counter clockwise and returns its new x coordinate
		return x * cos(angle) - y * sin(angle);
	}

	float rotateAndGetY(float x, float y, double angle) {
		// Rotates the point by the angle counter clockwise and returns its new y coordinate
		return x * sin(angle) + y * cos(angle);
	}
}


