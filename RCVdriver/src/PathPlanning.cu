#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h> // For memset
#include <errno.h>

#include "Headers/Parameters.h"
#include "Headers/PathPlanning.h"
#include "Headers/CudaErrorCheckFunctions.h"

#define LENGTH_OF_ONE_LONG_DEGREE_IN_METERS 57141
#define LENGTH_OF_ONE_LAT_DEGREE_IN_METERS 111398

namespace {
	// Non-member function declarations
	bool checkIfPathIsToCloseToObstacles(PathPointInLocalXY* path, int lengthOfPath, const ObstaclePoint *obstacleSquaresOnGPU, int nrOfObstacles, double currentVehicleHeading);
	bool checkIfaStarNodeIsTooCloseToObstacles(float nodeXpos, float nodeYpos, double localPathAngleFromPreviousNode, const ObstaclePoint* obstacleSquaresOnGPU, int nrOfObstacles);
	float rotateAndGetX(float x, float y, double angle);
	float rotateAndGetY(float x, float y, double angle);
	float getLatDistanceBetweenGPSpositions(GPSposition fromPosition, GPSposition toPosition);
	float getLongDistanceBetweenGPSpositions(GPSposition fromPosition, GPSposition toPosition);
	double getHeadingBetweenGPSpositions(GPSposition fromPosition, GPSposition toPosition);
}

// PathPlanning:
PathPlanning::PathPlanning(const VehicleState &vehicleState, const ObstaclePoint* obstacleSquaresOnGPU, const int &currentNrOfObstacles,VehicleStatus& vehicleStatus):
		vehicleState(vehicleState),
		obstacleSquaresOnGPU(obstacleSquaresOnGPU),
		currentNrOfObstacles(currentNrOfObstacles),
		vehicleStatus(vehicleStatus) {

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

void PathPlanning::updatePathAndControlSignals() {
	/* This is the main path-updating function that gets called once every iteration of the main control loop
	 * There are two main paths, macroPath and microPath. MacroPath is defined by the user when he types
	 * the command loadpath in the input. MicroPath is the path between the points in the macroPath. MicroPath
	 * is the path the vehicle will follow and adjusting its control signals to. MicroPath will be replanned
	 * if an obstacle is detected along its path
	 */
	if (lengthOfMacroPath==0) { // MacroPath has not been set, so return
		vehicleStatus.macroPathHasBeenSet=false;
		vehicleStatus.isRunning=false;
		return;
	}
	else if (lengthOfMicroPath==0) {generateMicroPath(macroPathGPS[currentIndexInMacroPath].position);} // MicroPath has not been set, so generate one

	// Update the path indexes, and return if the macroPath has been traversed
	if (updatePathIndexes()) {return;}

	// Now translate the microPath from GPS coords to XY coords
	for (int i=currentIndexInMicroPath;i<lengthOfMicroPath;i++) {
		translateGPSpositionToLocalXY(microPathGPS[i].position.latc,microPathGPS[i].position.longc,microPathXY[i].x,microPathXY[i].y);
	}

}

bool PathPlanning::updatePathIndexes() {
	// This function updates currentIndexInMicroPath and currentIndexInMacroPath based on the current vehicle position

	double currentPathAngle;
	float vehicleDeltaLatFromPrevGPSpoint,vehicleDeltaLongFromPrevGPSpoint,gpsPointDeltaLatFromPrevGPSpoint,gpsPointDeltaLongFromPrevGPSpoint,vehicleRotatedX,gpsPointRotatedX;
	while(true) {
		if (currentIndexInMicroPath==lengthOfMicroPath) {
			/* If currentIndexInMicroPath==lengthOfMicroPath, then the vehicle is trying to head towards a position in
			 * microPath that is outside the microPath array, which means that vehicle has traversed the microPath, and
			 * arrived to the macroPath position that the microPath was aiming for
			 */
			currentIndexInMacroPath++;
			while(true) {
				if (currentIndexInMacroPath==lengthOfMacroPath) {
					// This indicates that the vehicle has traversed the macro path and arrived at its destination
					printf("%s\n","Vehicle has successfully traversed macroPath");
					clearAllPaths(true);
					return true;
				}
				/* Now the task is to see if the vehicle has passed macroPathGPS[currentIndexInMacroPath]. To do that, RCVdriver
				 * uses macroPathGPS[currentIndexInMacroPath-1] as a reference position, and calculates the delta distance in meters
				 * from that position to the vehicle, and from that position to macroGPSpath[currentIndexInMacroPath]. After that, those
				 * two points, {vehicleDeltaNorthFromPrevGPSpoint,vehicleDeltaEastFromPrevGPSpoint} and {gpsPointDeltaNorthFromPrevGPSpoint,
				 * gpsPointDeltaEastFromPrevGPSpoint} are rotated counter clockwise by the heading (radians from true north) given by
				 * the path from macroPathGPS[currentIndexInMacroPath-1] to macroPathGPS[currentIndexInMacroPath]. RCV driver then
				 * compares the x coordinates of those rotated points, and see if the vehicle's x coordinate is greater than the next GSPpoint's
				 * x coordinate. If that is the case, the vehicle has passed that GPSpoint
				 *
				 * Note that at this point in the code both currentIndexInMacroPath and lengthOfMacroPath is at least 1, so there will be
				 * no buffer overflows reading from macroGPSpath
				 */
				vehicleDeltaLatFromPrevGPSpoint = getLatDistanceBetweenGPSpositions(macroPathGPS[currentIndexInMacroPath-1].position,vehicleState.currentPosition);
				vehicleDeltaLongFromPrevGPSpoint = getLongDistanceBetweenGPSpositions(macroPathGPS[currentIndexInMacroPath-1].position,vehicleState.currentPosition);
				gpsPointDeltaLatFromPrevGPSpoint = macroPathGPS[currentIndexInMacroPath].latDistanceFromPrevPathPoint;
				gpsPointDeltaLongFromPrevGPSpoint = macroPathGPS[currentIndexInMacroPath].longDistanceFromPrevPathPoint;
				currentPathAngle = macroPathGPS[currentIndexInMacroPath].headingFromPrevPathPoint;

				vehicleRotatedX = rotateAndGetX(vehicleDeltaLatFromPrevGPSpoint,vehicleDeltaLongFromPrevGPSpoint,currentPathAngle);
				gpsPointRotatedX = rotateAndGetX(gpsPointDeltaLatFromPrevGPSpoint,gpsPointDeltaLongFromPrevGPSpoint,currentPathAngle);

				if (vehicleRotatedX>gpsPointRotatedX) {currentIndexInMacroPath++;}
				else {break;} // If false, the currentIndexInMacroPath is still ahead of the vehicle, and should be the current target for the microPath
			}
			generateMicroPath(macroPathGPS[currentIndexInMacroPath].position);
		}

		/* Now do the same thing as described above but with the microGPSpath. Just like with macroGPSpath, currentIndexInMicroPath
		 * and lengthOfMicroPath is at least one at this point
		 */
		vehicleDeltaLatFromPrevGPSpoint = getLatDistanceBetweenGPSpositions(microPathGPS[currentIndexInMicroPath-1].position,vehicleState.currentPosition);
		vehicleDeltaLongFromPrevGPSpoint = getLongDistanceBetweenGPSpositions(microPathGPS[currentIndexInMicroPath-1].position,vehicleState.currentPosition);
		gpsPointDeltaLatFromPrevGPSpoint = microPathGPS[currentIndexInMicroPath].latDistanceFromPrevPathPoint;
		gpsPointDeltaLongFromPrevGPSpoint = microPathGPS[currentIndexInMicroPath].longDistanceFromPrevPathPoint;
		currentPathAngle = microPathGPS[currentIndexInMicroPath].headingFromPrevPathPoint;

		vehicleRotatedX = rotateAndGetX(vehicleDeltaLatFromPrevGPSpoint,vehicleDeltaLongFromPrevGPSpoint,currentPathAngle);
		gpsPointRotatedX = rotateAndGetX(gpsPointDeltaLatFromPrevGPSpoint,gpsPointDeltaLongFromPrevGPSpoint,currentPathAngle);

		if (vehicleRotatedX>gpsPointRotatedX) {currentIndexInMicroPath++;}
		else {break;}
	}

	return false;
}

void PathPlanning::clearAllPaths(bool includeMacroPath) {
	if (includeMacroPath) {
		delete[] macroPathGPS;
		macroPathGPS=NULL;
		lengthOfMacroPath=0;
		currentIndexInMacroPath=0;
	}
	delete[] microPathGPS;
	delete[] microPathXY;
	microPathGPS=NULL;
	microPathXY=NULL;
	lengthOfMicroPath=0;
	currentIndexInMicroPath=0;
}

bool PathPlanning::setMacroPath(const char* filePath) {
	// Reset paths first
	clearAllPaths(true);

	// This function is called by Input when a new path is loaded. The filepath to the pathfile is the parameter
	FILE *fp;
	char line[60];

	if ((fp = fopen(filePath,"rt"))  == NULL) {
		printf("%s%s\n","Unable to open path file: ", strerror(errno));
		return false;
	}

	while (fgets(line, 60, fp)) {if (*line!=35) {break;}} // Comment line, starting with '#'
	sscanf(line,"%d",&lengthOfMacroPath);
	if (lengthOfMacroPath<1) {
		printf("%s\n","Path must be of length greater than zero");
		return false;
	}

	lengthOfMacroPath++;
	macroPathGPS = new PathPointInGPScords[lengthOfMacroPath]();
	/* Add the vehicles own position as the first position in the macroGPSpath; the vehicle
	 * still has to travel from its initial position to the start position of the path
	 */
	macroPathGPS[0].position.latc = vehicleState.currentPosition.latc;
	macroPathGPS[0].position.longc = vehicleState.currentPosition.longc;

	for (int i=1;i<lengthOfMacroPath;i++) {
		if (fgets(line,60,fp)==0) {
			printf("%s\n","Not all coordinates in path are provided in path file");
			clearAllPaths(true);
			return false;
		}
		sscanf(line,"%lf %lf\n",&((macroPathGPS+i)->position.latc),&((macroPathGPS+i)->position.longc));
		macroPathGPS[i].headingFromPrevPathPoint = getHeadingBetweenGPSpositions(macroPathGPS[i-1].position,macroPathGPS[i].position);
		macroPathGPS[i].latDistanceFromPrevPathPoint = getLatDistanceBetweenGPSpositions(macroPathGPS[i-1].position,macroPathGPS[i].position);
		macroPathGPS[i].longDistanceFromPrevPathPoint = getLongDistanceBetweenGPSpositions(macroPathGPS[i-1].position,macroPathGPS[i].position);
	}

	currentIndexInMacroPath=1;
	/* When a new macroPath is generated its first GPScoordinate will be the vehicle's own position.
	 * CurrentIndexInMacroPath represents the index in macroGPSpath of the position the vehicle is heading
	 * towards. CurrentIndexInMacroPath-1 represents the position that the vehicle just left. The vehicle
	 * should start heading towards the position in the path that comes after its own position, and therefore
	 * currentIndexInMacroPath is set to 1
	 */

	fclose(fp);
	vehicleStatus.macroPathHasBeenSet=true;
	return true;
}

void PathPlanning::generateMicroPath(GPSposition targetPosition) {
	/* This function generates a path from {0,0} to targetX,targetY using a variant of A* tailored for producing vehicle paths
	 * returns true if a path was found, and false otherwise
	 */

	// First delete the current microGPSpath:
	clearAllPaths(false);

	hashTable->clearHashTable();
	minHeap->clearMinHeap();

	float targetX,targetY;
	translateGPSpositionToLocalXY(targetPosition.latc,targetPosition.longc,targetX,targetY);
	targetX = round(targetX/GROUND_GRID_RESOLUTION)*GROUND_GRID_RESOLUTION;
	targetY = round(targetY/GROUND_GRID_RESOLUTION)*GROUND_GRID_RESOLUTION;

	aStarNode *baseNode = hashTable->addAstarNode(0,0), *targetNode = hashTable->addAstarNode(targetX,targetY);
	while (true) {
		baseNode->isOnClosedSet=true;
		baseNode->isOnOpenSet=false;

		if (baseNode->x == targetNode->x && baseNode->y == targetNode->y) {break;} // Path found

		for (int i=0;i<14;i++) {
			if(!discoverNeighbor(baseNode,targetNode,i)) {
				vehicleStatus.isAbleToGenerateMicroPath=false;
				return; // Means that the minHeap is full, and cannot accept any more nodes
			}
		}
		baseNode = minHeap->popNode();
		if (baseNode==NULL) { // MinHeap is empty, so no path could be found
			vehicleStatus.isAbleToGenerateMicroPath=false;
			return;
		}
	}

	// Path found, now create microPathGPS and microPathXY
	const aStarNode* currentNode = targetNode;
	while((currentNode = currentNode->previousNodeInPath)!=NULL) {lengthOfMicroPath++;} // Find number of nodes in the path

	lengthOfMicroPath++; // Make room for the first position in the path; the vehicles own position
	microPathGPS = new PathPointInGPScords[lengthOfMicroPath]();
	microPathXY = new PathPointInLocalXY[lengthOfMicroPath]();

	currentNode = targetNode;
	for (int i=lengthOfMicroPath-1;i>=0;i--) {
		translateLocalXYtoGPSposition(currentNode->x,currentNode->y,microPathGPS[i].position);
		microPathGPS[i].headingFromPrevPathPoint = getHeadingBetweenGPSpositions(microPathGPS[i-1].position,microPathGPS[i].position);
		microPathGPS[i].latDistanceFromPrevPathPoint = getLatDistanceBetweenGPSpositions(microPathGPS[i-1].position,microPathGPS[i].position);
		microPathGPS[i].longDistanceFromPrevPathPoint = getLongDistanceBetweenGPSpositions(microPathGPS[i-1].position,microPathGPS[i].position);
		currentNode=currentNode->previousNodeInPath;
	}

	currentIndexInMicroPath=1;
	/* Just like in setMacroPath(), the first position in microPathGPS is the vehicles current position, so
	 * therefore set currentIndexInMicroPath to 1, in order to make the vehicle start heading for the position
	 * in microPath that comes after its own position
	 */

	vehicleStatus.isAbleToGenerateMicroPath=true;
	return;
}

bool PathPlanning::discoverNeighbor(const aStarNode *baseNode, const aStarNode *targetNode, int index) const {
	/* This function discovers a neighbor node to baseNode, looking in 8 directions forward, and 8 directions backward
	 * First time it is called (when baseNode==startNode), baseNode->angleFromPreviousNodeInPath = 0
	 */
	aStarNode* neighborNode;

	const float stepDistance=GROUND_GRID_RESOLUTION; // The stepping distance between each point in the path
	double deltaAngle,neighborLocalPathAngleFromPreviousNode;
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
	neighborLocalPathAngleFromPreviousNode = baseNode->localPathAngleFromPreviousNode + deltaAngle;
	if (neighborLocalPathAngleFromPreviousNode < -M_PI) {neighborLocalPathAngleFromPreviousNode+=2*M_PI;}
	if (neighborLocalPathAngleFromPreviousNode > M_PI) {neighborLocalPathAngleFromPreviousNode-=2*M_PI;}

	// The coordinates of the neighbor node, the values rounded to match the obstacleMatrixResolution in the grid:
	float neighborX,neighborY;
	neighborX = baseNode->x + round(stepDistance * cos(neighborLocalPathAngleFromPreviousNode)/GROUND_GRID_RESOLUTION)*GROUND_GRID_RESOLUTION;
	neighborY = baseNode->y + round(stepDistance * sin(neighborLocalPathAngleFromPreviousNode)/GROUND_GRID_RESOLUTION)*GROUND_GRID_RESOLUTION;

	neighborNode = hashTable->addAstarNode(neighborX,neighborY); // Gets the neighborNode from the hashTable, or adds it if it hasn't been visited

	if (neighborNode->isOnClosedSet || checkIfaStarNodeIsTooCloseToObstacles(neighborX,neighborY,neighborLocalPathAngleFromPreviousNode,obstacleSquaresOnGPU,currentNrOfObstacles)) {return true;}
	// If neigborNode is on closed set, or if it is too close to an obstacle, ignore it and return

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
		neighborNode->localPathAngleFromPreviousNode = neighborLocalPathAngleFromPreviousNode;
		neighborNode->previousNodeInPath = baseNode;
		if (index<7) {neighborNode->pathIsReversingFromPrevNode=false;}
		else {neighborNode->pathIsReversingFromPrevNode=true;}

		if(!minHeap->addNode(neighborNode)) {return false;} // Minheap is full, path cannot be found
	}
	else if (neighbourHeuristic < neighborNode->heuristic) {
		neighborNode->distanceFromStartNode = neighborDistanceFromStartNode;
		neighborNode->heuristic = neighbourHeuristic;
		neighborNode->localPathAngleFromPreviousNode = neighborLocalPathAngleFromPreviousNode;
		neighborNode->previousNodeInPath = baseNode;
		if (index<7) {neighborNode->pathIsReversingFromPrevNode=false;}
		else {neighborNode->pathIsReversingFromPrevNode=true;}

		minHeap->bubbleNode(neighborNode);
	}
	return true;
}

void PathPlanning::translateLocalXYtoGPSposition(float x, float y, GPSposition& target) const {
	/* The vehicle has the y axis pointing forward in the direction of travel. Rotate a point x
	 * and y counter clockwise by heading makes y align with true north, and x align with true east.
	 * Scale those and you get latc and longc
	 */
	float rotatedX = rotateAndGetX(x,y,vehicleState.currentHeading);
	float rotatedY = rotateAndGetY(x,y,vehicleState.currentHeading);

	target.latc = vehicleState.currentPosition.latc + rotatedY * (1.0/LENGTH_OF_ONE_LAT_DEGREE_IN_METERS);
	target.longc = vehicleState.currentPosition.longc + rotatedX * (1.0/LENGTH_OF_ONE_LONG_DEGREE_IN_METERS);
}

void PathPlanning::translateGPSpositionToLocalXY(double latc, double longc, float& x, float& y) const {
	// Translates latc and longc to vehicle local coordinate system and stores x and y in x and y
	float scaledY = (vehicleState.currentPosition.latc - latc) * LENGTH_OF_ONE_LAT_DEGREE_IN_METERS;
	float scaledX = (vehicleState.currentPosition.longc - longc) * LENGTH_OF_ONE_LONG_DEGREE_IN_METERS;

	x = rotateAndGetX(scaledX,scaledY,-vehicleState.currentHeading);
	y = rotateAndGetY(scaledX,scaledY,-vehicleState.currentHeading);
}

// Hashtable:
PathPlanning::HashTable::HashTable() {
}

PathPlanning::HashTable::~HashTable() {
	clearHashTable();
}

int PathPlanning::HashTable::getIndex(float x, float y) const {
	// I tested this hasher in matlab, it is based on Java's hashCode(), and it gives pretty even results with GROUND_GRID_RESOLUTION
	// between 0.01 and 0.2

	// x and y must always be aligned with the ground grid, however the discoverNeighbors function already does this before adding the nodes
	//x = round(x/GROUND_GRID_RESOLUTION)*GROUND_GRID_RESOLUTION;
	//y = round(y/GROUND_GRID_RESOLUTION)*GROUND_GRID_RESOLUTION;

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
// Non member and cuda functions implementations:
	__device__ bool checkIfPathPointIsTooCloseToObstacle(float obstacleX, float obstacleY, float pathPointX, float pathPointY, double localPathAngleFromPreviousNode) {
		// This function returns true if the obstacle is to close to the vehicle, otherwise returns false
		float rotatedObstacleX,rotatedObstacleY,minDistanceToObstacle;

		//TODO explanation of how this function works

		/* Observe that because that the x and y positions of the nodes and obstacles are aligned with the
		 * ground grid, they have an error margin of +/- GROUND_GRID_RESOLUTION. Therefore OBSTACLE_SAFETY_DISTANCE
		 * should never be less than GROUND_GRID_RESOLUTION
		 */
		obstacleX = obstacleX - pathPointX;
		obstacleY = obstacleY - pathPointY;
		rotatedObstacleX = abs(obstacleX * cos(localPathAngleFromPreviousNode) - obstacleY * sin(localPathAngleFromPreviousNode));
		rotatedObstacleY = abs(obstacleX * sin(localPathAngleFromPreviousNode) + obstacleY * cos(localPathAngleFromPreviousNode));

		if (rotatedObstacleX>RCV_WIDTH/2 && rotatedObstacleY>RCV_LENGTH/2) {
			// minDistanceToObstacle is defined by the distance from {RCV_WIDTH/2,RCV_LENGHT/2} to the obstacle point
			minDistanceToObstacle = sqrt((rotatedObstacleX-RCV_WIDTH/2)*(rotatedObstacleX-RCV_WIDTH/2)-(rotatedObstacleY-RCV_HEIGHT/2)*(rotatedObstacleY-RCV_HEIGHT/2));
		}
		else if (rotatedObstacleX > RCV_WIDTH/2) {minDistanceToObstacle = rotatedObstacleX - RCV_WIDTH/2;}
		else if (rotatedObstacleY > RCV_LENGTH/2) {minDistanceToObstacle = rotatedObstacleY - RCV_LENGTH/2;}
		else {minDistanceToObstacle=-1;} // Obstacle is inside the space defined by the vehicle

		if (minDistanceToObstacle < OBSTACLE_SAFETY_DISTANCE) {return true;}
		return false;
	}

	__global__ void checkIfaStarNodeIsTooCloseToObstaclesKernel(float nodeXpos, float nodeYpos, double localPathAngleFromPreviousNode, const ObstaclePoint* obstacleSquaresOnGPU, int nrOfObstacles, bool* isToClosePointerOnGPU) {
		int myThreadID = blockIdx.x*blockDim.x+threadIdx.x, myObstacleIndex = myThreadID/nrOfObstacles;
		float obstacleX,obstacleY;

		if (myObstacleIndex < nrOfObstacles) {
			obstacleX = (obstacleSquaresOnGPU + 4*myObstacleIndex)->x +GROUND_GRID_RESOLUTION/2.0;
			obstacleY = (obstacleSquaresOnGPU + 4*myObstacleIndex)->y +GROUND_GRID_RESOLUTION/2.0;

			*(isToClosePointerOnGPU) = checkIfPathPointIsTooCloseToObstacle(obstacleX,obstacleY,nodeXpos,nodeYpos,localPathAngleFromPreviousNode);
		}
	}

	__global__ void checkIfPathIsToCloseToObstaclesKernel(const PathPointInLocalXY* pathOnGPU, int lengthOfPath, const ObstaclePoint* obstacleSquaresOnGPU, int nrOfObstacles, double currentVehicleHeading, bool* isToClosePointerOnGPU) {
		/* Think of the function as a matrix, where nodes are the rows, and obstacles are the columns. For each node/obstacle pair, see if
		 * their distance is at least the min distance. To get the row and column index, row = index/columnSize, column = index - row*columnSize
		 */
		int myThreadID = blockIdx.x*blockDim.x+threadIdx.x, myPathIndex = myThreadID/nrOfObstacles, myObstacleIndex = myThreadID-myPathIndex*nrOfObstacles;
		float pathPointX,pathPointY,obstacleX,obstacleY;
		double localPathAngleFromPreviousNodeInPath;

		if (myPathIndex < lengthOfPath) {
			// The positions of the nodes and obstacles are the center distances of their respective square
			obstacleX = (obstacleSquaresOnGPU + 4*myObstacleIndex)->x +GROUND_GRID_RESOLUTION/2.0;
			obstacleY = (obstacleSquaresOnGPU + 4*myObstacleIndex)->y +GROUND_GRID_RESOLUTION/2.0;
			pathPointX = (pathOnGPU+myPathIndex)->x + GROUND_GRID_RESOLUTION/2.0;
			pathPointY = (pathOnGPU+myPathIndex)->y + GROUND_GRID_RESOLUTION/2.0;
			localPathAngleFromPreviousNodeInPath = currentVehicleHeading - (pathOnGPU+myPathIndex)->headingFromPrevPathPoint;

			*(isToClosePointerOnGPU) = checkIfPathPointIsTooCloseToObstacle(obstacleX,obstacleY,pathPointX,pathPointY,localPathAngleFromPreviousNodeInPath);
		}
	}

	bool checkIfPathIsToCloseToObstacles(PathPointInLocalXY* path, int lengthOfPath, const ObstaclePoint *obstacleSquaresOnGPU, int nrOfObstacles, double currentVehicleHeading) {
		// Returns false if all nodes in nodes have a min distance of minDistance to all obstacles in obstacleSquaresOnGPU, otherwise returns true
		bool* isToClosePointerOnGPU;
		CUDA_CHECK_RETURN(cudaMalloc((void**)&isToClosePointerOnGPU,sizeof(bool)));
		CUDA_CHECK_RETURN(cudaMemset(isToClosePointerOnGPU,0,sizeof(bool)));

		PathPointInLocalXY* pathOnGPU;
		CUDA_CHECK_RETURN(cudaMalloc((void**)&pathOnGPU,lengthOfPath*sizeof(PathPointInLocalXY)));
		CUDA_CHECK_RETURN(cudaMemcpy(path,pathOnGPU,lengthOfPath*sizeof(PathPointInLocalXY),cudaMemcpyDeviceToHost));

		int numberOfKernelBlocks = lengthOfPath*nrOfObstacles/256+1;
		checkIfPathIsToCloseToObstaclesKernel<<<numberOfKernelBlocks,256>>>(pathOnGPU,lengthOfPath,obstacleSquaresOnGPU,nrOfObstacles,currentVehicleHeading,isToClosePointerOnGPU);

		bool isToClose;
		CUDA_CHECK_RETURN(cudaMemcpy(&isToClose,isToClosePointerOnGPU,sizeof(bool),cudaMemcpyDeviceToHost));
		CUDA_CHECK_RETURN(cudaFree(isToClosePointerOnGPU));
		CUDA_CHECK_RETURN(cudaFree(pathOnGPU));
		cudaDeviceSynchronize(); // TODO change to specific instead of global stream
		return isToClose;
	}

	bool checkIfaStarNodeIsTooCloseToObstacles(float nodeXpos, float nodeYpos, double localPathAngleFromPreviousNode, const ObstaclePoint* obstacleSquaresOnGPU, int nrOfObstacles) {
		bool* isToClosePointerOnGPU;
		CUDA_CHECK_RETURN(cudaMalloc((void**)&isToClosePointerOnGPU,sizeof(bool)));
		CUDA_CHECK_RETURN(cudaMemset(isToClosePointerOnGPU,0,sizeof(bool)));

		int numberOfKernelBlocks = nrOfObstacles/256+1;
		checkIfaStarNodeIsTooCloseToObstaclesKernel<<<numberOfKernelBlocks,256>>>(nodeXpos,nodeYpos,localPathAngleFromPreviousNode,obstacleSquaresOnGPU,nrOfObstacles,isToClosePointerOnGPU);

		bool isToClose;
		CUDA_CHECK_RETURN(cudaMemcpy(&isToClose,isToClosePointerOnGPU,sizeof(bool),cudaMemcpyDeviceToHost));
		CUDA_CHECK_RETURN(cudaFree(isToClosePointerOnGPU));
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

	float getLatDistanceBetweenGPSpositions(GPSposition fromPosition, GPSposition toPosition) {
		// Returns the distance in meters between the longitudinal positions given by fromPositon and toPosition
		return (toPosition.latc - fromPosition.latc) * LENGTH_OF_ONE_LAT_DEGREE_IN_METERS;
	}
	float getLongDistanceBetweenGPSpositions(GPSposition fromPosition, GPSposition toPosition) {
		// Returns the distance in meters between the latitudinal positions given by fromPositon and toPosition
		return (toPosition.longc - fromPosition.longc) * LENGTH_OF_ONE_LONG_DEGREE_IN_METERS;
	}

	double getHeadingBetweenGPSpositions(GPSposition fromPosition, GPSposition toPosition) {
		// Returns the heading from fromPosition to toPosition in radians (i.e. the angle from NORTH to the line that goes through the two GPSpositions)

		float deltaLat = getLatDistanceBetweenGPSpositions(fromPosition,toPosition);
		float deltaLong = getLongDistanceBetweenGPSpositions(fromPosition,toPosition);

		return atan2(deltaLong,deltaLat);
	}
}


