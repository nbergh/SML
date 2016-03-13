#include "PathPlanner.h"
#include <assert.h>
#include <vector>

#define DEBUG 1

PathPlanner::PathPlanner(float occupancyGridCellSize):
	occupancyGridCellSize(occupancyGridCellSize) {

	grid = new aStarNode*[PARAMETERS::NR_OCCUPANCY_GRID_CELLS_X_WISE];
	for (int i=0;i<PARAMETERS::NR_OCCUPANCY_GRID_CELLS_X_WISE;i++) {
		grid[i] = new aStarNode[PARAMETERS::NR_OCCUPANCY_GRID_CELLS_Y_WISE]();
	}
	intializeRobotCollisionCheckingPoints();

	if (DEBUG) {debugSearchOut.open("/home/sml-linux/Matlab/cvap_astar_out.m");}
}

PathPlanner::~PathPlanner() {
	for (int i=0;i<PARAMETERS::NR_OCCUPANCY_GRID_CELLS_X_WISE;i++) {
		delete[] grid[i];
	}
	delete[] grid;
	delete[] robotCollisionCheckingPoints;

	if (DEBUG) {debugSearchOut.close();}
}

void PathPlanner::intializeRobotCollisionCheckingPoints() {
	// Initialize the collision checking points. Each obstacle in the map is expanded so that it makes up a 3*3 cell square.
	// Therefore, represent the robot as a number of points with less than 3*occupancyGridCellSize meters between them
	float halfStep = 1.49*occupancyGridCellSize; // Half of the distance that separates each collision checking point. 1.49 (instead of 1.5) is used because of rounding errors
	int nrPointsWidhtWise = (PARAMETERS::ROBOT_WIDTH-2*halfStep) / (2*halfStep) + 2;
	int nrPointsLengthWise = (PARAMETERS::ROBOT_LENGTH-2*halfStep) / (2*halfStep) + 2;
	int index=0;

	nrOfCollisionCheckingPoints = nrPointsWidhtWise * nrPointsLengthWise;
	robotCollisionCheckingPoints = new robotCollisionCheckingPoint[nrOfCollisionCheckingPoints];

	float currentX,currentY;
	currentX = halfStep - PARAMETERS::ROBOT_WIDTH/2.0;
	for (int i=0;i<nrPointsWidhtWise;i++) {
		currentY = halfStep - PARAMETERS::ROBOT_LENGTH/2.0;
		for (int j=0;j<nrPointsLengthWise;j++) {
			robotCollisionCheckingPoints[index].x = currentX;
			robotCollisionCheckingPoints[index].y = currentY;
			index++;
			currentY+=2*halfStep;
			if (j==nrPointsLengthWise-2) {currentY = PARAMETERS::ROBOT_LENGTH/2.0 - halfStep;}
		}
		currentX+=2*halfStep;
		if (i==nrPointsWidhtWise-2) {currentX = PARAMETERS::ROBOT_WIDTH/2.0 - halfStep;}
	}
}

void PathPlanner::zeroOutGrid() {
	for (int i=0;i<PARAMETERS::NR_OCCUPANCY_GRID_CELLS_X_WISE;i++) {
		memset(grid[i],0,PARAMETERS::NR_OCCUPANCY_GRID_CELLS_Y_WISE*sizeof(aStarNode));
	}
}

void PathPlanner::planNewPath(unsigned char** costMap, int costMapNrOfCellsXwise, int costMapNrOfCellsYwise, float costMapLowestX, float costMapLowestY, float startX, float startY, float goalX, float goalY) {
	mapData.costMap = costMap;
	mapData.nrOfCellsXwise = costMapNrOfCellsXwise;
	mapData.nrOfCellsYwise = costMapNrOfCellsYwise;
	// All of the following parameters are in meters:
	mapData.lowestXinMap = costMapLowestX;
	mapData.lowestYinMap = costMapLowestY;
	mapData.pathStartX = startX;
	mapData.pathStartY = startY;
	mapData.pathGoalX = goalX;
	mapData.pathGoalY = goalY;

	zeroOutGrid();
	minHeap.clearMinHeap();

	float localGoalX = goalX - startX;
	float localGoalY = goalY - startY;
	float distanceBaseNodeToGoal;
	aStarNode* baseNode = getNodeFromGrid(0,0);

	baseNode->x=0;
	baseNode->y=0;

	int iters=0;

	while(true) {
		baseNode->isOnClosedSet=true;
		baseNode->isOnOpenSet=false;
		distanceBaseNodeToGoal = sqrtf((localGoalX-baseNode->x)*(localGoalX-baseNode->x)+(localGoalY-baseNode->y)*(localGoalY-baseNode->y));

		if (distanceBaseNodeToGoal < 0.75*PARAMETERS::ASTAR_EXPANSION_STEP_LENGTH*occupancyGridCellSize) {
			// Goal reached
			break;
		}
//		if (iters==2000) {break;}

		for (int i=0;i<8;i++) {discoverNeighbor(*baseNode,i,localGoalX,localGoalY);}

		baseNode = minHeap.popNode();

		// If minheap is full or empty, then a path could not be found, so return:
		if (baseNode==NULL || minHeap.getAvailableSpace() < 8) {
			return;
		}
		iters++;
//		std::cout << iters << std::endl;
	}



	// Path found, now create microPathGPS and microPathXY
//	const aStarNode* currentNode = baseNode;
//	while(true) { // Find number of nodes in the path
//		if (currentNode->previousNodeInPath==NULL) {break;}
//
//		currentNode = currentNode->previousNodeInPath;
//		lengthOfMicroPath++;
//	}
//
//	lengthOfMicroPath++; // Make room for the first position in the path; the vehicles own position
//	microPathGPS = new PathPointInGPScords[lengthOfMicroPath]();
//	microPathXY = new PathPointInLocalXY[lengthOfMicroPath]();
//
//	// Add the start node to the first position in the path
//	microPathGPS[0].position=vehiclePosition.currentPosition;
//	microPathXY[0].x=0;
//	microPathXY[0].y=0;
//
//	currentNode = baseNode;
//	for (int i=lengthOfMicroPath-1;i>0;i--) {
//		/* The vehicle has the y axis pointing forward in the direction of travel. Rotate a point x
//		 * and y counter clockwise by heading makes y align with true north, and x align with true east.
//		 * Scale those and you get latc and longc
//		 */
//		microPathGPS[i].position.latc = vehiclePosition.currentPosition.latc + rotateAndGetY(currentNode->x,currentNode->y,vehiclePosition.currentHeading) * (1.0/LENGTH_OF_ONE_LAT_DEGREE_IN_METERS);
//		microPathGPS[i].position.longc = vehiclePosition.currentPosition.longc + rotateAndGetX(currentNode->x,currentNode->y,vehiclePosition.currentHeading) * (1.0/LENGTH_OF_ONE_LONG_DEGREE_IN_METERS);
//		microPathGPS[i].courseFromPreviousPathPoint = atan2f(currentNode->x-currentNode->previousNodeInPath->x,currentNode->y-currentNode->previousNodeInPath->y) + vehiclePosition.currentHeading;
//		microPathGPS[i].latDistanceFromPrevPathPoint = rotateAndGetY(currentNode->x-currentNode->previousNodeInPath->x,currentNode->y-currentNode->previousNodeInPath->y,vehiclePosition.currentHeading);
//		microPathGPS[i].longDistanceFromPrevPathPoint = rotateAndGetX(currentNode->x-currentNode->previousNodeInPath->x,currentNode->y-currentNode->previousNodeInPath->y,vehiclePosition.currentHeading);
//		microPathGPS[i].isReversingFromPrevNode = currentNode->vehicleIsReversingFromPrevNode;
//
//		microPathXY[i].x = currentNode->x;
//		microPathXY[i].y = currentNode->y;
//		if (currentNode->vehicleIsReversingFromPrevNode) {microPathXY[i].r=255;} // Set colors of the path; red for reversing, green otherwise
//		else {microPathXY[i].g=255;}
//
//		// Path debugging:
////		if (!currentNode->vehicleIsReversingFromPrevNode) {printf("%s%f%s%f%s%f%s%f%s\n","quiver(",currentNode->x,",",currentNode->y,",",currentNode->previousNodeInPath->x-currentNode->x,",",currentNode->previousNodeInPath->y-currentNode->y,",0,'g')");}
////		else {printf("%s%f%s%f%s%f%s%f%s\n","quiver(",currentNode->x,",",currentNode->y,",",currentNode->previousNodeInPath->x-currentNode->x,",",currentNode->previousNodeInPath->y-currentNode->y,",0,'r')");}
//
//		currentNode=currentNode->previousNodeInPath;
//	}
//
//	currentIndexInMicroPath=1;
//	/* Just like in setMacroPath(), the first position in microPathGPS is the vehicles current position, so
//	 * therefore set currentIndexInMicroPath to 1, in order to make the vehicle start heading for the position
//	 * in microPath that comes after its own position
//	 */
//
//	return true;
//	}

}

void PathPlanner::discoverNeighbor(aStarNode& baseNode, int index, float goalX, float goalY) {
	aStarNode* neighborNode;
	float distanceBaseToNeighbor,distanceStartToNeighbor,costBaseToNeighbor,costStartToNeighbor,distanceNeighborToGoal,neighborHeuristic;
	float deltaX,deltaY;
	switch (index) {
		case 0 : deltaX=-1;deltaY=-1; break;
		case 1 : deltaX=-1;deltaY=0; break;
		case 2 : deltaX=-1;deltaY=1; break;
		case 3 : deltaX=0;deltaY=-1; break;
		case 4 : deltaX=0;deltaY=1; break;
		case 5 : deltaX=1;deltaY=-1; break;
		case 6 : deltaX=1;deltaY=0; break;
		case 7 : deltaX=1;deltaY=1; break;
	}
	deltaX*=PARAMETERS::ASTAR_EXPANSION_STEP_LENGTH*occupancyGridCellSize;
	deltaY*=PARAMETERS::ASTAR_EXPANSION_STEP_LENGTH*occupancyGridCellSize;
	float robotHeading = atan2f(deltaX,deltaY);

	neighborNode = getNodeFromGrid(baseNode.x+deltaX,baseNode.y+deltaY);
	if (neighborNode==0) {return;} // neighborNode is 0 if it refers to a node that is outside of the grid (x or y too high/low)
	assert(neighborNode!=&baseNode);
	neighborNode->x = baseNode.x+deltaX;
	neighborNode->y = baseNode.y+deltaY;

	if (neighborNode->isOnClosedSet || (costBaseToNeighbor=getCostForNode(neighborNode,robotHeading))==-1) {return;} // Collision detected, so return

	distanceBaseToNeighbor = sqrtf((baseNode.x-neighborNode->x)*(baseNode.x-neighborNode->x)+(baseNode.y-neighborNode->y)*(baseNode.y-neighborNode->y));
	costBaseToNeighbor*=distanceBaseToNeighbor;

	distanceStartToNeighbor = baseNode.distanceFromStartNode + distanceBaseToNeighbor;
	costStartToNeighbor = baseNode.pathCostFromStartNode + costBaseToNeighbor;
	distanceNeighborToGoal = sqrtf((neighborNode->x-goalX)*(neighborNode->x-goalX)+(neighborNode->y-goalY)*(neighborNode->y-goalY));

	neighborHeuristic = /*distanceStartToNeighbor*/ costStartToNeighbor/100.0 + distanceNeighborToGoal;

	if (!neighborNode->isOnOpenSet) {
		// Node has not been discovered yet
		neighborNode->isOnOpenSet=true;

		neighborNode->distanceFromStartNode = distanceStartToNeighbor;
		neighborNode->pathCostFromStartNode = costStartToNeighbor;
		neighborNode->heuristic = neighborHeuristic;
		neighborNode->prevNodeInPath = &baseNode;

		minHeap.addNode(*neighborNode);

		// Path debugging:
		if(DEBUG) {debugPrintExpansion(baseNode,*neighborNode);}

	}
	else if (costStartToNeighbor < neighborNode->pathCostFromStartNode) {
		if(DEBUG) {debugPrintDeleteExpansion(*neighborNode->prevNodeInPath,*neighborNode);}

		neighborNode->distanceFromStartNode = distanceStartToNeighbor;
		neighborNode->pathCostFromStartNode = costStartToNeighbor;
		neighborNode->heuristic = neighborHeuristic;
		neighborNode->prevNodeInPath = &baseNode;

		minHeap.bubbleNode(*neighborNode);

		// Path debugging:
		if(DEBUG) {debugPrintExpansion(baseNode,*neighborNode);}
	}
}

float PathPlanner::getCostForNode(const aStarNode* neighborNode, float robotHeading) {
	float pointX,pointY,translatedX,translatedY,sum=0;
	int cost;

	for (int i=0;i<nrOfCollisionCheckingPoints;i++) {
		// This can be done in CUDA for faster execution. Without collision checking, a path is found 15 % of the time it takes with collision checking
		pointX = robotCollisionCheckingPoints[i].x;
		pointY = robotCollisionCheckingPoints[i].y;

		translatedX = pointX * cosf(robotHeading) + pointY * sinf(robotHeading);
		translatedY = -pointX * sinf(robotHeading) + pointY * cosf(robotHeading);
		translatedX += neighborNode->x;
		translatedY += neighborNode->y;

		if ((cost=getValueFromGlobalMap(translatedX,translatedY))==255) {
			return -1;
		}
		// -1 means that the node is within collision distance of an obstacle, and cannot be occupied by the robot
		sum+=cost;
	}
	return sum/nrOfCollisionCheckingPoints;
}

int PathPlanner::getValueFromGlobalMap(float localX, float localY) {
	float globalX = localX + mapData.pathStartX;
	float globalY = localY + mapData.pathStartY;

	float mapIndexXfloat = (globalX - mapData.lowestXinMap)/occupancyGridCellSize;
	float mapIndexYfloat = (globalY - mapData.lowestYinMap)/occupancyGridCellSize;
	int mapIndexX = (int)mapIndexXfloat;
	int mapIndexY = (int)mapIndexYfloat;

	if (abs(mapIndexXfloat-mapIndexX)>0.99) {mapIndexX = round(mapIndexXfloat);} //Sometimes gridXfloat will be 1.99999 (because of rounding errors) which will be int casted to 1, when it should be 2
	if (abs(mapIndexYfloat-mapIndexY)>0.99) {mapIndexY = round(mapIndexYfloat);}


	if (mapIndexX >= 0 && mapIndexY >= 0 && mapIndexX < mapData.nrOfCellsXwise && mapIndexY < mapData.nrOfCellsYwise) {
		return mapData.costMap[mapIndexX][mapIndexY];
	}
	return 0; // If the index is outside of the map, it is considered unknown
}

PathPlanner::aStarNode* PathPlanner::getNodeFromGrid(float x, float y) {
	float gridXfloat = x/occupancyGridCellSize + PARAMETERS::NR_OCCUPANCY_GRID_CELLS_X_WISE/2.0;
	float gridYfloat = y/occupancyGridCellSize + PARAMETERS::NR_OCCUPANCY_GRID_CELLS_Y_WISE/2.0;
	int gridX = (int)gridXfloat;
	int gridY = (int)gridYfloat;

	if (abs(gridXfloat-gridX)>0.99) {gridX = round(gridXfloat);} //Sometimes gridXfloat will be 1.99999 (because of rounding errors) which will be int casted to 1, when it should be 2
	if (abs(gridYfloat-gridY)>0.99) {gridY = round(gridYfloat);}

	if (gridX>=0 && gridY>=0 && gridX < PARAMETERS::NR_OCCUPANCY_GRID_CELLS_X_WISE && gridY < PARAMETERS::NR_OCCUPANCY_GRID_CELLS_Y_WISE) {
		return &grid[gridX][gridY];
	}
	return 0;
}

void PathPlanner::debugPrintDeleteExpansion(const aStarNode& baseNode,const aStarNode& neighborNode) {
	struct inputDataStruct {
		float fromX,toX,fromY,toY;
	};

	assert(neighborNode.prevNodeInPath!=0);

	debugSearchOut.close();
	std::ifstream debugSearchIn("/home/sml-linux/Matlab/cvap_astar_out.m");
	std::vector<inputDataStruct> inputData;
	inputDataStruct inputDataRow;

	// If replanned, the old path from neighborNode.prevNode to neighborNode needs to be deleted
	int row=0; bool done=false;
	while (debugSearchIn){
		while (!isdigit(debugSearchIn.peek()) && debugSearchIn.peek()!='-') {
			if (!debugSearchIn) {done=true;break;}
			debugSearchIn.get();
		}
		if (done) {break;}
		debugSearchIn >> inputDataRow.fromX;
		while (!isdigit(debugSearchIn.peek()) && debugSearchIn.peek()!='-') {debugSearchIn.get();}
		debugSearchIn >> inputDataRow.toX;
		while (!isdigit(debugSearchIn.peek()) && debugSearchIn.peek()!='-') {debugSearchIn.get();}
		debugSearchIn >> inputDataRow.fromY;
		while (!isdigit(debugSearchIn.peek()) && debugSearchIn.peek()!='-') {debugSearchIn.get();}
		debugSearchIn >> inputDataRow.toY;

		row++;
		if (	round(inputDataRow.fromX/occupancyGridCellSize) == round(neighborNode.x/occupancyGridCellSize) &&
				round(inputDataRow.toX/occupancyGridCellSize) == round(baseNode.x/occupancyGridCellSize) &&
				round(inputDataRow.fromY/occupancyGridCellSize) == round(neighborNode.y/occupancyGridCellSize) &&
				round(inputDataRow.toY/occupancyGridCellSize) == round(baseNode.y/occupancyGridCellSize)) {
			continue; // Skip this row
		}

		inputData.push_back(inputDataRow);
	}
	debugSearchIn.close();
	debugSearchOut.open("/home/sml-linux/Matlab/cvap_astar_out.m");

	for (std::vector<inputDataStruct>::iterator it=inputData.begin(); it!=inputData.end(); it++) {
		debugSearchOut <<"plot([" << it->fromX << "," << it->toX << "],[" << it->fromY << "," << it->toY << "],'g')" << std::endl;
	}
}

void PathPlanner::debugPrintExpansion(const aStarNode& baseNode, const aStarNode& neighborNode) {
	debugSearchOut <<"plot([" << neighborNode.x << "," << baseNode.x << "],[" << neighborNode.y << "," << baseNode.y << "],'g')" << std::endl;
}

void PathPlanner::debugPrintPath() {


}

void PathPlanner::debugTestHeap() {
//	bool isHealtyh=true;
	float lowestH=-1;
	aStarNode* currentNode;
	currentNode = minHeap.popNode();

	while (currentNode!=0) {
		if (!(lowestH==-1 || (currentNode->heuristic >= lowestH))) {
//			isHealtyh=false;
		}
		lowestH=currentNode->heuristic;
		currentNode=minHeap.popNode();
	}

	exit(0);
}

void PathPlanner::MinHeap::addNode(aStarNode& node) {
	if (currentNrOfNodesInHeap==PARAMETERS::HEAP_SIZE) {return;} // Return if heap is full; meaning that no path can be generated

	heapArray[currentNrOfNodesInHeap] = &node;
	node.heapArrayIndex = currentNrOfNodesInHeap;

	bubbleNode(node);

	currentNrOfNodesInHeap++;
}

void PathPlanner::MinHeap::bubbleNode(aStarNode& node) {
	// Bubble up, see https://en.wikipedia.org/wiki/Binary_heap
	while (true) {
		// If parents heuristic is lower than mine, or if i already have the lowest, then i don't need to bubble
		if (node.heapArrayIndex==0 || heapArray[(node.heapArrayIndex-1)/2]->heuristic < heapArray[node.heapArrayIndex]->heuristic) {return;}

		// Switch position with parent
		heapArray[node.heapArrayIndex] = heapArray[(node.heapArrayIndex-1)/2]; // Put parent in my position in the heapArray
		heapArray[node.heapArrayIndex]->heapArrayIndex = node.heapArrayIndex; // Update parent heapArrayIndex

		heapArray[(node.heapArrayIndex-1)/2]=&node; // Put me in my parents position
		node.heapArrayIndex = (node.heapArrayIndex-1)/2; // Update my heapArrayIndex
	}
}

PathPlanner::aStarNode* PathPlanner::MinHeap::popNode() {
	if (currentNrOfNodesInHeap==0) {return NULL;}
	aStarNode *returnNode = heapArray[0], *tempNode;

	currentNrOfNodesInHeap--;
	heapArray[0] = heapArray[currentNrOfNodesInHeap];
	heapArray[currentNrOfNodesInHeap]->heapArrayIndex=0;

	int currentNodeIndex=0,childNodeIndex;
	double childHeuristics;
	while (true) {
		// Bubble down

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

