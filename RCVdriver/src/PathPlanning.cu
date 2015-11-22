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
	__global__ void translatePathToXY(PathPointInGPScords* pathGPSOnGPU, PathPointInLocalXY* pathXYonGPU, int lengthOfPath, VehicleState vehicleState);
	__global__ void checkIfaStarNodeIsTooCloseToObstaclesKernel(float nodeX, float nodeY, double localPathAngleFromPreviousNode, const ObstaclePoint* obstacleSquaresOnGPU, int nrOfObstacles, bool* isTooClosePointerOnGPU);
	__global__ void checkPathForCollisions(PathPointInGPScords* pathGPSOnGPU, PathPointInLocalXY* pathXYonGPU, int lengthOfPath, const ObstaclePoint* obstacleSquaresOnGPU, int nrOfObstacles, VehicleState vehicleState, bool* isTooClosePointerOnGPU);
	float rotateAndGetX(float x, float y, double angle);
	float rotateAndGetY(float x, float y, double angle);
	float getLatDistanceBetweenGPSpositions(GPSposition fromPosition, GPSposition toPosition);
	float getLongDistanceBetweenGPSpositions(GPSposition fromPosition, GPSposition toPosition);
	double getHeadingBetweenGPSpositions(GPSposition fromPosition, GPSposition toPosition);
}

// PathPlanning:
PathPlanning::PathPlanning(const ObstaclePoint* obstacleSquaresOnGPU, const int &currentNrOfObstacles,const VehicleState &vehicleState, VehicleStatus& vehicleStatus):
		vehicleState(vehicleState),
		obstacleSquaresOnGPU(obstacleSquaresOnGPU),
		currentNrOfObstacles(currentNrOfObstacles),
		vehicleStatus(vehicleStatus) {

	this->hashTable=new HashTable();
	this->minHeap=new MinHeap();

	this->macroPathGPS=NULL;
	this->microPathGPS=NULL;
	this->microPathXY=NULL;
	this->macroPathXY=NULL;
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
	delete[] macroPathXY;
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
	translateMacroPathToXY();
	if ((lengthOfMicroPath==0 || !translateMicroPathToXYandCheckIfMicroPathIsTooCloseToObstacles()) && !generateMicroPath(macroPathXY[currentIndexInMacroPath].x,macroPathXY[currentIndexInMacroPath].y)) {
		/* If the code goes here, it means that {microPath hadn't been set OR microPath is no longer traversable}
		 * AND no new microPath could be generated. The program should then return
		 */
		vehicleStatus.isAbleToGenerateMicroPath=false;
		return;
	}

	if (updatePathIndexes()) {return;}

	// Update the path indexes, and return if the macroPath has been traversed
	vehicleStatus.isAbleToGenerateMicroPath=true;

	/* If the program makes it to here, the following is true:
	 * microPath has at least index 1 and length 2
	 * microPathXY has been translated by the code above (either by translateMicroPath or by generateMicroPath)
	 *
	 * Therefore microPath[currentIndexInMicroPath] and microPath[currentIndexInMicroPath-1] can
	 * be used as reference points for the lateral P-controller
	 */

	float lateralError, headingError;
	lateralError = rotateAndGetX(0-microPathXY[currentIndexInMicroPath-1].x,0-microPathXY[currentIndexInMicroPath-1].y,microPathGPS[currentIndexInMicroPath].headingFromPrevPathPoint);
	headingError = microPathGPS[currentIndexInMicroPath].headingFromPrevPathPoint - vehicleState.currentHeading;
	if (headingError>M_PI) {headingError-=2*M_PI;}
	if (headingError<M_PI) {headingError+=2*M_PI;}

	if (vehicleStatus.isRunning) {
		// Just send the control signals onto the CAN bus here
	}
}

bool PathPlanning::updatePathIndexes() {
	// This function updates currentIndexInMicroPath and currentIndexInMacroPath based on the current vehicle position
	if (lengthOfMicroPath==0) {return false;} //MicroPath hasn't been set yet, so return

	double currentPathAngle;
	float vehicleDeltaLatFromPrevGPSpoint,vehicleDeltaLongFromPrevGPSpoint,gpsPointDeltaLatFromPrevGPSpoint,gpsPointDeltaLongFromPrevGPSpoint,vehicleRotatedY,gpsPointRotatedY;
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
				 * uses macroPathGPS[currentIndexInMacroPath-1] as a reference position, and gets the delta distance latitudinally
				 * and longitudinally from the refpoint to the vehicle position and from the refpoint to macroGPSpath[currentIndexInMacroPath]
				 * The points given by {vehicleDeltaNorthFromPrevGPSpoint,vehicleDeltaEastFromPrevGPSpoint} and {gpsPointDeltaNorthFromPrevGPSpoint,
				 * gpsPointDeltaEastFromPrevGPSpoint} are then rotated counter clockwise by the heading (radians from true north) given by
				 * the path from macroPathGPS[currentIndexInMacroPath-1] to macroPathGPS[currentIndexInMacroPath]. By comparing the y coordinates
				 * of those rotated points, RCVdriver can see if the vehicle has passed macroPathGPS[currentIndexInMacroPath]. If so
				 * increase currentIndexInMacroPath
				 *
				 * Note that at this point in the code both currentIndexInMacroPath is at least 1, and and lengthOfMacroPath is at least 2, so
				 * there will be no buffer overflows reading from macroGPSpath
				 */
				vehicleDeltaLatFromPrevGPSpoint = getLatDistanceBetweenGPSpositions(macroPathGPS[currentIndexInMacroPath-1].position,vehicleState.currentPosition);
				vehicleDeltaLongFromPrevGPSpoint = getLongDistanceBetweenGPSpositions(macroPathGPS[currentIndexInMacroPath-1].position,vehicleState.currentPosition);
				gpsPointDeltaLatFromPrevGPSpoint = macroPathGPS[currentIndexInMacroPath].latDistanceFromPrevPathPoint;
				gpsPointDeltaLongFromPrevGPSpoint = macroPathGPS[currentIndexInMacroPath].longDistanceFromPrevPathPoint;
				currentPathAngle = macroPathGPS[currentIndexInMacroPath].headingFromPrevPathPoint;

				vehicleRotatedY = rotateAndGetY(vehicleDeltaLatFromPrevGPSpoint,vehicleDeltaLongFromPrevGPSpoint,currentPathAngle);
				gpsPointRotatedY = rotateAndGetY(gpsPointDeltaLatFromPrevGPSpoint,gpsPointDeltaLongFromPrevGPSpoint,currentPathAngle);

				if (vehicleRotatedY>gpsPointRotatedY) {currentIndexInMacroPath++;}
				else {break;} // If false, the currentIndexInMacroPath is still ahead of the vehicle, and should be the current target for the microPath
			}
			generateMicroPath(macroPathXY[currentIndexInMacroPath].x,macroPathXY[currentIndexInMacroPath].y);
			continue; // It is possible that microPath has length 1 at this point, so start from the top to check if that is the case (if so the macroPathIndex will be updated)
		}

		/* Now do the same thing as described above but with the microGPSpath. Just like with macroGPSpath, currentIndexInMicroPath
		 * is at least 1 and lengthOfMicroPath is at least 2 at this point
		 */
		vehicleDeltaLatFromPrevGPSpoint = getLatDistanceBetweenGPSpositions(microPathGPS[currentIndexInMicroPath-1].position,vehicleState.currentPosition);
		vehicleDeltaLongFromPrevGPSpoint = getLongDistanceBetweenGPSpositions(microPathGPS[currentIndexInMicroPath-1].position,vehicleState.currentPosition);
		gpsPointDeltaLatFromPrevGPSpoint = microPathGPS[currentIndexInMicroPath].latDistanceFromPrevPathPoint;
		gpsPointDeltaLongFromPrevGPSpoint = microPathGPS[currentIndexInMicroPath].longDistanceFromPrevPathPoint;
		currentPathAngle = microPathGPS[currentIndexInMicroPath].headingFromPrevPathPoint;

		vehicleRotatedY = rotateAndGetY(vehicleDeltaLatFromPrevGPSpoint,vehicleDeltaLongFromPrevGPSpoint,currentPathAngle);
		gpsPointRotatedY = rotateAndGetY(gpsPointDeltaLatFromPrevGPSpoint,gpsPointDeltaLongFromPrevGPSpoint,currentPathAngle);

		if (vehicleRotatedY>gpsPointRotatedY) {currentIndexInMicroPath++;}
		else {break;}
	}

	return false;
}

void PathPlanning::translateMacroPathToXY() {
	// Translates all the coordinates in macroPathGPS to local x,y and stores them is macroPathXY.

	int lengthOfMacroPathOnGPU = lengthOfMacroPath - (currentIndexInMacroPath-1);

	PathPointInGPScords* macroPathGPSOnGPU;
	PathPointInLocalXY* macroPathXYOnGPU;

	CUDA_CHECK_RETURN(cudaMalloc((void**)&macroPathGPSOnGPU,lengthOfMacroPathOnGPU*sizeof(PathPointInGPScords)));
	CUDA_CHECK_RETURN(cudaMalloc((void**)&macroPathXYOnGPU,lengthOfMacroPathOnGPU*sizeof(PathPointInLocalXY)));

	CUDA_CHECK_RETURN(cudaMemset(macroPathXYOnGPU,0,lengthOfMacroPathOnGPU*sizeof(PathPointInLocalXY)));
	CUDA_CHECK_RETURN(cudaMemcpy(macroPathGPSOnGPU,macroPathGPS+(currentIndexInMacroPath-1),lengthOfMacroPathOnGPU*sizeof(PathPointInGPScords),cudaMemcpyHostToDevice));

	int numberOfKernelBlocks = lengthOfMacroPathOnGPU/32+1;
	translatePathToXY<<<numberOfKernelBlocks,32>>>(macroPathGPSOnGPU,macroPathXYOnGPU,lengthOfMacroPathOnGPU,vehicleState);

	CUDA_CHECK_RETURN(cudaMemcpy(macroPathXY+(currentIndexInMacroPath-1),macroPathXYOnGPU,lengthOfMacroPathOnGPU*sizeof(PathPointInLocalXY),cudaMemcpyDeviceToHost));
	CUDA_CHECK_RETURN(cudaFree(macroPathGPSOnGPU));
	CUDA_CHECK_RETURN(cudaFree(macroPathXYOnGPU));

	cudaDeviceSynchronize(); // TODO change to specific instead of global stream
}

bool PathPlanning::translateMicroPathToXYandCheckIfMicroPathIsTooCloseToObstacles() const {
	/* Translates all the coordinates in microPathGPS to local x,y and stores them is microPathInLocalXY.
	 * After that the function checks if any point in microPathInLocalXY is too close to any obstacle
	 * defined in obstacleSquaresOnGPU, and if so returns true
	 *
	 * When translating the paths, the function only copy the points in the path that the vehicle hasn't
	 * already passed
	 */
	int lengthOfMicroPathOnGPU = lengthOfMicroPath - (currentIndexInMicroPath-1);
	// Translate all the points remaining on the microPath, as well as the point closest behind the vehicle

	bool* isTooClosePointerOnGPU;
	PathPointInGPScords* microPathGPSOnGPU;
	PathPointInLocalXY* microPathXYOnGPU;

	CUDA_CHECK_RETURN(cudaMalloc((void**)&isTooClosePointerOnGPU,sizeof(bool)));
	CUDA_CHECK_RETURN(cudaMalloc((void**)&microPathGPSOnGPU,lengthOfMicroPathOnGPU*sizeof(PathPointInGPScords)));
	CUDA_CHECK_RETURN(cudaMalloc((void**)&microPathXYOnGPU,lengthOfMicroPathOnGPU*sizeof(PathPointInGPScords)));

	CUDA_CHECK_RETURN(cudaMemset(microPathXYOnGPU,0,lengthOfMicroPathOnGPU*sizeof(PathPointInLocalXY)));
	CUDA_CHECK_RETURN(cudaMemset(isTooClosePointerOnGPU,0,sizeof(bool)));
	CUDA_CHECK_RETURN(cudaMemcpy(microPathGPSOnGPU,microPathGPS+(currentIndexInMicroPath-1),lengthOfMicroPathOnGPU*sizeof(PathPointInGPScords),cudaMemcpyHostToDevice));

	int numberOfKernelBlocks = lengthOfMicroPathOnGPU/32+1;
	translatePathToXY<<<numberOfKernelBlocks,32>>>(microPathGPSOnGPU,microPathXYOnGPU,lengthOfMicroPathOnGPU,vehicleState);

	/* Do collision check on microPathXY, but don't check collisions on the first node, since it is behind the vehicle
	 * and already has been passed
	 */
	numberOfKernelBlocks = (lengthOfMicroPathOnGPU-1)*currentNrOfObstacles/256+1;
	checkPathForCollisions<<<numberOfKernelBlocks,256>>>(microPathGPSOnGPU+1,microPathXYOnGPU+1,lengthOfMicroPathOnGPU-1,obstacleSquaresOnGPU,currentNrOfObstacles,vehicleState,isTooClosePointerOnGPU);

	bool isToClose;
	CUDA_CHECK_RETURN(cudaMemcpy(&isToClose,isTooClosePointerOnGPU,sizeof(bool),cudaMemcpyDeviceToHost));
	CUDA_CHECK_RETURN(cudaMemcpy(microPathXY+(currentIndexInMicroPath-1),microPathXYOnGPU,lengthOfMicroPathOnGPU*sizeof(PathPointInLocalXY),cudaMemcpyDeviceToHost));
	CUDA_CHECK_RETURN(cudaFree(isTooClosePointerOnGPU));
	CUDA_CHECK_RETURN(cudaFree(microPathGPSOnGPU));
	CUDA_CHECK_RETURN(cudaFree(microPathXYOnGPU));

	cudaDeviceSynchronize(); // TODO change to specific instead of global stream
	return isToClose;
}

void PathPlanning::setMacroPath(const char* filePath) {
	// Reset paths first
	clearAllPaths(true);

	// This function is called by Input when a new path is loaded. The filepath to the pathfile is the parameter
	FILE *fp;
	char line[60];

	if ((fp = fopen(filePath,"rt"))  == NULL) {
		printf("%s %s %s\n","Unable to open path file:",filePath, strerror(errno));
		return;
	}

	while (fgets(line, 60, fp)) {if (*line!=35) {break;}} // Comment line, starting with '#'
	sscanf(line,"%d",&lengthOfMacroPath);
	if (lengthOfMacroPath<1) {
		printf("%s\n","Error: path must be of length greater than zero");
		return;
	}

	lengthOfMacroPath++;
	macroPathGPS = new PathPointInGPScords[lengthOfMacroPath]();
	macroPathXY = new PathPointInLocalXY[lengthOfMacroPath]();
	/* Add the vehicles own position as the first position in the macroGPSpath; the vehicle
	 * still has to travel from its initial position to the start position of the path
	 */
	macroPathGPS[0].position.latc = vehicleState.currentPosition.latc;
	macroPathGPS[0].position.longc = vehicleState.currentPosition.longc;

	for (int i=1;i<lengthOfMacroPath;i++) {
		if (fgets(line,60,fp)==0) {
			printf("%s\n","Error: not all coordinates in path are provided in path file");
			clearAllPaths(true);
			return;
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
	printf("%s\n","Path successfully loaded");
}

bool PathPlanning::generateMicroPath(float targetX, float targetY) {
	/* This function generates a path from {0,0} to targetX,targetY using a variant of A* tailored for producing vehicle paths
	 * returns true if a path was found, and false otherwise
	 */

	// First delete the current microGPSpath:
	clearAllPaths(false);

	hashTable->clearHashTable();
	minHeap->clearMinHeap();

	aStarNode *baseNode = hashTable->addAstarNode(0,0), *targetNode = hashTable->addAstarNode(targetX,targetY);
	while (true) {
		baseNode->isOnClosedSet=true;
		baseNode->isOnOpenSet=false;

		if (baseNode->x == targetNode->x && baseNode->y == targetNode->y) {break;} // Path found

		if (minHeap->getAvailableSpace() < 20) {return false;} //Minheap is full, and path cannot be found
		for (int i=0;i<14;i++) {
			discoverNeighbor(baseNode,targetNode,i);
		}
		baseNode = minHeap->popNode();
		if (baseNode==NULL) {return false;} // MinHeap is empty, so no path could be found
	}

	// Path found, now create microPathGPS and microPathXY
	const aStarNode* currentNode = targetNode;
	while(true) { // Find number of nodes in the path
		if (currentNode->previousNodeInPath==NULL) {break;}

		currentNode = currentNode->previousNodeInPath;
		lengthOfMicroPath++;
	}

	lengthOfMicroPath++; // Make room for the first position in the path; the vehicles own position
	microPathGPS = new PathPointInGPScords[lengthOfMicroPath]();
	microPathXY = new PathPointInLocalXY[lengthOfMicroPath]();

	// Add the start node to the first position in the path
	translateLocalXYtoGPSposition(currentNode->x,currentNode->y,microPathGPS[0].position);
	microPathXY[0].x=0;
	microPathXY[0].y=0;

	currentNode = targetNode;
	for (int i=lengthOfMicroPath-1;i>0;i--) {
		translateLocalXYtoGPSposition(currentNode->x,currentNode->y,microPathGPS[i].position);
		microPathGPS[i].headingFromPrevPathPoint = vehicleState.currentHeading - currentNode->localPathAngleFromPreviousNode;
		microPathGPS[i].latDistanceFromPrevPathPoint = rotateAndGetY(currentNode->x-currentNode->previousNodeInPath->x,currentNode->y-currentNode->previousNodeInPath->y,vehicleState.currentHeading);
		microPathGPS[i].longDistanceFromPrevPathPoint = rotateAndGetX(currentNode->x-currentNode->previousNodeInPath->x,currentNode->y-currentNode->previousNodeInPath->y,vehicleState.currentHeading);
		microPathGPS[i].isReversingFromPrevNode = currentNode->pathIsReversingFromPrevNode;

		microPathXY[i].x = currentNode->x;
		microPathXY[i].y = currentNode->y;
		if (currentNode->pathIsReversingFromPrevNode) {microPathXY[i].r=255;} // Set colors of the path; red for reversing, green otherwise
		else {microPathXY[i].g=255;}

		currentNode=currentNode->previousNodeInPath;
	}

	currentIndexInMicroPath=1;
	/* Just like in setMacroPath(), the first position in microPathGPS is the vehicles current position, so
	 * therefore set currentIndexInMicroPath to 1, in order to make the vehicle start heading for the position
	 * in microPath that comes after its own position
	 */

	return true;
}

void PathPlanning::discoverNeighbor(const aStarNode *baseNode, const aStarNode *targetNode, int index) const {
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
	/* LocalPathAngleFromPreviousNode is the angle going from {the vector going from baseNode to nehgborNode}
	 * to {the vehicle centerline vector pointing forwards}. It is local in the coordinate system of the vehicle
	 */

	if (neighborLocalPathAngleFromPreviousNode < -M_PI) {neighborLocalPathAngleFromPreviousNode+=2*M_PI;}
	if (neighborLocalPathAngleFromPreviousNode > M_PI) {neighborLocalPathAngleFromPreviousNode-=2*M_PI;}

	// The coordinates of the neighbor node, the values rounded to match the obstacleMatrixResolution in the grid:
	float neighborX,neighborY;
	neighborX = baseNode->x + round(stepDistance * cos(neighborLocalPathAngleFromPreviousNode)/GROUND_GRID_RESOLUTION)*GROUND_GRID_RESOLUTION;
	neighborY = baseNode->y + round(stepDistance * sin(neighborLocalPathAngleFromPreviousNode)/GROUND_GRID_RESOLUTION)*GROUND_GRID_RESOLUTION;

	neighborNode = hashTable->addAstarNode(neighborX,neighborY); // Gets the neighborNode from the hashTable, or adds it if it hasn't been visited

	if (neighborNode->isOnClosedSet || checkIfaStarNodeIsTooCloseToObstacles(*neighborNode,neighborLocalPathAngleFromPreviousNode)) {return;}
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
		if(!minHeap->addNode(neighborNode)) {return;} // Minheap is full, however this should never happen, since that is being checked in generateMicroPath before calling this function

		// Node has not been discovered yet
		neighborNode->isOnOpenSet=true;

		neighborNode->distanceFromStartNode =neighborDistanceFromStartNode;
		neighborNode->heuristic = neighbourHeuristic;
		neighborNode->localPathAngleFromPreviousNode = neighborLocalPathAngleFromPreviousNode;
		neighborNode->previousNodeInPath = baseNode;
		if (index<7) {neighborNode->pathIsReversingFromPrevNode=false;}
		else {neighborNode->pathIsReversingFromPrevNode=true;}
	}
	else if (neighborDistanceFromStartNode < neighborNode->distanceFromStartNode) {
		neighborNode->distanceFromStartNode = neighborDistanceFromStartNode;
		neighborNode->heuristic = neighbourHeuristic;
		neighborNode->localPathAngleFromPreviousNode = neighborLocalPathAngleFromPreviousNode;
		neighborNode->previousNodeInPath = baseNode;
		if (index<7) {neighborNode->pathIsReversingFromPrevNode=false;}
		else {neighborNode->pathIsReversingFromPrevNode=true;}

		minHeap->bubbleNode(neighborNode);
	}
}

bool PathPlanning::checkIfaStarNodeIsTooCloseToObstacles(aStarNode node, double localPathAngleFromPreviousNode) const{
	// This function checks if node is too close to any obstacle defined in obstacleSquaresOnGPU, and returns true if so
	bool* isTooClosePointerOnGPU;
	CUDA_CHECK_RETURN(cudaMalloc((void**)&isTooClosePointerOnGPU,sizeof(bool)));
	CUDA_CHECK_RETURN(cudaMemset(isTooClosePointerOnGPU,0,sizeof(bool)));

	int numberOfKernelBlocks = currentNrOfObstacles/256+1;
	checkIfaStarNodeIsTooCloseToObstaclesKernel<<<numberOfKernelBlocks,256>>>(node.x,node.y,localPathAngleFromPreviousNode,obstacleSquaresOnGPU,currentNrOfObstacles,isTooClosePointerOnGPU);

	bool isToClose;
	CUDA_CHECK_RETURN(cudaMemcpy(&isToClose,isTooClosePointerOnGPU,sizeof(bool),cudaMemcpyDeviceToHost));
	CUDA_CHECK_RETURN(cudaFree(isTooClosePointerOnGPU));
	cudaDeviceSynchronize(); // TODO change to specific instead of global stream
	return isToClose;
}

void PathPlanning::clearAllPaths(bool includeMacroPath) {
	if (includeMacroPath) {
		delete[] macroPathGPS;
		delete[] macroPathXY;
		macroPathGPS=NULL;
		macroPathXY=NULL;
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

void PathPlanning::translateLocalXYtoGPSposition(float x, float y, GPSposition& target) const {
	/* The vehicle has the y axis pointing forward in the direction of travel. Rotate a point x
	 * and y counter clockwise by heading makes y align with true north, and x align with true east.
	 * Scale those and you get latc and longc
	 */
	float rotatedX = rotateAndGetX(x,y,-vehicleState.currentHeading);
	float rotatedY = rotateAndGetY(x,y,-vehicleState.currentHeading);

	target.latc = vehicleState.currentPosition.latc + rotatedY * (1.0/LENGTH_OF_ONE_LAT_DEGREE_IN_METERS);
	target.longc = vehicleState.currentPosition.longc + rotatedX * (1.0/LENGTH_OF_ONE_LONG_DEGREE_IN_METERS);
}

const PathPointInLocalXY* PathPlanning::getMicroPathXY() const {
	// Returns the microPathXY for graphical displaying
	return microPathXY + currentIndexInMicroPath;
}

const PathPointInLocalXY* PathPlanning::getMacroPathXY() const {
	// Returns the macroPathXY for graphical displaying
	return macroPathXY + currentIndexInMacroPath;
}

// Hashtable:
PathPlanning::HashTable::HashTable() {
	memset(hashArray,0,sizeof(hashArray)); // Set hashArray to null
}

PathPlanning::HashTable::~HashTable() {
	clearHashTable();
}

int PathPlanning::HashTable::getIndex(float x, float y) const {
	// I tested this hasher in matlab, it is based on Java's hashCode(), and it gives pretty even results with GROUND_GRID_RESOLUTION
	// between 0.01 and 0.2. X and Y are at this point always aligned with the ground grid

	int hash =100*abs(x)+100*abs(y)*abs(y);
	return hash % HASH_TABLE_ENTRIES;
}

PathPlanning::aStarNode* PathPlanning::HashTable::getAstarNode(float x, float y) const {
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
	for (int i=0;i<HASH_TABLE_ENTRIES;i++) {clearBucketList(hashArray[i]);}

	// Make all pointers in array null
	memset(hashArray,0,sizeof(hashArray));
}


PathPlanning::aStarNode *PathPlanning::HashTable::addAstarNode(float x, float y) {
	// First check if node already is in the hashtable
	aStarNode* addedNode = getAstarNode(x,y);
	if (addedNode != NULL) {return addedNode;}

	int arrayIndex = getIndex(x,y); // The index in the hashArray where the node is supposed to go

	HashBucket* bucketPointer=hashArray[arrayIndex];
	if (bucketPointer==NULL) {
		hashArray[arrayIndex]=new HashBucket(); //Everything (including all the fields of the aStarNode) set to zero
		bucketPointer=hashArray[arrayIndex];
	}
	else {
		while (bucketPointer->nextBucket!=NULL) {bucketPointer = bucketPointer->nextBucket;} // Get the latest bucket in the linked list for the specified index:
		bucketPointer->nextBucket = new HashBucket();
		bucketPointer = bucketPointer->nextBucket;
	}

	bucketPointer->node.x=x;
	bucketPointer->node.y=y;
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

PathPlanning::aStarNode* PathPlanning::MinHeap::popNode() {
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

int PathPlanning::MinHeap::getAvailableSpace() const {
	return HEAP_SIZE - currentNrOfNodesInHeap;
}

namespace {
// Non member and cuda functions implementations:
	__device__ bool checkIfPathPointIsTooCloseToObstacle(float pathPointX, float pathPointY, double localPathAngleFromPreviousNode,float obstacleX, float obstacleY) {
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
			minDistanceToObstacle = sqrt((rotatedObstacleX-RCV_WIDTH/2)*(rotatedObstacleX-RCV_WIDTH/2)+(rotatedObstacleY-RCV_HEIGHT/2)*(rotatedObstacleY-RCV_HEIGHT/2));
		}
		else if (rotatedObstacleX > RCV_WIDTH/2) {minDistanceToObstacle = rotatedObstacleX - RCV_WIDTH/2;}
		else if (rotatedObstacleY > RCV_LENGTH/2) {minDistanceToObstacle = rotatedObstacleY - RCV_LENGTH/2;}
		else {minDistanceToObstacle=-1;} // Obstacle is inside the space defined by the vehicle

		if (minDistanceToObstacle < OBSTACLE_SAFETY_DISTANCE) {return true;}
		return false;
	}

	__global__ void checkIfaStarNodeIsTooCloseToObstaclesKernel(float nodeX, float nodeY, double localPathAngleFromPreviousNode, const ObstaclePoint* obstacleSquaresOnGPU, int nrOfObstacles, bool* isTooClosePointerOnGPU) {
		int myThreadID = blockIdx.x*blockDim.x+threadIdx.x;
		float obstacleX,obstacleY;

		if (myThreadID < nrOfObstacles) {
			obstacleX = (obstacleSquaresOnGPU + 4*myThreadID)->x +GROUND_GRID_RESOLUTION/2.0;
			obstacleY = (obstacleSquaresOnGPU + 4*myThreadID)->y +GROUND_GRID_RESOLUTION/2.0;

			*(isTooClosePointerOnGPU) = checkIfPathPointIsTooCloseToObstacle(nodeX,nodeY,localPathAngleFromPreviousNode,obstacleX,obstacleY);
		}
	}

	__global__ void translatePathToXY(PathPointInGPScords* pathGPSOnGPU, PathPointInLocalXY* pathXYonGPU, int lengthOfPath, VehicleState vehicleState) {
		int myThreadID = blockIdx.x*blockDim.x+threadIdx.x;

		double longc, latc;
		float scaledX,scaledY,rotatedX,rotatedY;

		if (myThreadID < lengthOfPath) {
			longc = (pathGPSOnGPU+myThreadID)->position.longc;
			latc = (pathGPSOnGPU+myThreadID)->position.latc;

			scaledX = (longc - vehicleState.currentPosition.longc) * LENGTH_OF_ONE_LONG_DEGREE_IN_METERS;
			scaledY = (latc - vehicleState.currentPosition.latc) * LENGTH_OF_ONE_LAT_DEGREE_IN_METERS;
			rotatedX = scaledX * cos(vehicleState.currentHeading) - scaledY * sin(vehicleState.currentHeading);
			rotatedY = scaledX * sin(vehicleState.currentHeading) + scaledY * cos(vehicleState.currentHeading);
			(pathXYonGPU+myThreadID)->x=rotatedX;
			(pathXYonGPU+myThreadID)->y=rotatedY;

			if ((pathGPSOnGPU+myThreadID)->isReversingFromPrevNode) {(pathXYonGPU+myThreadID)->r=255;}
			else {(pathXYonGPU+myThreadID)->g=255;}
		}
	}

	__global__ void checkPathForCollisions(PathPointInGPScords* pathGPSOnGPU, PathPointInLocalXY* pathXYonGPU, int lengthOfPath, const ObstaclePoint* obstacleSquaresOnGPU, int nrOfObstacles, VehicleState vehicleState, bool* isTooClosePointerOnGPU) {
		/* Think of the function as a matrix, where nodes are the rows, and obstacles are the columns. For each node/obstacle pair, see if
		 * their distance is at least the min distance. To get the row and column index, row = index/columnSize, column = index - row*columnSize
		 */
		int myThreadID = blockIdx.x*blockDim.x+threadIdx.x, myPathIndex = myThreadID/nrOfObstacles, myObstacleIndex = myThreadID-myPathIndex*nrOfObstacles;
		float pathPointX,pathPointY,obstacleX,obstacleY;
		double localPathAngleFromPreviousNode;

		if (myThreadID < lengthOfPath*nrOfObstacles) {
			// The positions of the nodes and obstacles are the center distances of their respective square
			pathPointX = (pathXYonGPU+myPathIndex)->x + GROUND_GRID_RESOLUTION/2.0;
			pathPointY = (pathXYonGPU+myPathIndex)->y + GROUND_GRID_RESOLUTION/2.0;
			localPathAngleFromPreviousNode = vehicleState.currentHeading - (pathGPSOnGPU+myPathIndex)->headingFromPrevPathPoint;

			obstacleX = (obstacleSquaresOnGPU + 4*myObstacleIndex)->x +GROUND_GRID_RESOLUTION/2.0;
			obstacleY = (obstacleSquaresOnGPU + 4*myObstacleIndex)->y +GROUND_GRID_RESOLUTION/2.0;

			*(isTooClosePointerOnGPU) = checkIfPathPointIsTooCloseToObstacle(pathPointX,pathPointY,localPathAngleFromPreviousNode,obstacleX,obstacleY);

			//TODO explain localPathAngleFromPreviousNode
		}
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


