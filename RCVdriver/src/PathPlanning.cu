#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h> // For memset
#include <errno.h>
#include <sys/time.h> // For timing the path finding

#include "Headers/Parameters.h"
#include "Headers/PathPlanning.h"
#include "Headers/CudaErrorCheckFunctions.h"

#define LENGTH_OF_ONE_LONG_DEGREE_IN_METERS 57141
#define LENGTH_OF_ONE_LAT_DEGREE_IN_METERS 111398

namespace {
	// Non-member function declarations
	__device__ bool nodeIsCollidingOnGPU; // Flag used by the GPU to indicate if an aStar or pathnode is within collision distance to an obstacle
	__global__ void translatePathToXY(PathPointInGPScords* pathGPSOnGPU, PathPointInLocalXY* pathXYonGPU, int lengthOfPath, VehiclePosition vehiclePosition);
	__global__ void checkPathForCollisions(PathPointInGPScords* pathGPSOnGPU, PathPointInLocalXY* pathXYonGPU, int lengthOfPath, const char* occupancyGridOnGPU, const PathPlanning::vehicleCollisionCheckCoordinate* vehicleCollisionCheckingPointsOnGPU, int nrOfVehicleCollisionCheckingPoints);
	__global__ void checkIfNodeIsColliding(float nodeX, float nodeY, float localVehicleHeadingAtNode, const char* occupancyGridOnGPU, const PathPlanning::vehicleCollisionCheckCoordinate* vehicleCollisionCheckingPointsOnGPU, int nrOfVehicleCollisionCheckingPoints);
	float rotateAndGetX(float x, float y, double angle);
	float rotateAndGetY(float x, float y, double angle);
	float getLatDistanceBetweenGPSpositions(GPSposition fromPosition, GPSposition toPosition);
	float getLongDistanceBetweenGPSpositions(GPSposition fromPosition, GPSposition toPosition);
	double getCourseBetweenGPSpositions(GPSposition fromPosition, GPSposition toPosition);
}
// PathPlanning:
PathPlanning::PathPlanning(const LidarExportData& lidarExportData,const VehiclePosition &vehiclePosition, VehicleStatus& vehicleStatus):
		lidarExportData(lidarExportData),vehiclePosition(vehiclePosition),vehicleStatus(vehicleStatus),
		pathExportData(lengthOfMacroPath,currentIndexInMacroPath,lengthOfMicroPath,currentIndexInMicroPath,macroPathXY,microPathXY),
		minHeap(),hashTable() {

	macroPathGPS=NULL;
	microPathGPS=NULL;
	microPathXY=NULL;
	macroPathXY=NULL;
	lengthOfMacroPath=0;
	lengthOfMicroPath=0;
	currentIndexInMacroPath=0;
	currentIndexInMicroPath=0;

	macroPathFilePath=NULL;
	loadNewMacroPathFlag=false;

	initializeCollisionCheckingPoints();
}

PathPlanning::~PathPlanning() {
	CUDA_ERROR_CHECK_FUNCTIONS::CUDA_CHECK_RETURN(cudaFree(vehicleCollisionCheckingPointsOnGPU));
	clearAllPaths(true);
}

void PathPlanning::updatePathAndControlSignals() {
	/* This is the main path-updating function that gets called once every iteration of the main control loop
	 * There are two main paths, macroPath and microPath. MacroPath is defined by the user when he types
	 * the command loadpath in the input. MicroPath is the path between the points in the macroPath. MicroPath
	 * is the path the vehicle will follow and adjusting its control signals to. MicroPath will be replanned
	 * if an obstacle is detected along its path
	 */
	if (loadNewMacroPathFlag) {loadNewMacroPath();}
	if (lengthOfMacroPath==0) { // MacroPath has not been set, so return
		vehicleStatus.macroPathHasBeenSet=false;
		vehicleStatus.isRunning=false;
		return;
	}
	translateMacroPathToXY();
	if ((lengthOfMicroPath==0 || translateMicroPathToXYandCheckIfMicroPathIsTooCloseToObstacles()) && !generateMicroPath(macroPathXY[currentIndexInMacroPath].x,macroPathXY[currentIndexInMacroPath].y,getTargetHeadingForMicroPath())) {

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

//	float lateralError, headingError;
//	lateralError = rotateAndGetX(0-microPathXY[currentIndexInMicroPath-1].x,0-microPathXY[currentIndexInMicroPath-1].y,microPathGPS[currentIndexInMicroPath].courseFromPreviousPathPoint);
//	headingError = microPathGPS[currentIndexInMicroPath].headingAtNode - vehiclePosition.currentHeading;
//	if (headingError>M_PI) {headingError-=2*M_PI;}
//	if (headingError<M_PI) {headingError+=2*M_PI;}

	if (vehicleStatus.isRunning) {
		// Just send the control signals onto the CAN bus here
	}
}

bool PathPlanning::updatePathIndexes() {
	// This function updates currentIndexInMicroPath and currentIndexInMacroPath based on the current vehicle position
	double currentPathCourse;
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
				vehicleDeltaLatFromPrevGPSpoint = getLatDistanceBetweenGPSpositions(macroPathGPS[currentIndexInMacroPath-1].position,vehiclePosition.currentPosition);
				vehicleDeltaLongFromPrevGPSpoint = getLongDistanceBetweenGPSpositions(macroPathGPS[currentIndexInMacroPath-1].position,vehiclePosition.currentPosition);
				gpsPointDeltaLatFromPrevGPSpoint = macroPathGPS[currentIndexInMacroPath].latDistanceFromPrevPathPoint;
				gpsPointDeltaLongFromPrevGPSpoint = macroPathGPS[currentIndexInMacroPath].longDistanceFromPrevPathPoint;
				currentPathCourse = macroPathGPS[currentIndexInMacroPath].courseFromPreviousPathPoint;

				vehicleRotatedY = rotateAndGetY(vehicleDeltaLongFromPrevGPSpoint,vehicleDeltaLatFromPrevGPSpoint,currentPathCourse);
				gpsPointRotatedY = rotateAndGetY(gpsPointDeltaLongFromPrevGPSpoint,gpsPointDeltaLatFromPrevGPSpoint,currentPathCourse);

				if (vehicleRotatedY>gpsPointRotatedY) {currentIndexInMacroPath++;}
				else {break;} // If false, the currentIndexInMacroPath is still ahead of the vehicle, and should be the current target for the microPath
			}
			generateMicroPath(macroPathXY[currentIndexInMacroPath].x,macroPathXY[currentIndexInMacroPath].y,getTargetHeadingForMicroPath());
			continue; // It is possible that microPath has length 1 at this point, so start from the top to check if that is the case (if so the macroPathIndex will be updated)
		}

		/* Now do the same thing as described above but with the microGPSpath. Just like with macroGPSpath, currentIndexInMicroPath
		 * is at least 1 and lengthOfMicroPath is at least 2 at this point
		 */
		vehicleDeltaLatFromPrevGPSpoint = getLatDistanceBetweenGPSpositions(microPathGPS[currentIndexInMicroPath-1].position,vehiclePosition.currentPosition);
		vehicleDeltaLongFromPrevGPSpoint = getLongDistanceBetweenGPSpositions(microPathGPS[currentIndexInMicroPath-1].position,vehiclePosition.currentPosition);
		gpsPointDeltaLatFromPrevGPSpoint = microPathGPS[currentIndexInMicroPath].latDistanceFromPrevPathPoint;
		gpsPointDeltaLongFromPrevGPSpoint = microPathGPS[currentIndexInMicroPath].longDistanceFromPrevPathPoint;
		currentPathCourse = microPathGPS[currentIndexInMicroPath].courseFromPreviousPathPoint;

		vehicleRotatedY = rotateAndGetY(vehicleDeltaLongFromPrevGPSpoint,vehicleDeltaLatFromPrevGPSpoint,currentPathCourse);
		gpsPointRotatedY = rotateAndGetY(gpsPointDeltaLongFromPrevGPSpoint,gpsPointDeltaLatFromPrevGPSpoint,currentPathCourse);

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

	CUDA_ERROR_CHECK_FUNCTIONS::CUDA_CHECK_RETURN(cudaMalloc((void**)&macroPathGPSOnGPU,lengthOfMacroPathOnGPU*sizeof(PathPointInGPScords)));
	CUDA_ERROR_CHECK_FUNCTIONS::CUDA_CHECK_RETURN(cudaMalloc((void**)&macroPathXYOnGPU,lengthOfMacroPathOnGPU*sizeof(PathPointInLocalXY)));

	CUDA_ERROR_CHECK_FUNCTIONS::CUDA_CHECK_RETURN(cudaMemset(macroPathXYOnGPU,0,lengthOfMacroPathOnGPU*sizeof(PathPointInLocalXY)));
	CUDA_ERROR_CHECK_FUNCTIONS::CUDA_CHECK_RETURN(cudaMemcpy(macroPathGPSOnGPU,macroPathGPS+(currentIndexInMacroPath-1),lengthOfMacroPathOnGPU*sizeof(PathPointInGPScords),cudaMemcpyHostToDevice));

	int numberOfKernelBlocks = lengthOfMacroPathOnGPU/32+1;
	translatePathToXY<<<numberOfKernelBlocks,32>>>(macroPathGPSOnGPU,macroPathXYOnGPU,lengthOfMacroPathOnGPU,vehiclePosition);

	CUDA_ERROR_CHECK_FUNCTIONS::CUDA_CHECK_RETURN(cudaMemcpy(macroPathXY+(currentIndexInMacroPath-1),macroPathXYOnGPU,lengthOfMacroPathOnGPU*sizeof(PathPointInLocalXY),cudaMemcpyDeviceToHost));
	CUDA_ERROR_CHECK_FUNCTIONS::CUDA_CHECK_RETURN(cudaFree(macroPathGPSOnGPU));
	CUDA_ERROR_CHECK_FUNCTIONS::CUDA_CHECK_RETURN(cudaFree(macroPathXYOnGPU));
}

bool PathPlanning::translateMicroPathToXYandCheckIfMicroPathIsTooCloseToObstacles() const {
	/* Translates all the coordinates in microPathGPS to local x,y and stores them is microPathInLocalXY.
	 * After that the function checks if any point in microPathInLocalXY is too close to any obstacle
	 * defined in obstacleSquaresOnGPU, and if so returns true
	 *
	 * When translating the paths, the function only copy the points in the path that the vehicle hasn't
	 * already passed
	 */

	bool nodeIsColliding=false;

	int lengthOfMicroPathOnGPU = lengthOfMicroPath - (currentIndexInMicroPath-1);
	// Translate all the points remaining on the microPath, as well as the point closest behind the vehicle

	PathPointInGPScords* microPathGPSOnGPU;
	PathPointInLocalXY* microPathXYOnGPU;

	CUDA_ERROR_CHECK_FUNCTIONS::CUDA_CHECK_RETURN(cudaMalloc((void**)&microPathGPSOnGPU,lengthOfMicroPathOnGPU*sizeof(PathPointInGPScords)));
	CUDA_ERROR_CHECK_FUNCTIONS::CUDA_CHECK_RETURN(cudaMalloc((void**)&microPathXYOnGPU,lengthOfMicroPathOnGPU*sizeof(PathPointInGPScords)));

	CUDA_ERROR_CHECK_FUNCTIONS::CUDA_CHECK_RETURN(cudaMemcpyToSymbol(nodeIsCollidingOnGPU,&nodeIsColliding,sizeof(bool),0,cudaMemcpyHostToDevice));
	CUDA_ERROR_CHECK_FUNCTIONS::CUDA_CHECK_RETURN(cudaMemset(microPathXYOnGPU,0,lengthOfMicroPathOnGPU*sizeof(PathPointInLocalXY)));
	CUDA_ERROR_CHECK_FUNCTIONS::CUDA_CHECK_RETURN(cudaMemcpy(microPathGPSOnGPU,microPathGPS+(currentIndexInMicroPath-1),lengthOfMicroPathOnGPU*sizeof(PathPointInGPScords),cudaMemcpyHostToDevice));

	int numberOfKernelBlocks = lengthOfMicroPathOnGPU/32+1;
	translatePathToXY<<<numberOfKernelBlocks,32>>>(microPathGPSOnGPU,microPathXYOnGPU,lengthOfMicroPathOnGPU,vehiclePosition);

	// Do collision check on microPathXY
	numberOfKernelBlocks = lengthOfMicroPathOnGPU*nrOfVehicleCollisionCheckingPoints/256+1;
	checkPathForCollisions<<<numberOfKernelBlocks,256>>>(microPathGPSOnGPU, microPathXYOnGPU, lengthOfMicroPathOnGPU, lidarExportData.occupancyGridOnGPU, vehicleCollisionCheckingPointsOnGPU, nrOfVehicleCollisionCheckingPoints);

	CUDA_ERROR_CHECK_FUNCTIONS::CUDA_CHECK_RETURN(cudaMemcpyFromSymbol(&nodeIsColliding,nodeIsCollidingOnGPU,sizeof(bool),0,cudaMemcpyDeviceToHost));
	CUDA_ERROR_CHECK_FUNCTIONS::CUDA_CHECK_RETURN(cudaMemcpy(microPathXY+(currentIndexInMicroPath-1),microPathXYOnGPU,lengthOfMicroPathOnGPU*sizeof(PathPointInLocalXY),cudaMemcpyDeviceToHost));
	CUDA_ERROR_CHECK_FUNCTIONS::CUDA_CHECK_RETURN(cudaFree(microPathGPSOnGPU));
	CUDA_ERROR_CHECK_FUNCTIONS::CUDA_CHECK_RETURN(cudaFree(microPathXYOnGPU));

	return nodeIsColliding;
}

void PathPlanning::loadNewMacroPath() {
	// Reset paths first
	clearAllPaths(true);
	loadNewMacroPathFlag=false;

	// This function is called by Input when a new path is loaded. The filepath to the pathfile is the parameter
	FILE *fp;
	char line[60];

	if ((fp = fopen(macroPathFilePath,"rt"))  == NULL) {
		printf("%s %s %s\n","Unable to open path file:",macroPathFilePath, strerror(errno));
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
	macroPathGPS[0].position.latc = vehiclePosition.currentPosition.latc;
	macroPathGPS[0].position.longc = vehiclePosition.currentPosition.longc;

	for (int i=1;i<lengthOfMacroPath;i++) {
		if (fgets(line,60,fp)==0) {
			printf("%s\n","Error: not all coordinates in path are provided in path file");
			clearAllPaths(true);
			return;
		}
		sscanf(line,"%lf %lf\n",&macroPathGPS[i].position.latc,&macroPathGPS[i].position.longc);
		macroPathGPS[i].courseFromPreviousPathPoint = getCourseBetweenGPSpositions(macroPathGPS[i-1].position,macroPathGPS[i].position);
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

void PathPlanning::initializeCollisionCheckingPoints() {
	// Initialize the collision checking points:
	int index=0;
	int nrPointsWidhtWise = round(PARAMETERS::RCV_WIDTH / (2*PARAMETERS::OCCUPANCY_GRID_CELL_SIZE));
	int nrPointsLengthWise = round(PARAMETERS::RCV_LENGTH / (2*PARAMETERS::OCCUPANCY_GRID_CELL_SIZE));

	nrOfVehicleCollisionCheckingPoints = nrPointsWidhtWise * nrPointsLengthWise;
	vehicleCollisionCheckCoordinate* vehicleCollisionCheckingPoints = new vehicleCollisionCheckCoordinate[nrOfVehicleCollisionCheckingPoints];

	float currentX,currentY;
	currentX = 1.5*PARAMETERS::OCCUPANCY_GRID_CELL_SIZE - PARAMETERS::RCV_WIDTH/2.0;
	for (int i=0;i<nrPointsWidhtWise;i++) {
		currentY = 1.5*PARAMETERS::OCCUPANCY_GRID_CELL_SIZE - PARAMETERS::RCV_LENGTH/2.0;
		for (int j=0;j<nrPointsLengthWise;j++) {
			vehicleCollisionCheckingPoints[index].x = currentX;
			vehicleCollisionCheckingPoints[index].y = currentY;
			index++;
			currentY+=2*PARAMETERS::OCCUPANCY_GRID_CELL_SIZE;
		}
		currentX+=2*PARAMETERS::OCCUPANCY_GRID_CELL_SIZE;
	}

	CUDA_ERROR_CHECK_FUNCTIONS::CUDA_CHECK_RETURN(cudaMalloc((void**)&vehicleCollisionCheckingPointsOnGPU,nrOfVehicleCollisionCheckingPoints*sizeof(vehicleCollisionCheckCoordinate)));
	CUDA_ERROR_CHECK_FUNCTIONS::CUDA_CHECK_RETURN(cudaMemcpy(vehicleCollisionCheckingPointsOnGPU, vehicleCollisionCheckingPoints, nrOfVehicleCollisionCheckingPoints*sizeof(vehicleCollisionCheckCoordinate), cudaMemcpyHostToDevice));

	delete[] vehicleCollisionCheckingPoints;
}

double PathPlanning::getTargetHeadingForMicroPath() {
//	if (currentIndexInMacroPath==lengthOfMacroPath-1) {return macroPathGPS[currentIndexInMacroPath].courseFromPreviousPathPoint;} // TODO, change to local heading
//	return macroPathGPS[currentIndexInMacroPath+1].courseFromPreviousPathPoint;
	return 0; // TODO
}

int iter; // TODO
bool PathPlanning::generateMicroPath(const float targetX, const float targetY, const double targetHeading) {
	/* This function generates a path from {0,0} to targetX,targetY using a variant of A* tailored for producing vehicle paths
	 * returns true if a path was found, and false otherwise
	 */

	// First delete the current microGPSpath:
	clearAllPaths(false);
	hashTable.clearHashTable();
	minHeap.clearMinHeap();

	float baseNodeDistanceToTarget, baseNodeDeltaCourseToTarget;
	int timePassedSinceStartOfLoop=0;
	timeval startTime,currentTime;
	gettimeofday(&startTime,NULL);

	aStarNode* baseNode = &hashTable.addAstarNode(0,-1,0,0,false),* goalNode;
	iter=0;

	while (true) {
		baseNode->isOnClosedSet=true;
		baseNode->isOnOpenSet=false;

		iter++;

		// Check if baseNode is close enough to the target:
		baseNodeDistanceToTarget = sqrt((baseNode->x-targetX)*(baseNode->x-targetX)+(baseNode->y-targetY)*(baseNode->y-targetY));
		baseNodeDeltaCourseToTarget = atan2(targetX-baseNode->x,targetY-baseNode->y) - baseNode->localVehicleHeadingAtNode;
		if (baseNodeDistanceToTarget < 10*PARAMETERS::OCCUPANCY_GRID_CELL_SIZE && (abs(baseNodeDeltaCourseToTarget) < 0.1 || abs(abs(baseNodeDeltaCourseToTarget)-M_PI) < 0.1)) {
			bool isReversing = !(abs(baseNodeDeltaCourseToTarget) < 0.1);
			goalNode = &hashTable.addAstarNode(baseNode->x,baseNode->y,targetX,targetY,isReversing);
			if (!checkIfaStarNodeIsTooCloseToObstacles(*goalNode,baseNode->localVehicleHeadingAtNode+baseNodeDeltaCourseToTarget)) {
				if (baseNode!=goalNode) {
					goalNode->vehicleIsReversingFromPrevNode = isReversing;
					goalNode->previousNodeInPath = baseNode;
					baseNode = goalNode;
				}
				//temp:
				printf("%s%d\n","path found, path planning time:",timePassedSinceStartOfLoop);
				printf("%s%d\n","iters:",iter);
				printf("%s%d\n","minheap av space:",minHeap.getAvailableSpace());
				break;
			}
		}

		for (int i=0;i<10;i++) {discoverNeighbor(*baseNode,targetX,targetY,targetHeading,i);}

		baseNode = minHeap.popNode();

		gettimeofday(&currentTime,NULL);
		timePassedSinceStartOfLoop = (currentTime.tv_sec*1000000 + currentTime.tv_usec) - (startTime.tv_sec*1000000 + startTime.tv_usec);

		// If minheap is full or empty, or if the pathplanner has taken more than 100 msec to find a path, then a path could not be found, so return:
		if (timePassedSinceStartOfLoop > 80000 ||  /*iter==350 ||*/ baseNode==NULL || minHeap.getAvailableSpace() < 10) {
			printf("%s%d\n","path not found, path planning time:",timePassedSinceStartOfLoop);
			printf("%s%d\n","iters:",iter);
			printf("%s%d\n","minheap av space:",minHeap.getAvailableSpace());
			return false;
		}
	}

	// Path found, now create microPathGPS and microPathXY
	const aStarNode* currentNode = baseNode;
	while(true) { // Find number of nodes in the path
		if (currentNode->previousNodeInPath==NULL) {break;}

		currentNode = currentNode->previousNodeInPath;
		lengthOfMicroPath++;
	}

	lengthOfMicroPath++; // Make room for the first position in the path; the vehicles own position
	microPathGPS = new PathPointInGPScords[lengthOfMicroPath]();
	microPathXY = new PathPointInLocalXY[lengthOfMicroPath]();

	// Add the start node to the first position in the path
	microPathGPS[0].position=vehiclePosition.currentPosition;
	microPathXY[0].x=0;
	microPathXY[0].y=0;

	currentNode = baseNode;
	for (int i=lengthOfMicroPath-1;i>0;i--) {
		/* The vehicle has the y axis pointing forward in the direction of travel. Rotate a point x
		 * and y counter clockwise by heading makes y align with true north, and x align with true east.
		 * Scale those and you get latc and longc
		 */
		microPathGPS[i].position.latc = vehiclePosition.currentPosition.latc + rotateAndGetY(currentNode->x,currentNode->y,vehiclePosition.currentHeading) * (1.0/LENGTH_OF_ONE_LAT_DEGREE_IN_METERS);
		microPathGPS[i].position.longc = vehiclePosition.currentPosition.longc + rotateAndGetX(currentNode->x,currentNode->y,vehiclePosition.currentHeading) * (1.0/LENGTH_OF_ONE_LONG_DEGREE_IN_METERS);
		microPathGPS[i].courseFromPreviousPathPoint = atan2f(currentNode->x-currentNode->previousNodeInPath->x,currentNode->y-currentNode->previousNodeInPath->y) + vehiclePosition.currentHeading;
		microPathGPS[i].latDistanceFromPrevPathPoint = rotateAndGetY(currentNode->x-currentNode->previousNodeInPath->x,currentNode->y-currentNode->previousNodeInPath->y,vehiclePosition.currentHeading);
		microPathGPS[i].longDistanceFromPrevPathPoint = rotateAndGetX(currentNode->x-currentNode->previousNodeInPath->x,currentNode->y-currentNode->previousNodeInPath->y,vehiclePosition.currentHeading);
		microPathGPS[i].isReversingFromPrevNode = currentNode->vehicleIsReversingFromPrevNode;

		microPathXY[i].x = currentNode->x;
		microPathXY[i].y = currentNode->y;
		if (currentNode->vehicleIsReversingFromPrevNode) {microPathXY[i].r=255;} // Set colors of the path; red for reversing, green otherwise
		else {microPathXY[i].g=255;}

		// Path debugging:
//		if (!currentNode->vehicleIsReversingFromPrevNode) {printf("%s%f%s%f%s%f%s%f%s\n","quiver(",currentNode->x,",",currentNode->y,",",currentNode->previousNodeInPath->x-currentNode->x,",",currentNode->previousNodeInPath->y-currentNode->y,",0,'g')");}
//		else {printf("%s%f%s%f%s%f%s%f%s\n","quiver(",currentNode->x,",",currentNode->y,",",currentNode->previousNodeInPath->x-currentNode->x,",",currentNode->previousNodeInPath->y-currentNode->y,",0,'r')");}

		currentNode=currentNode->previousNodeInPath;
	}

	currentIndexInMicroPath=1;
	/* Just like in setMacroPath(), the first position in microPathGPS is the vehicles current position, so
	 * therefore set currentIndexInMicroPath to 1, in order to make the vehicle start heading for the position
	 * in microPath that comes after its own position
	 */

	return true;
}


void PathPlanning::discoverNeighbor(aStarNode& baseNode, const float targetX, const float targetY, const double targetHeading, const int index) {
	/* This function discovers a neighbor node to baseNode, looking in 8 directions forward, and 8 directions backward
	 * First time it is called (when baseNode==startNode), baseNode->HeadingFromPreviousNodeInPath = 0
	 */
	float distanceFromStartNode, neighborHeuristic, stepDistance, distanceToTarget;  // The stepping distance between each point in the path
	double deltaAngle,newAngle,localCourseToTarget,localVehicleHeadingAtNode;
	bool vehicleIsReversingFromBaseNode;
	switch(index) {
		// Keep going in the same direction:
		case 0: {deltaAngle=0;stepDistance=PARAMETERS::OCCUPANCY_GRID_CELL_SIZE*5;break;}
		case 1: {deltaAngle=0.0515;stepDistance=PARAMETERS::OCCUPANCY_GRID_CELL_SIZE*20;break;}
		case 2: {deltaAngle=0.103;stepDistance=PARAMETERS::OCCUPANCY_GRID_CELL_SIZE*10;break;}
		case 3: {deltaAngle=-0.0515;stepDistance=PARAMETERS::OCCUPANCY_GRID_CELL_SIZE*20;break;}
		case 4: {deltaAngle=-0.103;stepDistance=PARAMETERS::OCCUPANCY_GRID_CELL_SIZE*10;break;}
		// Switch direction (going from driving forwards to reversing, or vice versa)
		case 5: {deltaAngle=0+M_PI;stepDistance=PARAMETERS::OCCUPANCY_GRID_CELL_SIZE*5;break;}
		case 6: {deltaAngle=0.0515+M_PI;stepDistance=PARAMETERS::OCCUPANCY_GRID_CELL_SIZE*20;break;}
		case 7: {deltaAngle=0.103+M_PI;stepDistance=PARAMETERS::OCCUPANCY_GRID_CELL_SIZE*10;break;}
		case 8: {deltaAngle=-0.0515+M_PI;stepDistance=PARAMETERS::OCCUPANCY_GRID_CELL_SIZE*20;break;}
		case 9: {deltaAngle=-0.103+M_PI;stepDistance=PARAMETERS::OCCUPANCY_GRID_CELL_SIZE*10;break;}
	}
	newAngle = baseNode.localVehicleHeadingAtNode + (baseNode.vehicleIsReversingFromPrevNode ? M_PI : 0) + deltaAngle;
	vehicleIsReversingFromBaseNode = ((baseNode.vehicleIsReversingFromPrevNode && index<5) || (!baseNode.vehicleIsReversingFromPrevNode && index>4));

	// Adds the new node to the hashtable, or gets it if it's already there. The y-axis goes along the vehicle longitudinal centerline
	aStarNode& neighborNode = hashTable.addAstarNode(baseNode.x,baseNode.y,baseNode.x+stepDistance*sin(newAngle),baseNode.y+stepDistance*cos(newAngle),vehicleIsReversingFromBaseNode);

	localVehicleHeadingAtNode = atan2(neighborNode.x-baseNode.x,neighborNode.y-baseNode.y);
	localVehicleHeadingAtNode += (vehicleIsReversingFromBaseNode ? M_PI : 0);
	if (localVehicleHeadingAtNode > M_PI) {localVehicleHeadingAtNode-=2*M_PI;}

	if (neighborNode.isOnClosedSet || checkIfaStarNodeIsTooCloseToObstacles(neighborNode,localVehicleHeadingAtNode)) {return;}
	// If neigborNode is on closed set, or if it is too close to an obstacle, ignore it and return

	distanceFromStartNode = baseNode.distanceFromStartNode + sqrt((baseNode.x-neighborNode.x)*(baseNode.x-neighborNode.x)+(baseNode.y-neighborNode.y)*(baseNode.y-neighborNode.y));
	distanceToTarget = sqrt((neighborNode.x-targetX)*(neighborNode.x-targetX)+(neighborNode.y-targetY)*(neighborNode.y-targetY));
	localCourseToTarget = atan2(targetX-neighborNode.x,targetY-neighborNode.y);

	neighborHeuristic = distanceFromStartNode;
	neighborHeuristic += distanceToTarget;
	neighborHeuristic += 6*abs(localCourseToTarget - localVehicleHeadingAtNode);
//	neighborHeuristic += 6*abs(targetHeading - vehicleHeadingAtNode); // Make the path align with targetheading
	neighborHeuristic += (index>4) ? 1 : 0; // Punish switching direction

	if (!neighborNode.isOnOpenSet) {
		// Node has not been discovered yet
		neighborNode.isOnOpenSet=true;

		neighborNode.localVehicleHeadingAtNode=localVehicleHeadingAtNode;
		neighborNode.distanceFromStartNode = distanceFromStartNode;
		neighborNode.heuristic = neighborHeuristic;
		neighborNode.previousNodeInPath = &baseNode;
		neighborNode.vehicleIsReversingFromPrevNode=vehicleIsReversingFromBaseNode;

		minHeap.addNode(neighborNode);

		// Path debugging:
//		if (neighborNode.vehicleIsReversingFromPrevNode) {printf("%s%f%s%f%s%f%s%f%s\n","plot([",neighborNode.x,",",baseNode.x,"],[",neighborNode.y,",",baseNode.y,"],'r')");}
//		else {printf("%s%f%s%f%s%f%s%f%s\n","plot([",neighborNode.x,",",baseNode.x,"],[",neighborNode.y,",",baseNode.y,"],'g')");}
	}
	else if (distanceFromStartNode < neighborNode.distanceFromStartNode) {
		// Path debugging:
//		printf("%s%f%s%f%s%f%s%f%s\n","plot([",neighborNode.x,",",neighborNode.previousNodeInPath->x,"],[",neighborNode.y,",",neighborNode.previousNodeInPath->y,"],'w')");

		neighborNode.localVehicleHeadingAtNode=localVehicleHeadingAtNode;
		neighborNode.distanceFromStartNode = distanceFromStartNode;
		neighborNode.heuristic = neighborHeuristic;
		neighborNode.previousNodeInPath = &baseNode;
		neighborNode.vehicleIsReversingFromPrevNode=vehicleIsReversingFromBaseNode;

		minHeap.bubbleNode(neighborNode);

		// Path debugging:
//		if (neighborNode.vehicleIsReversingFromPrevNode) {printf("%s%f%s%f%s%f%s%f%s\n","plot([",neighborNode.x,",",baseNode.x,"],[",neighborNode.y,",",baseNode.y,"],'r')");}
//		else {printf("%s%f%s%f%s%f%s%f%s\n","plot([",neighborNode.x,",",baseNode.x,"],[",neighborNode.y,",",baseNode.y,"],'g')");}
	}
}

bool PathPlanning::checkIfaStarNodeIsTooCloseToObstacles(const aStarNode& node, const double localVehicleHeadingAtNode) const {
	// This function checks if node is too close to any obstacle defined in obstacleSquaresOnGPU, and returns true if so
	bool nodeIsColliding=false;
	int numberOfKernelBlocks = nrOfVehicleCollisionCheckingPoints/32+1;

	CUDA_ERROR_CHECK_FUNCTIONS::CUDA_CHECK_RETURN(cudaMemcpyToSymbol(nodeIsCollidingOnGPU,&nodeIsColliding,sizeof(bool),0,cudaMemcpyHostToDevice));

	checkIfNodeIsColliding<<<numberOfKernelBlocks,32>>>(node.x,node.y,localVehicleHeadingAtNode,lidarExportData.occupancyGridOnGPU,vehicleCollisionCheckingPointsOnGPU,nrOfVehicleCollisionCheckingPoints);

	CUDA_ERROR_CHECK_FUNCTIONS::CUDA_CHECK_RETURN(cudaMemcpyFromSymbol(&nodeIsColliding,nodeIsCollidingOnGPU,sizeof(bool),0,cudaMemcpyDeviceToHost));
	return nodeIsColliding;
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

// Hashtable:
PathPlanning::HashTable::HashTable() {
	memset(hashArray,0,sizeof(hashArray)); // Set hashArray to null
}

PathPlanning::HashTable::~HashTable() {
	clearHashTable();
}

int PathPlanning::HashTable::getIndex(int gridX, int gridY, int gridVehicleHeading) const {
	// I tested this hasher in matlab, it is based on Java's hashCode(), and it gives pretty even results with PARAMETERS::OCCUPANCY_GRID_CELL_SIZE
	// between 0.01 and 0.2. X and Y are at this point always aligned with the ground grid

	unsigned int hash = 999*abs(gridX)+998*gridY*gridY+997*abs(gridVehicleHeading)*gridVehicleHeading*gridVehicleHeading;
	return hash % HASH_TABLE_ENTRIES;
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


PathPlanning::aStarNode& PathPlanning::HashTable::addAstarNode(float baseNodeX, float baseNodeY, float newX, float newY, bool vehicleIsReversingToNode) {
	// gridX and gridY are the rounded coordinates that match the ground grid resolution
	int gridX = round(newX/PARAMETERS::OCCUPANCY_GRID_CELL_SIZE),gridY = round(newY/PARAMETERS::OCCUPANCY_GRID_CELL_SIZE),gridVehicleHeading,otherGridX,otherGridY,otherGridVehicleHeading,arrayIndex;
	double localVehicleHeadingAtNode = atan2(gridX*PARAMETERS::OCCUPANCY_GRID_CELL_SIZE-baseNodeX,gridY*PARAMETERS::OCCUPANCY_GRID_CELL_SIZE-baseNodeY);
	localVehicleHeadingAtNode += (vehicleIsReversingToNode ? M_PI : 0);
	if (localVehicleHeadingAtNode > M_PI) {localVehicleHeadingAtNode-=2*M_PI;}
	/* gridVehicleHeading is the angle going from {the vehicle centerline vector pointing forwards}
	 * to {the vector going from baseNode to neihgborNode}. It is local in the coordinate system of the vehicle
	 */

	gridVehicleHeading = round(localVehicleHeadingAtNode/(2*M_PI/NR_OF_GRID_HEADINGS));
	arrayIndex = getIndex(gridX,gridY,gridVehicleHeading);

	// First check if node already is in the hashtable:
	HashBucket* bucketPointer = hashArray[arrayIndex];
	while(true) {
		if (bucketPointer==NULL) {break;} //No bucket was found matching x and y

		otherGridX = round(bucketPointer->node.x/PARAMETERS::OCCUPANCY_GRID_CELL_SIZE);
		otherGridY = round(bucketPointer->node.y/PARAMETERS::OCCUPANCY_GRID_CELL_SIZE);
		otherGridVehicleHeading = round(bucketPointer->node.localVehicleHeadingAtNode/(2*M_PI/NR_OF_GRID_HEADINGS));
		if (gridX == otherGridX && gridY == otherGridY && gridVehicleHeading == otherGridVehicleHeading) {return bucketPointer->node;}

		bucketPointer=bucketPointer->nextBucket;
	}
	// No match was found in the hashtable; add new node

	bucketPointer=hashArray[arrayIndex];
	if (bucketPointer==NULL) {
		hashArray[arrayIndex]=new HashBucket(gridX*PARAMETERS::OCCUPANCY_GRID_CELL_SIZE,gridY*PARAMETERS::OCCUPANCY_GRID_CELL_SIZE); //Everything (including all the fields of the aStarNode) set to zero
		bucketPointer=hashArray[arrayIndex];
	}
	else {
		while (bucketPointer->nextBucket!=NULL) {bucketPointer = bucketPointer->nextBucket;} // Get the latest bucket in the linked list for the specified index:
		bucketPointer->nextBucket = new HashBucket(gridX*PARAMETERS::OCCUPANCY_GRID_CELL_SIZE,gridY*PARAMETERS::OCCUPANCY_GRID_CELL_SIZE);
		bucketPointer = bucketPointer->nextBucket;
	}

	return bucketPointer->node;
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

void PathPlanning::MinHeap::addNode(aStarNode& node) {
	if (currentNrOfNodesInHeap==HEAP_SIZE) {return;} // Return if heap is full; meaning that no path can be generated

	heapArray[currentNrOfNodesInHeap] = &node;
	node.heapArrayIndex = currentNrOfNodesInHeap;

	bubbleNode(node);

	currentNrOfNodesInHeap++;
}

void PathPlanning::MinHeap::bubbleNode(aStarNode& node) {
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

PathPlanning::aStarNode* PathPlanning::MinHeap::popNode() {
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

namespace {
// Non member and cuda functions implementations:
	__device__ bool checkOccupancyGridForCollision(float nodeX, float nodeY, float localVehicleHeadingAtNode, PathPlanning::vehicleCollisionCheckCoordinate vehicleCoordinate, const char* occupancyGridOnGPU) {
		float myX = vehicleCoordinate.x;
		float myY = vehicleCoordinate.y;
		float myTranslatedX = myX * cosf(localVehicleHeadingAtNode) - myY * sinf(localVehicleHeadingAtNode);
		float myTranslatedY = myX * sinf(localVehicleHeadingAtNode) + myY * cosf(localVehicleHeadingAtNode);
		myTranslatedX += nodeX;
		myTranslatedY += nodeY;

		int myMatrixXcoord = myTranslatedX/PARAMETERS::OCCUPANCY_GRID_CELL_SIZE - PARAMETERS::NR_OCCUPANCY_GRID_CELLS_X_WISE/2.0; // Integer division
		int myMatrixYcoord = myTranslatedY/PARAMETERS::OCCUPANCY_GRID_CELL_SIZE - PARAMETERS::NR_OCCUPANCY_GRID_CELLS_Y_WISE/2.0;

		if (myMatrixXcoord >= 0 && myMatrixYcoord >= 0
			&& myMatrixXcoord < PARAMETERS::NR_OCCUPANCY_GRID_CELLS_X_WISE && myMatrixYcoord < PARAMETERS::NR_OCCUPANCY_GRID_CELLS_Y_WISE
			&& occupancyGridOnGPU[myMatrixXcoord * PARAMETERS::NR_OCCUPANCY_GRID_CELLS_Y_WISE + myMatrixYcoord]>0) {
			// This cell is occupied by an obstacle
			return true;
		}
		return false;
	}

	__global__ void translatePathToXY(PathPointInGPScords* pathGPSOnGPU, PathPointInLocalXY* pathXYonGPU, int lengthOfPath, VehiclePosition vehiclePosition) {
		int myThreadID = blockIdx.x*blockDim.x+threadIdx.x;
		float scaledX,scaledY,rotatedX,rotatedY;

		if (myThreadID < lengthOfPath) {
			scaledX = (pathGPSOnGPU[myThreadID].position.longc - vehiclePosition.currentPosition.longc) * LENGTH_OF_ONE_LONG_DEGREE_IN_METERS;
			scaledY = (pathGPSOnGPU[myThreadID].position.latc - vehiclePosition.currentPosition.latc) * LENGTH_OF_ONE_LAT_DEGREE_IN_METERS;
			rotatedX = scaledX * cosf(vehiclePosition.currentHeading) - scaledY * sinf(vehiclePosition.currentHeading);
			rotatedY = scaledX * sinf(vehiclePosition.currentHeading) + scaledY * cosf(vehiclePosition.currentHeading);
			pathXYonGPU[myThreadID].x=rotatedX;
			pathXYonGPU[myThreadID].y=rotatedY;

			if (pathGPSOnGPU[myThreadID].isReversingFromPrevNode) {pathXYonGPU[myThreadID].r=255;}
			else {pathXYonGPU[myThreadID].g=255;}
		}
	}

	__global__ void checkPathForCollisions(PathPointInGPScords* pathGPSOnGPU, PathPointInLocalXY* pathXYonGPU, int lengthOfPath, const char* occupancyGridOnGPU, const PathPlanning::vehicleCollisionCheckCoordinate* vehicleCollisionCheckingPointsOnGPU, int nrOfVehicleCollisionCheckingPoints) {
		/* Think of the function as a matrix, where nodes are the rows, and obstacles are the columns. For each node/obstacle pair, see if
		 * their distance is at least the min distance. To get the row and column index, row = index/columnSize, column = index - row*columnSize
		 */
		int myThreadID = blockIdx.x*blockDim.x+threadIdx.x, myPathIndex = myThreadID/nrOfVehicleCollisionCheckingPoints, myVehicleCollisionCheckPointIndex = myThreadID-myPathIndex*nrOfVehicleCollisionCheckingPoints;
		float pathPointX,pathPointY,localVehicleHeadingAtNode;

		if (myThreadID < lengthOfPath*nrOfVehicleCollisionCheckingPoints && myPathIndex > 0) { // myPathIndex>0 so the function doesn't collision check microPath[currentMicroPathIndex]. That point is behind the vehicle, and should not be checked
			// The positions of the nodes and obstacles are the center distances of their respective square
			pathPointX = pathXYonGPU[myPathIndex].x;
			pathPointY = pathXYonGPU[myPathIndex].y;
			localVehicleHeadingAtNode = atan2f(pathPointX-pathXYonGPU[myPathIndex-1].x,pathPointY-pathXYonGPU[myPathIndex-1].y);

			if(checkOccupancyGridForCollision(pathPointX,pathPointY,localVehicleHeadingAtNode,vehicleCollisionCheckingPointsOnGPU[myVehicleCollisionCheckPointIndex],occupancyGridOnGPU)) {
				nodeIsCollidingOnGPU=true;
			}
		}
	}

	__global__ void checkIfNodeIsColliding(float nodeX, float nodeY, float localVehicleHeadingAtNode, const char* occupancyGridOnGPU, const PathPlanning::vehicleCollisionCheckCoordinate* vehicleCollisionCheckingPointsOnGPU, int nrOfVehicleCollisionCheckingPoints) {
		// This function checks
		int myThreadID = blockIdx.x*blockDim.x+threadIdx.x;

		if (myThreadID < nrOfVehicleCollisionCheckingPoints && checkOccupancyGridForCollision(nodeX,nodeY,localVehicleHeadingAtNode,vehicleCollisionCheckingPointsOnGPU[myThreadID],occupancyGridOnGPU)) {
			nodeIsCollidingOnGPU=true;
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

	double getCourseBetweenGPSpositions(GPSposition fromPosition, GPSposition toPosition) {
		// Returns the heading from fromPosition to toPosition in radians (i.e. the angle from NORTH to the line that goes through the two GPSpositions)

		float deltaLat = getLatDistanceBetweenGPSpositions(fromPosition,toPosition);
		float deltaLong = getLongDistanceBetweenGPSpositions(fromPosition,toPosition);

		return atan2(deltaLong,deltaLat);
	}
}



//	__device__ bool checkIfPathPointIsTooCloseToObstacle(float pathPointX, float pathPointY, double localVehicleHeadingAtNode,float obstacleX, float obstacleY) {
//		// This function returns true if the obstacle is to close to the vehicle, otherwise returns false
//		float rotatedObstacleX,rotatedObstacleY,minDistanceToObstacle;
//		//TODO explanation of how this function works
//
//		/* Observe that because that the x and y positions of the nodes and obstacles are aligned with the
//		 * ground grid, they have an error margin of +/- PARAMETERS::OCCUPANCY_GRID_CELL_SIZE. Therefore OBSTACLE_SAFETY_DISTANCE
//		 * should never be less than PARAMETERS::OCCUPANCY_GRID_CELL_SIZE
//		 */
//		rotatedObstacleX = abs((obstacleX - pathPointX) * cos(localVehicleHeadingAtNode) - (obstacleY - pathPointY) * sin(localVehicleHeadingAtNode));
//		rotatedObstacleY = abs((obstacleX - pathPointX) * sin(localVehicleHeadingAtNode) + (obstacleY - pathPointY) * cos(localVehicleHeadingAtNode));
//
//		if (rotatedObstacleX>RCV_WIDTH/2.0 && rotatedObstacleY>RCV_LENGTH/2.0) {
//			// minDistanceToObstacle is defined by the distance from {RCV_WIDTH/2.0,RCV_LENGHT/2.0} to the obstacle point
//			minDistanceToObstacle = sqrt((rotatedObstacleX-RCV_WIDTH/2.0)*(rotatedObstacleX-RCV_WIDTH/2.0)+(rotatedObstacleY-RCV_HEIGHT/2.0)*(rotatedObstacleY-RCV_HEIGHT/2.0));
//		}
//		else if (rotatedObstacleX > RCV_WIDTH/2.0) {minDistanceToObstacle = rotatedObstacleX - RCV_WIDTH/2.0;}
//		else if (rotatedObstacleY > RCV_LENGTH/2.0) {minDistanceToObstacle = rotatedObstacleY - RCV_LENGTH/2.0;}
//		else {minDistanceToObstacle = -1;} // Obstacle is inside the vehicle
//
//		if (minDistanceToObstacle < OBSTACLE_SAFETY_DISTANCE) {
////			double lvhan=localVehicleHeadingAtNode;
////			float ppx = pathPointX,ppy=pathPointY,ox=obstacleX,oy=obstacleY;
//			return true;
//		}
//		return false;
//	}

//	__global__ void checkIfaStarNodeIsTooCloseToObstaclesKernel(float nodeX, float nodeY, double localVehicleHeadingAtNode, const ObstaclePoint* obstacleSquaresOnGPU, int nrOfObstacles) {
//		int myThreadID = blockIdx.x*blockDim.x+threadIdx.x;
//		float obstacleX,obstacleY;
//
//		if (myThreadID < nrOfObstacles) {
//			obstacleX = (obstacleSquaresOnGPU + 4*myThreadID)->x +PARAMETERS::OCCUPANCY_GRID_CELL_SIZE/2.0;
//			obstacleY = (obstacleSquaresOnGPU + 4*myThreadID)->y +PARAMETERS::OCCUPANCY_GRID_CELL_SIZE/2.0;
//
//			if(checkIfPathPointIsTooCloseToObstacle(nodeX,nodeY,localVehicleHeadingAtNode,obstacleX,obstacleY)) {nodeIsCollidingOnGPU=true;}
//		}
//	}
