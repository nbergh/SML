/*
 ============================================================================
 Name        : RCVdriver.cu
 Author      : Niklas Bergh
 Version     : 1.0
 Description : The driver software for the KTH RCV
 ============================================================================
 */

#include "LidarProcessing/Headers/LidarProcessing.h"
#include "PositionEstimation/Headers/PositionEstimation.h"
#include "Headers/PathPlanning.h"
#include "Headers/Input.h"
#include "Headers/Graphics.h"
#include "Headers/Parameters.h"

LidarProcessing* lidarProcessing;
PositionEstimation* positionEstimation;
PathPlanning* pathPlanning;
Input* input;

void controllerMainLoopStep() {
	// This function defines everything the RCV should in each iteration in the main controller loop
	lidarProcessing->processLidarData(); // Process the lidar data from the sensors
	pathPlanning->updatePathAndControlSignals();
	// Update the path
	// Send the control signals
}

#include <unistd.h> // temp
#include <stdio.h>
int main(void)
{
	//Temp:
	VehicleStatus vehicleStatus = {0};

	// First initialize the main classes:
	lidarProcessing = new LidarProcessing();
	positionEstimation = new PositionEstimation();
	pathPlanning = new PathPlanning(lidarProcessing->getObstacleSquaresOnGPU(),lidarProcessing->getCurrentNrOfObstacles(),positionEstimation->getCurrentVehicleState(),vehicleStatus);
	input = new Input(vehicleStatus,*pathPlanning);

	//Temp:
	for (int i=0;i<1000;i++) {controllerMainLoopStep();}

	// Set the controllerMainLoopStep to execute periodically:
	sleep(100000);


	//startGraphics(lidarProcessing->getLidarDataPoints(),lidarProcessing->getObstacleSquares(),lidarProcessing->getCurrentNrOfObstacles(),GRAPHICS_UPDATE_RATE);

	delete input;
	delete pathPlanning;
	delete positionEstimation;
	delete lidarProcessing;
}

