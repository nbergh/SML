/*
 ============================================================================
 Name        : RCVdriver.cu
 Author      : Niklas Bergh
 Version     : 1.0
 Description : The driver software for the KTH RCV
 ============================================================================
 */

#include "LidarProcessing/Headers/LidarProcessing.h"
#include "Headers/PositionEstimation.h"
#include "Headers/PathPlanning.h"
#include "Headers/Input.h"
#include "Headers/Graphics.h"

#define CONTROLLER_UPDATE_RATE 10 //Hz
#define GRAPHICS_UPDATE_RATE 60 //Hz

LidarProcessing* lidarProcessing;
PositionEstimation* positionEstimation;
PathPlanning* pathPlanning;
Input* input;

void controllerMainLoopStep() {
	// This function defines everything the RCV should in each iteration in the main controller loop

	lidarProcessing->processLidarData(); // Process the lidar data from the sensors
	// Update the path
	// Send the control signals
}

#include <unistd.h> // temp
int main(void)
{
	// First initialize the
	//lidarProcessing = new LidarProcessing();
	//gpsProcessing = new GPSprocessing();
	pathPlanning = new PathPlanning(NULL);//Temp
	//input = new Input(pathPlanning->setMacroPath);

	// Set the controllerMainLoopStep to execute periodically:

	sleep(100000);

	//startGraphics(lidarProcessing->getLidarDataPoints(),lidarProcessing->getObstacleSquares(),lidarProcessing->getCurrentNrOfObstacles(),GRAPHICS_UPDATE_RATE);

	delete lidarProcessing;
	delete positionEstimation;
	delete pathPlanning;
	delete input;
}

