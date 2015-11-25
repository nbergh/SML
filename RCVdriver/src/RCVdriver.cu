/*
 ============================================================================
 Name        : RCVdriver.cu
 Author      : Niklas Bergh
 Version     : 1.0
 Description : The driver software for the KTH RCV
 License	 : GPL
 ============================================================================
 */

#include <unistd.h>

#include "LidarProcessing/Headers/LidarProcessing.h"
#include "PositionEstimation/Headers/PositionEstimation.h"
#include "Headers/PathPlanning.h"
#include "Headers/Input.h"
#include "Headers/Graphics.h"
#include "Headers/Parameters.h"

int main(void)
{
	//Temp:
	VehicleStatus vehicleStatus = {0};

	// First initialize the main classes:
	LidarProcessing* lidarProcessing = new LidarProcessing();
	PositionEstimation* positionEstimation = new PositionEstimation();
	PathPlanning* pathPlanning = new PathPlanning(lidarProcessing->getObstacleSquaresOnGPU(),lidarProcessing->getCurrentNrOfObstacles(),positionEstimation->getCurrentVehicleState(),vehicleStatus);
	Input* input = new Input(vehicleStatus,*pathPlanning);
	Graphics* graphics = new Graphics();

	while (!input->getStopMainControllerLoop()) {
		// runMainControllerLoop controller loop
		lidarProcessing->processLidarData(); // Process the lidar data from the sensors
		pathPlanning->updatePathAndControlSignals(); // Update the path and send control signals
		sleep(1); // TODO make dynamic
	}

	delete graphics;
	delete input;
	delete pathPlanning;
	delete positionEstimation;
	delete lidarProcessing;
}

