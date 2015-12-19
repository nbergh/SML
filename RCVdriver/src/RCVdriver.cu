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

	// Initialize the main objects:
	LidarProcessing lidarProcessing = LidarProcessing();
	PositionEstimation positionEstimation = PositionEstimation();
	PathPlanning pathPlanning = PathPlanning(lidarProcessing.getLidarExportData(),positionEstimation.getCurrentVehiclePosition(),vehicleStatus);
//	Graphics graphics = Graphics(lidarProcessing.getLidarExportData(),pathPlanning.getPathExportData());
	Input input = Input(vehicleStatus,pathPlanning);

	// Start the main controller thread:
	while (!input.getExitProgramBool()) {
		// runMainControllerLoop controller loop
		sleep(1); // TODO make dynamic
		lidarProcessing.processLidarData(); // Process the lidar data from the sensors
		positionEstimation.updatePosition(); // Update the position (VehiclePosition)
		pathPlanning.updatePathAndControlSignals(); // Update the path and send control signals

	}
}

