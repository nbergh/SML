/*
 ============================================================================
 Name        : RCVdriver.cu
 Author      : Niklas Bergh
 Version     : 1.0
 Description : The driver software for the KTH RCV
 License	 : GPL
 ============================================================================
 */

#include "LidarProcessing/Headers/LidarProcessing.h"
#include "PositionEstimation/Headers/PositionEstimation.h"
#include "Headers/PathPlanning.h"
#include "Headers/Graphics.h"
#include "Headers/Input.h"
#include "Headers/Parameters.h"

#include <unistd.h>
#include <stdio.h>
#include <sys/time.h>

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

	timeval startTime,endTime;
	int iterationTime=0;

	sleep(1);

	// Start the main controller thread:
	while (!input.getExitProgramBool()) {
		gettimeofday(&startTime,NULL);

		lidarProcessing.processLidarData(); // Process the lidar data from the sensors
		positionEstimation.updatePosition(); // Update the position (VehiclePosition)
		pathPlanning.updatePathAndControlSignals(); // Update the path and send control signals

		gettimeofday(&endTime,NULL);
		iterationTime = (endTime.tv_sec*1000000 + endTime.tv_usec) - (startTime.tv_sec*1000000 + startTime.tv_usec);

//		printf("%s%d\n","itertime: ",iterationTime);
//		sleep(0.1-(iterationTime/1000000.0)); // Run at 10 Hz
	}

	//TODO debug deletion of objects
}

