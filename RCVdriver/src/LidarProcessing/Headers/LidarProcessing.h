#ifndef LIDARPROCESSING_H_
#define LIDARPROCESSING_H_

#include "../../Headers/Structs.h"
#include "LidarUDPReceiver.h"

class LidarProcessing {
	//Pointers to device and host memory:
	int sizeOfRawLidarData,sizeOfLidarDataPoints,sizeOfObstacleSquares,sizeOfObstacleMatrix;
	char *rawLidarData, *rawLidarDataOnGPU;
	LidarDataPoint *lidarDataPoints, *lidarDataPointsOnGPU;
	ObstaclePoint *obstacleSquares, *obstacleSquaresOnGPU;
	int *obstacleMatrixForMaxZOnGPU, *obstacleMatrixForMinZOnGPU;

	// Members:
	LidarUDPReceiver* lidarUDPReceiver;
	int currentNrOfObstacles;

	void allocateMemory();
	void freeMemory() const;
	void translateLidarDataFromRawToXYZ();
	void identifyObstaclesInLidarData();

	public:
		LidarProcessing();
		~LidarProcessing();
		void processLidarData();
		const LidarDataPoint* getLidarDataPoints() const;
		const ObstaclePoint* getObstacleSquares() const;
		const ObstaclePoint* getObstacleSquaresOnGPU() const;
		const int& getCurrentNrOfObstacles() const;
};


#endif /* LIDARPROCESSING_H_ */
