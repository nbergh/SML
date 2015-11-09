#ifndef LIDARPROCESSING_H_
#define LIDARPROCESSING_H_

#include "../../Headers/Structs.h"
#include "LidarUDPReceiver.h"

struct LidarMemoryPointers {
	char *rawLidarData, *rawLidarDataOnGPU; // One rawLidarDataBuffer per lidar unit
	int sizeOfRawLidarData;

	LidarDataPoint *lidarDataPoints, *lidarDataPointsOnGPU;
	int sizeOfLidarDataPoints;

	ObstaclePoint *obstacleSquares, *obstacleSquaresOnGPU;
	int sizeOfObstacleSquares;

	int *obstacleMatrixForMaxZOnGPU, *obstacleMatrixForMinZOnGPU;
	int sizeOfObstacleMatrix,numberOfMatrixFieldsPerSide;
};

class LidarProcessing {
	LidarMemoryPointers* lidarMemoryPointers;
	LidarUDPReceiver* lidarUDPReceiver;
	int currentNrOfObstacles;

	void allocateMemory();
	void freeMemory();

	public:
		LidarProcessing();
		~LidarProcessing();
		void processLidarData();
		LidarDataPoint* getLidarDataPoints();
		ObstaclePoint* getObstacleSquares();
		int* getCurrentNrOfObstacles();
};


#endif /* LIDARPROCESSING_H_ */
