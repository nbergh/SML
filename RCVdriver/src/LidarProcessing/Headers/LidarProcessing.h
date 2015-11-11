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

	void allocateMemory() const;
	void freeMemory() const;

	public:
		LidarProcessing();
		~LidarProcessing();
		void processLidarData();
		const LidarDataPoint* getLidarDataPoints() const;
		const ObstaclePoint* getObstacleSquares() const;
		const int& getCurrentNrOfObstacles() const;
};


#endif /* LIDARPROCESSING_H_ */
