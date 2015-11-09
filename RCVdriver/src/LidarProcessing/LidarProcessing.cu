#include "Headers/LidarProcessing.h"
#include "Headers/LidarCudaFunctions.h"
#include "../Headers/CudaErrorCheckFunctions.h"


#define LIDAR_UDP_PORT 2368 // The port that receives lidar data
#define MAX_OBSTACLE_DETECTION_DISTANCE 20 // The maximal distance (in meters) that an obstacle can be identified in
#define OBSTACLE_POINT_SIDE_LENGTH 0.05 // The length of one side in the square that represents an obstacle point (decreasing this value by a factor of X will require X² times more device memory)
#define MIN_OBSTACLE_DELTA_Z 0.1 // The difference in z coordinates (in meters) required for two points with the same x and y coordinates to be registered as an obstacle
#define MAX_NUMBER_OF_OBSTACLES 5000 //The maximal number of obstacle points that can be displayed

LidarProcessing::LidarProcessing() {
	// First allocate all data needed by this class:
	currentNrOfObstacles=0;
	lidarMemoryPointers = new LidarMemoryPointers();
	allocateMemory();

	// Then initialize the UDP socket, test the connection, and start the UDP receiver thread:
	lidarUDPReceiver = new LidarUDPReceiver(LIDAR_UDP_PORT);
	lidarUDPReceiver->startReceiverThread(lidarMemoryPointers->rawLidarData);
}

LidarProcessing::~LidarProcessing() {
	delete lidarUDPReceiver;
	freeMemory();
	delete lidarMemoryPointers;
}

void LidarProcessing::allocateMemory() {
	/* The size of the buffer array is 90000 bytes which is enough space for 75 packets (75*1200=90000).
	 * The VLP-16 will send exactly 754 data packets per second, and if the lidar is spinning at 603.2 rpm,
	 * that means one revolution will make up exactly 75 packets (754/(603.2/60) = 75). We want exactly one
	 * revolution worth of data in the buffer, since more than that means that we will get "old" points from
	 * one revolution back displayed alongside the new points and less than that means that not all points
	 * in one revolution will fit inside the buffer, creating holes when displaying the graphics. 75 packets
	 * contains data for 28800 lidar points
	 */

	// First is rawLidarData. It is a byte array of raw UDP data (minus UDP headers and factory bytes), representing 75 UDP packets (one revolution of the lidar sensor). Size is 1200*75 = 90000 bytes
	lidarMemoryPointers->sizeOfRawLidarData = 90000;
	CUDA_CHECK_RETURN(cudaMallocHost((void**)&lidarMemoryPointers->rawLidarData,lidarMemoryPointers->sizeOfRawLidarData));
	CUDA_CHECK_RETURN(cudaMalloc((void**)&lidarMemoryPointers->rawLidarDataOnGPU,lidarMemoryPointers->sizeOfRawLidarData));

	// Second is the locationLidarData. This is an array of OpenGlvertex structs, that contain values for x,y,z for 28800 points (one revolution)
	lidarMemoryPointers->sizeOfLidarDataPoints = 28800*sizeof(LidarDataPoint);
	CUDA_CHECK_RETURN(cudaMallocHost((void**)&lidarMemoryPointers->lidarDataPoints,lidarMemoryPointers->sizeOfLidarDataPoints));
	CUDA_CHECK_RETURN(cudaMalloc((void**)&lidarMemoryPointers->lidarDataPointsOnGPU,lidarMemoryPointers->sizeOfLidarDataPoints));

	// Third is the data that hold the obstacles. Each obstacle is represented as a square (4 vertices) in the graphics
	// and 6 ints of index data needed by glDrawElements
	lidarMemoryPointers->sizeOfObstacleSquares = MAX_NUMBER_OF_OBSTACLES*4*sizeof(ObstaclePoint);
	CUDA_CHECK_RETURN(cudaMallocHost((void**)&lidarMemoryPointers->obstacleSquares,lidarMemoryPointers->sizeOfObstacleSquares));
	CUDA_CHECK_RETURN(cudaMalloc((void**)&lidarMemoryPointers->obstacleSquaresOnGPU,lidarMemoryPointers->sizeOfObstacleSquares));

	// Allocate the two obstacle matrices on the device
	lidarMemoryPointers->numberOfMatrixFieldsPerSide = 2 * MAX_OBSTACLE_DETECTION_DISTANCE / OBSTACLE_POINT_SIDE_LENGTH; // As an integer
	lidarMemoryPointers->sizeOfObstacleMatrix = lidarMemoryPointers->numberOfMatrixFieldsPerSide * lidarMemoryPointers->numberOfMatrixFieldsPerSide*sizeof(int);
	CUDA_CHECK_RETURN(cudaMalloc((void**)&lidarMemoryPointers->obstacleMatrixForMaxZOnGPU,lidarMemoryPointers->sizeOfObstacleMatrix));
	CUDA_CHECK_RETURN(cudaMalloc((void**)&lidarMemoryPointers->obstacleMatrixForMinZOnGPU,lidarMemoryPointers->sizeOfObstacleMatrix));
}

void LidarProcessing::freeMemory() {
	CUDA_CHECK_RETURN(cudaFreeHost((void*)lidarMemoryPointers->rawLidarData));
	CUDA_CHECK_RETURN(cudaFree(lidarMemoryPointers->rawLidarDataOnGPU));
	CUDA_CHECK_RETURN(cudaFreeHost((void*)lidarMemoryPointers->lidarDataPoints));
	CUDA_CHECK_RETURN(cudaFree(lidarMemoryPointers->lidarDataPointsOnGPU));
	CUDA_CHECK_RETURN(cudaFreeHost((void*)lidarMemoryPointers->obstacleSquares));
	CUDA_CHECK_RETURN(cudaFree(lidarMemoryPointers->obstacleSquaresOnGPU));
	CUDA_CHECK_RETURN(cudaFree(lidarMemoryPointers->obstacleMatrixForMaxZOnGPU));
	CUDA_CHECK_RETURN(cudaFree(lidarMemoryPointers->obstacleMatrixForMinZOnGPU));
}

void LidarProcessing::processLidarData() {
	translateLidarDataFromRawToXYZ(lidarMemoryPointers);
	currentNrOfObstacles = identifyObstaclesInLidarData(lidarMemoryPointers,OBSTACLE_POINT_SIDE_LENGTH,MIN_OBSTACLE_DELTA_Z,MAX_NUMBER_OF_OBSTACLES);
}

LidarDataPoint* LidarProcessing::getLidarDataPoints() {
	return lidarMemoryPointers->lidarDataPoints;
}

ObstaclePoint* LidarProcessing::getObstacleSquares() {
	return lidarMemoryPointers->obstacleSquares;
}

int* LidarProcessing::getCurrentNrOfObstacles() {
	return &currentNrOfObstacles;
}
