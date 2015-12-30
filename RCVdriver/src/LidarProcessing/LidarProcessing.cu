#include "Headers/LidarProcessing.h"
#include "../Headers/Parameters.h"
#include "../Headers/CudaErrorCheckFunctions.h"

#include <stdio.h>

namespace {
	//CUDA kernels and fields:
	__device__ int currentNrOfObstaclesOnGPU;
	__global__ void translateLidarDataFromRawToXYZkernel(char* rawLidarDataOnGPU, LidarDataPoint *lidarPointsOnGPU);
	__global__ void writeMaxAndMinZToMatrices(int* obstacleMatrixForMaxZOnGPU,int* obstacleMatrixForMinZOnGPU,const LidarDataPoint* lidarDataPointsOnGPU);
	__global__ void identifyObstacles(const int* obstacleMatrixForMaxZOnGPU,const int* obstacleMatrixForMinZOnGPU,ObstaclePoint* obstacleSquaresOnGPU);
}


LidarProcessing::LidarProcessing() {
	// First allocate all data needed by this class:
	allocateMemory();

	lidarExportData.currentNrOfObstacles=0;
	lidarExportData.lidarDataPoints = lidarDataPoints;
	lidarExportData.obstacleSquares = obstacleSquares;
	lidarExportData.obstacleSquaresOnGPU = obstacleSquaresOnGPU;

	// Then initialize the UDP socket, test the connection, and start the UDP receiver thread:
	lidarUDPReceiver = new LidarUDPReceiver(rawLidarData);
}

LidarProcessing::~LidarProcessing() {
	delete lidarUDPReceiver;
	freeMemory();
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
	sizeOfRawLidarData = 90000;
	CUDA_CHECK_RETURN(cudaMallocHost((void**)&rawLidarData,sizeOfRawLidarData));
	CUDA_CHECK_RETURN(cudaMalloc((void**)&rawLidarDataOnGPU,sizeOfRawLidarData));

	// Second is the locationLidarData. This is an array of OpenGlvertex structs, that contain values for x,y,z for 28800 points (one revolution)
	sizeOfLidarDataPoints = 28800*sizeof(LidarDataPoint);
	CUDA_CHECK_RETURN(cudaMallocHost((void**)&lidarDataPoints,sizeOfLidarDataPoints));
	CUDA_CHECK_RETURN(cudaMalloc((void**)&lidarDataPointsOnGPU,sizeOfLidarDataPoints));

	// Third is the data that hold the obstacles. Each obstacle is represented as a square (4 vertices) in the graphics
	// and 6 ints of index data needed by glDrawElements
	sizeOfObstacleSquares = MAX_NUMBER_OF_OBSTACLES*4*sizeof(ObstaclePoint);
	CUDA_CHECK_RETURN(cudaMallocHost((void**)&obstacleSquares,sizeOfObstacleSquares));
	CUDA_CHECK_RETURN(cudaMalloc((void**)&obstacleSquaresOnGPU,sizeOfObstacleSquares));

	// Allocate the two obstacle matrices on the device
	sizeOfObstacleMatrix = GROUND_GRID_FIELDS_PER_SIDE * GROUND_GRID_FIELDS_PER_SIDE*sizeof(int);
	CUDA_CHECK_RETURN(cudaMalloc((void**)&obstacleMatrixForMaxZOnGPU,sizeOfObstacleMatrix));
	CUDA_CHECK_RETURN(cudaMalloc((void**)&obstacleMatrixForMinZOnGPU,sizeOfObstacleMatrix));
}

void LidarProcessing::freeMemory() const {
	CUDA_CHECK_RETURN(cudaFreeHost((void*)rawLidarData));
	CUDA_CHECK_RETURN(cudaFree(rawLidarDataOnGPU));
	CUDA_CHECK_RETURN(cudaFreeHost((void*)lidarDataPoints));
	CUDA_CHECK_RETURN(cudaFree(lidarDataPointsOnGPU));
	CUDA_CHECK_RETURN(cudaFreeHost((void*)obstacleSquares));
	CUDA_CHECK_RETURN(cudaFree(obstacleSquaresOnGPU));
	CUDA_CHECK_RETURN(cudaFree(obstacleMatrixForMaxZOnGPU));
	CUDA_CHECK_RETURN(cudaFree(obstacleMatrixForMinZOnGPU));
}

void LidarProcessing::processLidarData() {
	translateLidarDataFromRawToXYZ();
	identifyObstaclesInLidarData();

	// Lidar debugging:
//	for (int i=0;i<lidarExportData.currentNrOfObstacles;i++) {
//		printf("%s%f%s%f%s%f%s%f%s\n","plot([",(obstacleSquares+4*i)->x,",",(obstacleSquares+4*i+1)->x,"],[",(obstacleSquares+4*i)->y,",",(obstacleSquares+4*i+1)->y,"],'r')");
//		printf("%s%f%s%f%s%f%s%f%s\n","plot([",(obstacleSquares+4*i)->x,",",(obstacleSquares+4*i+3)->x,"],[",(obstacleSquares+4*i)->y,",",(obstacleSquares+4*i+3)->y,"],'r')");
//		printf("%s%f%s%f%s%f%s%f%s\n","plot([",(obstacleSquares+4*i+1)->x,",",(obstacleSquares+4*i+2)->x,"],[",(obstacleSquares+4*i+1)->y,",",(obstacleSquares+4*i+2)->y,"],'r')");
//		printf("%s%f%s%f%s%f%s%f%s\n","plot([",(obstacleSquares+4*i+3)->x,",",(obstacleSquares+4*i+2)->x,"],[",(obstacleSquares+4*i+3)->y,",",(obstacleSquares+4*i+2)->y,"],'r')");
//	}
}

void LidarProcessing::translateLidarDataFromRawToXYZ() {
	// First copy the raw lidar data from the host to the device
	CUDA_CHECK_RETURN(cudaMemcpy(rawLidarDataOnGPU, rawLidarData, sizeOfRawLidarData, cudaMemcpyHostToDevice));

	// The kernel function uses 900 blocks with 32 threads each. Each block will process 32 laser readings, which corresponds to two laser shootings.
	translateLidarDataFromRawToXYZkernel<<<900,32>>> (rawLidarDataOnGPU,lidarDataPointsOnGPU);

	// Copy the xyz lidar data back to the device for openGL visualization
	CUDA_CHECK_RETURN(cudaMemcpy(lidarDataPoints, lidarDataPointsOnGPU, sizeOfLidarDataPoints, cudaMemcpyDeviceToHost));
}

void LidarProcessing::identifyObstaclesInLidarData() {
	/* This function uses two integer matrices called obstacleMatrixForMaxZOnGPU and obstacleMatrixForMinZOnGPU. Every field
	 * in those matrices represents a coordinate in x and y. Every LidarDataPoint can map its x and y position into a field of
	 * those matrices, by rounding the float x and float y of their coordinates, into int x and int y values that represents
	 * the matrix row and column. Every point in locationLidarDataOnDevice will then write its z coordinate into those two
	 * matrices in the field that represents their x and y. The operation is managed so that if there are two or more
	 * LidarDataPoints that map to the same field in the matrices, only the point with the highest z coordinate will have its
	 * z coordinate written into the obstacleMatrixForMaxZOnGPU, and only the point with the  lowest z coordinate will have
	 * its z coordinate written into the obstacleMatrixForMinZOnGPU. When that is done, a new kernel look at every matrix field
	 * and calculate the difference between the highest and lowest z coordinate. If that difference is larger than a certain threshold value,
	 * then an obstacle have been identified at that matrix field. The definition of an obstacle is then "Two or more LidarDataPoints that
	 * share a similar x and y position but a different z position", e.g. some kind of vertical shape.
	 */
	int numberOfMatrixFields = GROUND_GRID_FIELDS_PER_SIDE * GROUND_GRID_FIELDS_PER_SIDE, numberOfBlocksForMatrixOperations = numberOfMatrixFields/256+1,currentNrOfObstacles=0;

	// Start by zeroing out the obstacleMatrices
	CUDA_CHECK_RETURN(cudaMemcpyToSymbol(currentNrOfObstaclesOnGPU,&currentNrOfObstacles,sizeof(int),0,cudaMemcpyHostToDevice));
	CUDA_CHECK_RETURN(cudaMemset(obstacleMatrixForMaxZOnGPU,0,sizeOfObstacleMatrix));
	CUDA_CHECK_RETURN(cudaMemset(obstacleMatrixForMinZOnGPU,0,sizeOfObstacleMatrix));

	// No write to the max and min matrix. One thread per lidarDataPoint 225*128 = 28800, then identify the obstacles in the matrices and write them to obstacleDataOnDevice
	writeMaxAndMinZToMatrices<<<225,128>>>(obstacleMatrixForMaxZOnGPU,obstacleMatrixForMinZOnGPU,lidarDataPointsOnGPU);
	identifyObstacles<<<numberOfBlocksForMatrixOperations,256>>>(obstacleMatrixForMaxZOnGPU,obstacleMatrixForMinZOnGPU,obstacleSquaresOnGPU);

	// Copy the obstacle data back to the device
	CUDA_CHECK_RETURN(cudaMemcpy(obstacleSquares, obstacleSquaresOnGPU, sizeOfObstacleSquares, cudaMemcpyDeviceToHost));
	CUDA_CHECK_RETURN(cudaMemcpyFromSymbol(&currentNrOfObstacles,currentNrOfObstaclesOnGPU,sizeof(int),0,cudaMemcpyDeviceToHost));
	lidarExportData.currentNrOfObstacles=currentNrOfObstacles;
}

namespace { // Limit scope to translation unit
	#define zValAsInt(zVal) zVal*10000
	#define zValAsFloat(zVal) zVal/10000.0

	__device__ int getShort(const char* atLocation) {
		// 2 chars to int
		return ((*(atLocation+1) & 0xFF)<<8) | (*atLocation & 0xFF);
	}

	__device__ double getAzimuth(const char* atLocation) {
		// In radians
		return ((double) getShort(atLocation))*2*M_PI/36000.0;
	}

	__device__ double getDistance(const char* atLocation) {
		// In meters
		return ((double) getShort(atLocation))*0.002;
	}

	__device__ double getVerticalAngle(int laserIndex) {
		// Returns the vertical angle of the laser with index laserIndex
		switch(laserIndex) {
			case 0: return -15*(M_PI*2/360.0);
			case 1: return 1*(M_PI*2/360.0);
			case 2: return -13*(M_PI*2/360.0);
			case 3: return -3*(M_PI*2/360.0);
			case 4: return -11*(M_PI*2/360.0);
			case 5: return 5*(M_PI*2/360.0);
			case 6: return -9*(M_PI*2/360.0);
			case 7: return 7*(M_PI*2/360.0);
			case 8: return -7*(M_PI*2/360.0);
			case 9: return 9*(M_PI*2/360.0);
			case 10: return -5*(M_PI*2/360.0);
			case 11: return 11*(M_PI*2/360.0);
			case 12: return -3*(M_PI*2360.0);
			case 13: return 13*(M_PI*2/360.0);
			case 14: return -1*(M_PI*2/360.0);
			case 15: return 15*(M_PI*2/360.0);
		}
		return 0;
	}

	__global__ void translateLidarDataFromRawToXYZkernel(char* rawLidarDataOnGPU, LidarDataPoint *lidarPointsOnGPU) {
		/* There are 900 blocks with 32 threads each in this kernel. The rawLidarDataOnGPU contains data from 75 packets from the lidar sensor
		 * Each block will therefore process 1/12th of a packet (75/900). Every thread in a block must know the pointer to the start of the lidar
		 * reading in the rawLidarData. This is accomplished by adding 100*blockId.x to rawLidarDataOnGPU. This is a shared variable in the block
		 * meaning that every thread will have the pointer point to the xFFEE flag in the lidar data ouput (see VLP-16 user manual). In order to read the
		 * azimuth value, the threads will read two bytes starting on  pointerToStartOfPacket+2. In order to know the azimuth values for the second half
		 * of the threads (thread id 16-31), one has to interpolate the azimuth value for the previous blocks azimuth value according to the formula: azimuth
		 * = (myAzimuth - prevBlockAzimuth) / 2. Previous block azimuth is read from pointerToStartOfPacket-98, for every block except block nr 0, where
		 * that would read outside of the rawLidarData buffer (will try to read from position -98). Therefore that block will read azimuths from position
		 * rawLidarData + 89902, which is the same as pointerToStartOfPacket + 89902. Since the entire buffer represents one revolution, that will be ok
		 */
		__shared__ char* pointerToStartOfPacket;
		__shared__ double blockAzimuthAngle, deltaAzimuth;
		pointerToStartOfPacket= rawLidarDataOnGPU + 100*blockIdx.x;
		blockAzimuthAngle = getAzimuth(pointerToStartOfPacket+2);
		deltaAzimuth = (blockAzimuthAngle - getAzimuth(pointerToStartOfPacket + ((blockIdx.x==0) ? 89902 : -98)))/2.0;

		double myHorizontalAngle = (threadIdx.x > 15) ? blockAzimuthAngle : blockAzimuthAngle + deltaAzimuth;
		double myVerticalAngle = getVerticalAngle(threadIdx.x%16);
		double myDistance = getDistance(pointerToStartOfPacket + 4 + 3*threadIdx.x);

		int myThreadNr = blockIdx.x * blockDim.x + threadIdx.x;
		(lidarPointsOnGPU + myThreadNr)->x = myDistance * cos(myVerticalAngle) * sin (myHorizontalAngle);
		(lidarPointsOnGPU + myThreadNr)->y = myDistance * cos(myVerticalAngle) * cos (myHorizontalAngle);
		(lidarPointsOnGPU + myThreadNr)->z = myDistance * sin(myVerticalAngle);
	}

	__global__ void writeMaxAndMinZToMatrices(int* obstacleMatrixForMaxZOnGPU,int* obstacleMatrixForMinZOnGPU,const LidarDataPoint* lidarDataPointsOnGPU) {

		// This function has one thread for each LidarDataPoint, and will write the highest and lowest z value to each matrix field
		int myThreadID = blockIdx.x*blockDim.x+threadIdx.x,myMatrixZval; // 28800 threads
		float myPointX,myPointY,myPointZ;

		myPointX = (lidarDataPointsOnGPU+myThreadID)->x;
		myPointY = (lidarDataPointsOnGPU+myThreadID)->y;
		myPointZ = (lidarDataPointsOnGPU+myThreadID)->z;
		myMatrixZval = zValAsInt(myPointZ); // The integer value that will be written to the min and max matrices
		if (myMatrixZval==0) {myMatrixZval=1;} // matrixZval can't be zero, since a zero in the obstacle matrices means that no value has been written

		/* To go from LidarDataPoint x and y to matrix x and y, first the theoretical max distance that an obstacle can be
		 * from a sensor needs to be calculated. If every index of the obstacleMatrix represents a location for a potential obstacle
		 * then the obstacle that is the farthest away will be obstaclePointSideLength * sideLengthOfObstacleMatrix/2
		 * meters away from the sensor. Matrices don't allow negative values for row or column indexes, so first the LidarDataPoint x and y
		 * has to be incremented by maxObstacleDetectionDistance so that (if they are within the range limited by [-maxObstacleDetectionDistance,
		 * maxObstacleDetectionDistance]) they are positive. After that they will be divided by obstaclePointSideLength so that their x and y positions
		 * in meters can be expressed as a matrix row or column index. After that they will simply be rounded to an integer.
		 */
		__shared__ float maxObstacleDetectionDistance;
		maxObstacleDetectionDistance= GROUND_GRID_RESOLUTION * GROUND_GRID_FIELDS_PER_SIDE/2.0;
		int myMatrixXcoord,myMatrixYcoord,myMatrixPointerOffset;

		if (myPointX<maxObstacleDetectionDistance && myPointX>-maxObstacleDetectionDistance && myPointY<maxObstacleDetectionDistance && myPointY>-maxObstacleDetectionDistance) {
			myMatrixXcoord = (myPointX + maxObstacleDetectionDistance)/GROUND_GRID_RESOLUTION;
			myMatrixYcoord = (myPointY + maxObstacleDetectionDistance)/GROUND_GRID_RESOLUTION;

			myMatrixPointerOffset = myMatrixXcoord * GROUND_GRID_FIELDS_PER_SIDE + myMatrixYcoord;

			// Write to matrices. Remember that max field for a matrix may be negative, and min field for a matrix may be positive:
			atomicCAS((obstacleMatrixForMaxZOnGPU+myMatrixPointerOffset),0,myMatrixZval); // Check if a value has been written to the obstacle matrix first
			atomicMax((obstacleMatrixForMaxZOnGPU+myMatrixPointerOffset),myMatrixZval); // Then write the largest of myMatrixZval and whatever is written on the obstacle matrix
			atomicCAS((obstacleMatrixForMinZOnGPU+myMatrixPointerOffset),0,myMatrixZval);
			atomicMin((obstacleMatrixForMinZOnGPU+myMatrixPointerOffset),myMatrixZval);
		}
	}

	__global__ void identifyObstacles(const int* obstacleMatrixForMaxZOnGPU,const int* obstacleMatrixForMinZOnGPU,ObstaclePoint* obstacleSquaresOnGPU) {
		int myThreadID = blockIdx.x*blockDim.x+threadIdx.x,myMatrixXcoord,myMatrixYcoord,myObstacleIndex,numberOfMatrixFields = GROUND_GRID_FIELDS_PER_SIDE*GROUND_GRID_FIELDS_PER_SIDE;
		float obstacleX,obstacleY;

		myMatrixXcoord = myThreadID/GROUND_GRID_FIELDS_PER_SIDE;
		myMatrixYcoord = myThreadID - myMatrixXcoord*GROUND_GRID_FIELDS_PER_SIDE;
		obstacleX = myMatrixXcoord*GROUND_GRID_RESOLUTION - GROUND_GRID_RESOLUTION*GROUND_GRID_FIELDS_PER_SIDE/2.0;
		obstacleY = myMatrixYcoord*GROUND_GRID_RESOLUTION - GROUND_GRID_RESOLUTION*GROUND_GRID_FIELDS_PER_SIDE/2.0;

		/* To go back from matrix coordinates to LidarDataPoint x and y coordinates, we do the same steps as described
		 * above, but in reverse. An obstacle is registered if myThreadID is within the matrix, the obstacle coords are outside
		 * of the RCV vehicle , and the max Z in obstacleMatrixForMaxZOnGPU minus the min Z in obstacleMatrixForMinZOnGPU is larger
		 * than MIN_OBSTACLE_DELTA_Z
		 */
		if (	myThreadID<numberOfMatrixFields && (abs(obstacleX+GROUND_GRID_RESOLUTION/2.0) > (RCV_WIDTH/2.0 + OBSTACLE_SAFETY_DISTANCE) || abs(obstacleY+GROUND_GRID_RESOLUTION/2.0) > (RCV_LENGTH/2.0 + OBSTACLE_SAFETY_DISTANCE)) &&
				(*(obstacleMatrixForMaxZOnGPU+myThreadID)-*(obstacleMatrixForMinZOnGPU+myThreadID) > zValAsInt(MIN_OBSTACLE_DELTA_Z))) {

			// Obstacle identified
			myObstacleIndex = atomicAdd(&currentNrOfObstaclesOnGPU,1);
			if (myObstacleIndex < MAX_NUMBER_OF_OBSTACLES) {
				// Now write the coordinates to the obstacle square:
				(obstacleSquaresOnGPU+4*myObstacleIndex)->x = obstacleX;
				(obstacleSquaresOnGPU+4*myObstacleIndex)->y = obstacleY;
				(obstacleSquaresOnGPU+4*myObstacleIndex+1)->x = obstacleX+GROUND_GRID_RESOLUTION;
				(obstacleSquaresOnGPU+4*myObstacleIndex+1)->y = obstacleY;
				(obstacleSquaresOnGPU+4*myObstacleIndex+2)->x = obstacleX+GROUND_GRID_RESOLUTION;
				(obstacleSquaresOnGPU+4*myObstacleIndex+2)->y = obstacleY+GROUND_GRID_RESOLUTION;
				(obstacleSquaresOnGPU+4*myObstacleIndex+3)->x = obstacleX;
				(obstacleSquaresOnGPU+4*myObstacleIndex+3)->y = obstacleY+GROUND_GRID_RESOLUTION;
			}
		}
	}
}


