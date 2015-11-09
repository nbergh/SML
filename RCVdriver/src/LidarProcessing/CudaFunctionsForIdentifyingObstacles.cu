#include <math.h>

#include "Headers/LidarProcessing.h"
#include "../Headers/CudaErrorCheckFunctions.h"

#define zValAsInt(zVal) zVal*10000
#define zValAsFloat(zVal) zVal/10000.0

__global__ void writeMaxAndMinZToMatrices(	int* obstacleMatrixForMaxZOnGPU,
											int* obstacleMatrixForMinZOnGPU,
											LidarDataPoint* lidarDataPointsOnGPU,
											int numberOfMatrixFieldsPerSide,
											float obstaclePointSideLength) {

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
	maxObstacleDetectionDistance= obstaclePointSideLength * numberOfMatrixFieldsPerSide/2.0;
	int myMatrixXcoord,myMatrixYcoord,myMatrixPointerOffset;

	if (myPointX<maxObstacleDetectionDistance && myPointX>-maxObstacleDetectionDistance && myPointY<maxObstacleDetectionDistance && myPointY>-maxObstacleDetectionDistance) {
		myMatrixXcoord = (myPointX + maxObstacleDetectionDistance)/obstaclePointSideLength;
		myMatrixYcoord = (myPointY + maxObstacleDetectionDistance)/obstaclePointSideLength;

		myMatrixPointerOffset = myMatrixXcoord * numberOfMatrixFieldsPerSide + myMatrixYcoord;

		// Write to matrices:
		atomicCAS((obstacleMatrixForMaxZOnGPU+myMatrixPointerOffset),0,myMatrixZval); // Check if a value has been written to the obstacle matrix first
		atomicMax((obstacleMatrixForMaxZOnGPU+myMatrixPointerOffset),myMatrixZval); // Then write the largest of myMatrixZval and whatever is written on the obstacle matrix
		atomicCAS((obstacleMatrixForMinZOnGPU+myMatrixPointerOffset),0,myMatrixZval);
		atomicMin((obstacleMatrixForMinZOnGPU+myMatrixPointerOffset),myMatrixZval);
	}
}

__global__ void identifyObstacles(	int* obstacleMatrixForMaxZOnGPU,
									int* obstacleMatrixForMinZOnGPU,
									ObstaclePoint* obstacleSquaresOnGPU,
									int* deviceObstacleArrayIndex,
									int numberOfMatrixFieldsPerSide,
									int numberOfMatrixFields,
									float obstaclePointSideLength,
									float minObstacleDeltaZ,
									int maxNumberOfObstacles) {

	__shared__ float maxObstacleDetectionDistance;
	maxObstacleDetectionDistance = obstaclePointSideLength * numberOfMatrixFieldsPerSide/2.0;
	int myThreadID = blockIdx.x*blockDim.x+threadIdx.x,myMatrixXcoord,myMatrixYcoord,myObstacleIndex;
	float obstacleX,obstacleY;

	/* To go back from matrix coordinates to LidarDataPoint x and y coordinates, we do the same steps as described
	 * above, but in reverse
	 */
	if (myThreadID<numberOfMatrixFields && (*(obstacleMatrixForMaxZOnGPU+myThreadID) - *(obstacleMatrixForMinZOnGPU+myThreadID) > zValAsInt(minObstacleDeltaZ))) {
		// Obstacle identified
		myObstacleIndex = atomicAdd(deviceObstacleArrayIndex,1);
		if (myObstacleIndex < maxNumberOfObstacles) {
			myMatrixXcoord = myThreadID/numberOfMatrixFieldsPerSide;
			myMatrixYcoord = myThreadID - myMatrixXcoord*numberOfMatrixFieldsPerSide;
			obstacleX = myMatrixXcoord*obstaclePointSideLength-maxObstacleDetectionDistance;
			obstacleY = myMatrixYcoord*obstaclePointSideLength-maxObstacleDetectionDistance;
			// Now write the coordinates to the obstacle square:
			(obstacleSquaresOnGPU+4*myObstacleIndex)->x = obstacleX;
			(obstacleSquaresOnGPU+4*myObstacleIndex)->y = obstacleY;
			(obstacleSquaresOnGPU+4*myObstacleIndex+1)->x = obstacleX+obstaclePointSideLength;
			(obstacleSquaresOnGPU+4*myObstacleIndex+1)->y = obstacleY;
			(obstacleSquaresOnGPU+4*myObstacleIndex+2)->x = obstacleX;
			(obstacleSquaresOnGPU+4*myObstacleIndex+2)->y = obstacleY+obstaclePointSideLength;
			(obstacleSquaresOnGPU+4*myObstacleIndex+3)->x = obstacleX+obstaclePointSideLength;
			(obstacleSquaresOnGPU+4*myObstacleIndex+3)->y = obstacleY+obstaclePointSideLength;
		}
	}
}


int identifyObstaclesInLidarData(LidarMemoryPointers* lidarMemoryPointers, float obstaclePointSideLength,float minObstacleDeltaZ, int maxNumberOfObstacles) {
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

	int numberOfMatrixFields = lidarMemoryPointers->numberOfMatrixFieldsPerSide * lidarMemoryPointers->numberOfMatrixFieldsPerSide;
	int numberOfBlocksForMatrixOperations = numberOfMatrixFields/256+1;
	int *deviceObstacleArrayIndex; // A pointer to a global variable used by the kernels
	CUDA_CHECK_RETURN(cudaMalloc((void**)&deviceObstacleArrayIndex,sizeof(int))); // Allocate deviceObstacleArrayIndex on device
	CUDA_CHECK_RETURN(cudaMemset(deviceObstacleArrayIndex,0,sizeof(int))); // Set it to zero

	// Start by zeroing out the obstacleMatrices
	CUDA_CHECK_RETURN(cudaMemset(lidarMemoryPointers->obstacleMatrixForMaxZOnGPU,0,lidarMemoryPointers->sizeOfObstacleMatrix));
	CUDA_CHECK_RETURN(cudaMemset(lidarMemoryPointers->obstacleMatrixForMinZOnGPU,0,lidarMemoryPointers->sizeOfObstacleMatrix));

	// No write to the max and min matrix. One thread per lidarDataPoint 225*128 = 28800
	writeMaxAndMinZToMatrices<<<225,128>>>(	lidarMemoryPointers->obstacleMatrixForMaxZOnGPU,
											lidarMemoryPointers->obstacleMatrixForMinZOnGPU,
											lidarMemoryPointers->lidarDataPointsOnGPU,
											lidarMemoryPointers->numberOfMatrixFieldsPerSide,
											obstaclePointSideLength);

	// Finally identify the obstacles in the matrices and write them to obstacleDataOnDevice
	identifyObstacles<<<numberOfBlocksForMatrixOperations,256>>>(	lidarMemoryPointers->obstacleMatrixForMaxZOnGPU,
																	lidarMemoryPointers->obstacleMatrixForMinZOnGPU,
																	lidarMemoryPointers->obstacleSquaresOnGPU,
																	deviceObstacleArrayIndex,
																	lidarMemoryPointers->numberOfMatrixFieldsPerSide,
																	numberOfMatrixFields,
																	obstaclePointSideLength,
																	minObstacleDeltaZ,
																	maxNumberOfObstacles);

	// Copy the obstacle data back to the device
	CUDA_CHECK_RETURN(cudaMemcpy(lidarMemoryPointers->obstacleSquares, lidarMemoryPointers->obstacleSquaresOnGPU, lidarMemoryPointers->sizeOfObstacleSquares, cudaMemcpyDeviceToHost));

	// Set the number of obstacles idenitified in lidarMemoryPointers->currentNrOfObstacles
	int nrOfObstacles;
	CUDA_CHECK_RETURN(cudaMemcpy(&nrOfObstacles, deviceObstacleArrayIndex, sizeof(int), cudaMemcpyDeviceToHost));
	CUDA_CHECK_RETURN(cudaFree(deviceObstacleArrayIndex)); // TODO this shouldnt be done every iteration

	cudaDeviceSynchronize();
	return nrOfObstacles;
}
