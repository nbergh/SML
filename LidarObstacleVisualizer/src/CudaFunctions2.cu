#include <math.h>

#include "Headers/CudaFunctions.h"
#include "Headers/CudaErrorCheckFunctions.h"


__global__ void zeroOutObstacleMatrix(char* obstacleMatrixOnDevice, const int sizeOfObstacleMatrix) {
	int threadID = blockIdx.x*blockDim.x+threadIdx.x;

	if (threadID<sizeOfObstacleMatrix) {
		*(obstacleMatrixOnDevice+threadID)=0;
	}
}

__global__ void identifyObstaclesKernel(LidarDataPoint* locationLidarDataOnDevice, char* obstacleMatrixOnDevice, ObstaclePoint *obstacleDataOnDevice, const int sizeOfObstacleMatrix, const int maxNrOfObstacles) {
	int threadID = blockIdx.x*blockDim.x+threadIdx.x;
	int myPointX,myPointy,myPointZ;

	myPointX = (locationLidarDataOnDevice+threadID)->x;


}



void identifyObstacles(MemoryPointers* memoryPointers) {
	int *globalObstacleArrayIndex; // A pointer to a global variable used by the kernels
	int maxNrOfObstacles = memoryPointers->sizeOfObstacleMatrix/sizeof(ObstaclePoint),nrOfBlocksForMatrixOperations = memoryPointers->sizeOfObstacleMatrix/256 + 1;
	CUDA_CHECK_RETURN(cudaMalloc((void**)&globalObstacleArrayIndex,sizeof(int)));
	CUDA_CHECK_RETURN(cudaMemset(globalObstacleArrayIndex,0,sizeof(int)));

	// Start by zeroing out the obstacleMatrix
	zeroOutObstacleMatrix<<<nrOfBlocksForMatrixOperations,256>>>(memoryPointers->obstacleMatrixOnDevice, memoryPointers->sizeOfObstacleMatrix);

	identifyObstaclesKernel<<<225,128>>>(memoryPointers->locationLidarDataOnDevice,memoryPointers->obstacleMatrixOnDevice,memoryPointers->obstacleDataOnDevice,memoryPointers->sizeOfObstacleMatrix,maxNrOfObstacles);

	// Copy the obstacle data back to the device
	CUDA_CHECK_RETURN(cudaMemcpy(memoryPointers->obstacleData, memoryPointers->obstacleDataOnDevice, memoryPointers->sizeOfObstacleData, cudaMemcpyDeviceToHost));
	cudaDeviceSynchronize();
}
