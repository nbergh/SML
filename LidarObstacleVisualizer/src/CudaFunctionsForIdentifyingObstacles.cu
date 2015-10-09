#include <math.h>

#include "Headers/StructDefinitions.h"
#include "Headers/CudaErrorCheckFunctions.h"


__global__ void zeroOutObstacleMatrix(char* obstacleMatrixOnDevice, const int sizeOfObstacleMatrix) {
	int threadID = blockIdx.x*blockDim.x+threadIdx.x;

	if (threadID<sizeOfObstacleMatrix) {
		*(obstacleMatrixOnDevice+threadID)=0;
	}
}

__global__ void writeMaxAndMinZToMatrices(LidarDataPoint* locationLidarDataOnDevice, char* obstacleMatrixForMaxZOnDevice, char* obstacleMatrixForMinZOnDevice, int sizeOfOneObstacleMatrix) {
	int threadID = blockIdx.x*blockDim.x+threadIdx.x;
	int myPointX,myPointY,myPointZ;

	myPointX = (locationLidarDataOnDevice+threadID)->x;
	myPointY = (locationLidarDataOnDevice+threadID)->y;
	myPointZ = (locationLidarDataOnDevice+threadID)->z;



}



void identifyObstaclesInLidarData(MemoryPointers* memoryPointers, int maxNrOfObstacles) {
	int nrOfBlocksForMatrixOperations = memoryPointers->sizeOfOneObstacleMatrix/256 + 1;
	int *deviceObstacleArrayIndex; // A pointer to a global variable used by the kernels
	CUDA_CHECK_RETURN(cudaMalloc((void**)&deviceObstacleArrayIndex,sizeof(int))); // Allocate deviceObstacleArrayIndex on device and set it to zero
	CUDA_CHECK_RETURN(cudaMemset(deviceObstacleArrayIndex,0,sizeof(int)));

	// Start by zeroing out the obstacleMatrices
	zeroOutObstacleMatrix<<<nrOfBlocksForMatrixOperations,256>>>(memoryPointers->obstacleMatrixForMaxZOnDevice, memoryPointers->sizeOfOneObstacleMatrix);
	zeroOutObstacleMatrix<<<nrOfBlocksForMatrixOperations,256>>>(memoryPointers->obstacleMatrixForMinZOnDevice, memoryPointers->sizeOfOneObstacleMatrix);
	// No write to the max and min matrix
	writeMaxAndMinZToMatrices<<<225,128>>>(memoryPointers->locationLidarDataOnDevice,memoryPointers->obstacleMatrixForMaxZOnDevice,memoryPointers->obstacleMatrixForMinZOnDevice,memoryPointers->sizeOfOneObstacleMatrix);

	// Copy the obstacle data back to the device
	CUDA_CHECK_RETURN(cudaMemcpy(memoryPointers->obstacleData, memoryPointers->obstacleDataOnDevice, memoryPointers->sizeOfObstacleData, cudaMemcpyDeviceToHost));
	cudaDeviceSynchronize();
}
