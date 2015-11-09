#include <math.h>

#include "Headers/LidarProcessing.h"
#include "../Headers/CudaErrorCheckFunctions.h"

__device__ int getShort(char* atLocation) {
	// 2 chars to int
	return ((*(atLocation+1) & 0xFF)<<8) | (*atLocation & 0xFF);
}

__device__ double getAzimuth(char* atLocation) {
	// In radians
	return ((double) getShort(atLocation))*2*M_PI/36000.0;
}

__device__ double getDistance(char* atLocation) {
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
	// MAKE POINTERS CONST
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



void translateLidarDataFromRawToXYZ(LidarMemoryPointers* lidarMemoryPointers) {
	// First copy the raw lidar data from the host to the device
	CUDA_CHECK_RETURN(cudaMemcpy(lidarMemoryPointers->rawLidarDataOnGPU, lidarMemoryPointers->rawLidarData, lidarMemoryPointers->sizeOfRawLidarData, cudaMemcpyHostToDevice));

	// The kernel function uses 900 blocks with 32 threads each. Each block will process 32 laser readings, which corresponds to two laser shootings.
	translateLidarDataFromRawToXYZkernel<<<900,32>>> (lidarMemoryPointers->rawLidarDataOnGPU,lidarMemoryPointers->lidarDataPointsOnGPU);

	// Copy the xyz lidar data back to the device for openGL visualization
	CUDA_CHECK_RETURN(cudaMemcpy(lidarMemoryPointers->lidarDataPoints, lidarMemoryPointers->lidarDataPointsOnGPU, lidarMemoryPointers->sizeOfLidarDataPoints, cudaMemcpyDeviceToHost));
	cudaDeviceSynchronize(); // Wait for the device to finish all its work
}
