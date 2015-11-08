/* This header declares the two entry functions (for GPU calculations) used by the CPU
 *
 */

#ifndef CUDAFUNCTIONS_H_
#define CUDAFUNCTIONS_H_

#include "StructDefinitions.h"

void translateLidarDataFromRawToXYZ(MemoryPointers* memoryPointers);
void intializeObstacleSquareIndexesArray(MemoryPointers* memoryPointers, int maxNrOfObstacles);
void identifyObstaclesInLidarData(MemoryPointers* memoryPointers,float obstaclePointSideLength,float minObstacleDeltaZ, int maxNumberOfObstacles);

#endif /* CUDAFUNCTIONS_H_ */
