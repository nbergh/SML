#ifndef LIDARCUDAFUNCTIONS_H_
#define LIDARCUDAFUNCTIONS_H_

void translateLidarDataFromRawToXYZ(LidarMemoryPointers* lidarMemoryPointers);
int identifyObstaclesInLidarData(LidarMemoryPointers* lidarMemoryPointers,float obstaclePointSideLength,float minObstacleDeltaZ, int maxNumberOfObstacles);

#endif /* LIDARCUDAFUNCTIONS_H_ */
