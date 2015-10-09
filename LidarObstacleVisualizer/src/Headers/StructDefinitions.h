/*
 * StructDefinitions.h
 *
 *  Created on: Oct 4, 2015
 *      Author: sml-linux
 */

#ifndef STRUCTDEFINITIONS_H_
#define STRUCTDEFINITIONS_H_

struct LidarDataPoint {
	// Simple struct for representing a lidar data point
	float x,y,z; // In meters
};
struct ObstaclePoint {
	float x,y; // In meters
};

struct CameraPosition {
	float x;
	float y;
	float z;
	float roll; // The rotation along the x axis
	float yaw; // The rotation around the z axis
	float oldRoll; // The value of roll as it was when it was last modified
	float oldYaw; // The value of yaw as it was when it was last modified
};

struct KeysAndMouseState {
	// Keys and mouse state for openGL visualization
	bool leftButtonIsPressed;
	int mousePosXwhenPressed;
	int mousePosYwhenPressed;
};

struct MemoryPointers {
	// A struct to holds all the pointers and the sizes of allocated memory blocks
	char *rawLidarData, *rawLidarDataOnDevice;
	int sizeOfRawLidarData;

	LidarDataPoint *locationLidarData, *locationLidarDataOnDevice;
	int sizeOfLocationLidarData;

	ObstaclePoint *obstacleData, *obstacleDataOnDevice;
	int sizeOfObstacleData;

	char *obstacleMatrixOnDevice;
	int sizeOfObstacleMatrix;
};

#endif /* STRUCTDEFINITIONS_H_ */
