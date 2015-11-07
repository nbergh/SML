#ifndef STRUCTDEFINITIONS_H_
#define STRUCTDEFINITIONS_H_

#include <GL/gl.h>

struct OpenGLvertex {
	// This struct represents an openGL vertex with 3 coordinates
	GLfloat x,y,z;
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
	bool leftButtonIsPressed,forwardKeyIsPressed,backwardKeyIsPressed,leftStrafeKeyIsPressed,rightStrafeKeyIsPressed,upStrafeKeyIsPressed,downStrafeKeyIsPressed;
	int mousePosXwhenPressed;
	int mousePosYwhenPressed;
};

struct GPSpoint {
	double latc, longc; // Lat and long coords for the GPSpoint. Always in decimal degrees
};

struct Path {
	// A struct representing a path as a linked list of gps points
	GPSpoint* myGPSpoint;
	Path* nextPoint;
};

struct VehicleState {
	// TODO make separate structs for all sensor data
	GPSpoint* currentGPSposition; // This is maintained by both the GPS and IMU
	double currentHeading; // The current heading
	int gpsTimeStamp; // Timestamp of the time when the GPS was last updated, usec from gettimeofday()
	// Add more stuff
};

struct MemoryPointers {
	// A struct to holds all the pointers and the sizes of allocated memory blocks
	char *rawLidarData, *rawLidarDataOnDevice;
	int sizeOfRawLidarData;

	OpenGLvertex *lidarPoints, *lidarPointsOnDevice;
	int sizeOfLidarPoints;

	OpenGLvertex *obstacleSquares, *obstacleSquaresOnDevice;
	GLuint* obstacleSquareIndexesArray;
	int sizeOfObstacleSquares,sizeOfObstacleSquareIndexesArray;

	int *obstacleMatrixForMaxZOnDevice;
	int *obstacleMatrixForMinZOnDevice;
	int sizeOfObstacleMatrix,numberOfMatrixFieldsPerSide;

	OpenGLvertex* pathPoints; // Representation of the path local coordinates for displaying
	int sizeOfPathPoints;

	VehicleState* vehicleState; // The current state of the vehicle

	int currentNrOfObstacles; // The number of obstacles to be displayed in the current frame
};

#endif /* STRUCTDEFINITIONS_H_ */
