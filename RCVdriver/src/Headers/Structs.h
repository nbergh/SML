#ifndef STRUCTS_H_
#define STRUCTS_H_

// This header contains definitions for structs that are used in this program

struct LidarDataPoint {
	// This struct represents an openGL vertex with 3 coordinates
	float x,y,z;
};
struct ObstaclePoint {
	float x,y;
};
struct PathPoint {
	float x,y;
	char r,b,g; // Colors of the point
};
struct GPSposition {
	// GPS position, always in decimal degrees, latc degrees NORTH, longc degrees EAST
	double latc, longc;
};
struct VehicleState {
	// The position of the vehicle
	GPSposition currentPosition;
	double currentHeading;
	bool isRunning; // If this is false, no data will go on to the CAN bus
};


#endif /* STRUCTS_H_ */
