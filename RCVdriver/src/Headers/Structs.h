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
struct GPSposition {
	// GPS position, always in decimal degrees
	float latc, longc;
};
struct VehicleState {
	// The position of the vehicle
	GPSposition currentPosition;
	double heading;
};


#endif /* STRUCTS_H_ */
