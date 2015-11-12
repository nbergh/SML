#ifndef STRUCTS_H_
#define STRUCTS_H_

// This header contains definitions for structs that are used in RCVdriver

// Used by opengGL:
struct LidarDataPoint {
	// This struct represents an openGL vertex with 3 coordinates
	float x,y,z;
};
struct ObstaclePoint {
	float x,y;
};
struct PathPointInLocalXY {
	float x,y;
	char r,b,g; // Colors of the point
	double headingFromPrevPathPoint;
};

//Not used by openGL:
struct GPSposition {
	// GPS position, always in decimal degrees, latc degrees NORTH, longc degrees EAST
	double latc, longc;
};
struct PathPointInGPScords {
	/* Struct used in microPathGPS and microPathGPS. HeadingFromPrevPathPoint is the heading
	 * (radian degrees from true north) from the previous point in the path
	 */
	GPSposition position;
	float latDistanceFromPrevPathPoint,longDistanceFromPrevPathPoint; // The distance in meters from prevPathPoint to this point, lat and long
	double headingFromPrevPathPoint;
};
struct VehicleState {
	// The position and heading of the vehicle
	GPSposition currentPosition;
	double currentHeading;
};
struct VehicleStatus {
	/* A struct with information that the program uses to inform on what is going on in runtime
	 * If isRunning is false, no data will go on the CAN bus
	 */
	bool isRunning,isAbleToGenerateMicroPath,isReceivingGPSdata,macroPathHasBeenSet;
};


#endif /* STRUCTS_H_ */
