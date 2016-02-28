#ifndef STRUCTS_H_
#define STRUCTS_H_

// This header contains definitions for structs that are used in RCVdriver

// Used by opengGL:
struct LidarPointForGraphics {
	// This struct represents an openGL vertex with 3 coordinates
	float x,y,z;
};
struct ObstaclePointForGraphics {
	float x,y;
};
struct PathPointInLocalXY {
	float x,y;
	unsigned char r,g,b; // Colors of the point
};
struct LidarExportData {
	LidarPointForGraphics* lidarPointsListForGraphics;
	ObstaclePointForGraphics* obstaclePointListForGraphics;
	int currentNrOfObstaclesForGraphics;
};
struct PathExportData {
	PathExportData(int& lengthOfMacroPath,int& currentIndexInMacroPath,int& lengthOfMicroPath,int& currentIndexInMicroPath, PathPointInLocalXY*& macroPathXY, PathPointInLocalXY*& microPathXY) :
		lengthOfMacroPath(lengthOfMacroPath),currentIndexInMacroPath(currentIndexInMacroPath),
		lengthOfMicroPath(lengthOfMicroPath),currentIndexInMicroPath(currentIndexInMicroPath),
		macroPathXY(macroPathXY),microPathXY(microPathXY) {}

	PathPointInLocalXY*& macroPathXY,*& microPathXY;
	int& lengthOfMacroPath,& currentIndexInMacroPath,& lengthOfMicroPath,& currentIndexInMicroPath;
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
	double courseFromPreviousPathPoint;
	bool isReversingFromPrevNode;
};
struct VehiclePosition {
	// The position and heading of the vehicle. This is the data coming from the trimble and IMU
	GPSposition currentPosition;
	float currentHeading, groundSpeed;
};
struct VehicleStatus {
	/* A struct with information that the program uses to inform on what is going on in runtime
	 * If isRunning is false, no data will go on the CAN bus
	 */
	bool isRunning,isAbleToGenerateMicroPath,isReceivingGPSdata,macroPathHasBeenSet;
};


#endif /* STRUCTS_H_ */
