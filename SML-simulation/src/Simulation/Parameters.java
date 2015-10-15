package Simulation;

final class Parameters {
	// This class contain every parameter that the simulation requires aswell as the functions used to create them
	
	// Sublclasses
	final MapParameters mapParameters;
	final VehiceParameters vehiceParameters;
	final MapParameters.MapTransmitterParameters mapTransmitterParameters;
	
	Parameters() {
		// Instantiate subclasses
		mapParameters = new MapParameters();
		vehiceParameters = new VehiceParameters();
		mapTransmitterParameters = mapParameters.new MapTransmitterParameters();
	}
	
	boolean checkParamerets() {
		return true;
	}
	
	final class VehiceParameters {
		// This class contains all the parameters of the vehicles
		final int numberOfVehicles=100,controllerUpdateFrequency=10;
		
		// Max/min states of the vehicles + Maximal control signals and speeds. They are the same for every vehicle
		final double maxAcc=0.01, minAcc=-0.025, maxYawSpeed=0.04, minYawSpeed=-0.04, maxSpeed=1,minSpeed=0;
		final int minVehicleWidth=10, maxVehicleWidth=10, minVehicleLength= 10, maxVehicleLength=35; 
		
		// Other vehicle parameters
		final int wifiRange=100, radarRangeFront=30, radarRangeSide=5; // Range for V2V and radar communication. 
		// Radar range side is the range in pixels that the radar has on each side of the middle line of the vehicle

		final double[] vehicleStartStates = {
				// The coordinate system has {0,0} in the lower left corner of the screen, and X grows to the right and Y grows upwards
				
				// The vehicles that are not set manually here will be assigned random values based on the max/min
				// vehicle states as assigned above
							
				// Node target is the index of the node the vehicle wants to go to
				
				// vehicleWidth, vehicleLength, initialStartNode, initialTargetNode, StartSpeed. 			
//				10, 20, 14, 21, 1.0,
//				10, 30, 110, 145, 0.7,
//				10, 10, 142, 34, 1.3,
//				10, 35, 213, 132, 1.2,
//				10, 35, 30, 124, 1.0,
//				10, 30, 134, 27, 0.4,
//				10, 20, 63, 73, 1.3,
//				10, 25, 125, 56, 1.2,
//				10, 10, 30, 12, 1.0,
//				10, 20, 150, 52, 0.4
				};
	}	

	
	final class MapParameters {
		// This class contains all the parameters of the map
		
		// The coordinate system is rotated 90 degrees clockwise from a "normal" x-y system. The X-axis is vertical over the screen 
		// growing from the top of the screen towards the bottom of the screen. The Y-axis is horizontal over the screen growing from 
		// the left of the screen to the right of the screen. Yaw is an angle measured positively counter-clockwise from the X-axis. 
		// It can vary between 0 and 2*pi. This is due to the fact that a matrix has position {row=0,column=0} at the top left corner of
		// a screen and grows downwards (row-wise) and rightwards (column-wise). The coordinate system is adapted so that a row is equal to
		// a position in X and a column is equal to a position in Y		
		final int mapWidth=1690, mapHeight=1000; // In pixels
		final int windowWidth=1680,windowHeight=1050-25,mapUpdateFPS=50;
		
		final int numberOfMapNodes=144, maxNumberOfNeighbours=2, mapNodesRowLength = maxNumberOfNeighbours+2;		
		final int[] mapNodes = {				
				// Coordinate X, coordinate Y, neighbor1, neighbor2,... neighborX
				921, 577, 113, -1,
				943, 442, 34, -1,
				927, 261, 141, -1,
				13, 767, 63, -1,
				11, 841, 3, -1,
				18, 293, 58, -1,
				12, 1462, 70, -1,
				12, 1498, 6, -1,
				17, 1538, 7, -1,
				35, 1564, 8, -1,
				57, 1584, 9, -1,
				81, 1599, 10, -1,
				115, 1607, 11, -1,
				163, 1613, 12, -1,
				218, 1621, 13, -1,
				304, 1624, 14, -1,
				376, 1625, 15, -1,
				464, 1633, 16, -1,
				559, 1639, 17, -1,
				676, 1644, 18, -1,
				765, 1640, 19, -1,
				816, 1634, 20, -1,
				845, 1626, 21, -1,
				864, 1615, 22, -1,
				879, 1601, 23, -1,
				897, 1577, 24, -1,
				907, 1547, 25, -1,
				919, 1511, 26, -1,
				925, 1455, 27, -1,
				928, 1349, 28, -1,
				930, 1221, 29, -1,
				934, 1062, 30, -1,
				936, 867, 31, -1,
				937, 716, 32, -1,
				942, 488, 143, -1,
				947, 312, 142, -1,
				947, 196, 35, -1,
				942, 151, 36, -1,
				932, 123, 37, -1,
				924, 104, 38, -1,
				913, 86, 39, -1,
				898, 71, 40, -1,
				880, 56, 41, -1,
				853, 42, 42, -1,
				791, 28, 43, -1,
				692, 24, 44, -1,
				611, 19, 45, -1,
				523, 16, 46, -1,
				417, 10, 71, -1,
				317, 14, 48, -1,
				219, 22, 49, -1,
				166, 33, 50, -1,
				131, 43, 51, -1,
				92, 60, 52, -1,
				60, 83, 53, -1,
				38, 109, 54, -1,
				24, 136, 55, -1,
				17, 168, 56, -1,
				16, 202, 57, -1,
				15, 380, 5, -1,
				15, 475, 59, -1,
				15, 561, 60, -1,
				16, 650, 61, -1,
				14, 728, 62, -1,
				10, 902, 4, -1,
				11, 978, 64, -1,
				14, 1074, 65, -1,
				14, 1144, 66, -1,
				14, 1213, 67, -1,
				16, 1284, 68, -1,
				17, 1368, 69, -1,
				445, 11, 47, -1,
				36, 1369, 73, -1,
				35, 1284, 74, -1,
				33, 1213, 75, -1,
				33, 1144, 76, -1,
				33, 1073, 77, -1,
				30, 978, 78, -1,
				29, 902, 79, -1,
				30, 842, 80, -1,
				32, 768, 81, -1,
				33, 734, 82, -1,
				35, 650, 83, -1,
				34, 561, 84, -1,
				34, 475, 85, -1,
				34, 380, 86, -1,
				37, 294, 87, -1,
				35, 202, 88, -1,
				36, 171, 89, -1,
				42, 142, 90, -1,
				54, 120, 91, -1,
				73, 97, 92, -1,
				101, 77, 93, -1,
				138, 61, 94, -1,
				171, 51, 95, -1,
				222, 41, 96, -1,
				319, 33, 97, -1,
				417, 29, 98, -1,
				444, 30, 99, -1,
				523, 35, 100, -1,
				610, 38, 101, -1,
				691, 43, 102, -1,
				788, 47, 103, -1,
				847, 61, 104, -1,
				869, 72, 105, -1,
				885, 85, 106, -1,
				897, 98, 107, -1,
				907, 113, 108, -1,
				915, 131, 109, -1,
				923, 155, 110, -1,
				927, 197, 2, -1,
				927, 312, 112, -1,
				923, 488, 0, -1,
				918, 716, 114, -1,
				916, 866, 115, -1,
				915, 1061, 116, -1,
				911, 1220, 117, -1,
				909, 1348, 118, -1,
				906, 1454, 119, -1,
				900, 1507, 120, -1,
				889, 1541, 121, -1,
				880, 1568, 122, -1,
				864, 1588, 123, -1,
				853, 1600, 124, -1,
				837, 1608, 125, -1,
				812, 1615, 126, -1,
				763, 1621, 127, -1,
				676, 1624, 128, -1,
				560, 1620, 129, -1,
				466, 1614, 130, -1,
				377, 1606, 131, -1,
				305, 1605, 132, -1,
				219, 1601, 133, -1,
				165, 1594, 134, -1,
				118, 1589, 135, -1,
				88, 1581, 136, -1,
				68, 1569, 137, -1,
				49, 1551, 138, -1,
				36, 1531, 139, -1,
				31, 1497, 140, -1,
				31, 1462, 72, -1,
				927, 282, 111, 142,
				944, 413, 1, -1,
				941, 535, 33, 113};
	
		final class MapTransmitterParameters {
			final int numberOfTransmitters=1;
			// This class contains all the parameters for the I2V transmitters in the map
			
			int[] transmitterParameters = {
					// PosX, posY, transmitter range, fromZoneStartX,fromZoneEndX,fromZoneStartY,fromZoneEndY,toZoneStartX,toZoneEndX,toZoneStartY,toZoneEndY
					50, 835, 400, 23, 43, 410 ,1250, 5, 23, 410 ,1250
			};
		}
	}		
}


