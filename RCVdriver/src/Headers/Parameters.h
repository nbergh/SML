#ifndef PARAMETERS_H_
#define PARAMETERS_H_

/* This header file includes a lot of macros that govern the way the vehicle runs
 *
 * Occupancy grid is a rectangular 2-D XY plane with the vehicle in the center
 */

namespace PARAMETERS {
	const int CONTROLLER_UPDATE_RATE = 10;
	const int  GRAPHICS_FRAME_RATE = 60;
	const float OCCUPANCY_GRID_CELL_SIZE = 0.05; // The length in meters of the side of one ground grid cell (square)
	const int NR_OCCUPANCY_GRID_CELLS_Y_WISE = 1200; // The number of cells longitudinally in the occupancy grid
	const int NR_OCCUPANCY_GRID_CELLS_X_WISE = 600; // The number of cells latitudinally in the occupancy grid
	const float MIN_OBSTACLE_DELTA_Z = 0.1; // The minimum delta-z required between two points for them to be counted as an obstacle
	const int MAX_NUMBER_OF_OBSTACLES = 5000; //The maximum number of obstacles that can be stored in memory every main controller loop iteration
	const float RCV_WIDTH = 1.72; // In meters
	const float RCV_LENGTH = 2.75; // In meters
	const float RCV_HEIGHT = 1.4; // In meters
}

#endif /* PARAMETERS_H_ */
