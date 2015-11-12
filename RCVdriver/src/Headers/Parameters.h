#ifndef PARAMETERS_H_
#define PARAMETERS_H_

/* This header file includes a lot of macros that govern the way the vehicle runs
 *
 * Ground grid is quadratic 2-D XY plane with the vehicle in the center...
 */


#define CONTROLLER_UPDATE_RATE 10
#define GRAPHICS_FRAME_RATE 60
#define GROUND_GRID_RESOLUTION 0.05 // The length in meters of the side of one ground grid square
#define GROUND_GRID_SIDE_LENGTH 40 // The length in meters of one side of the ground grid
#define MIN_OBSTACLE_DELTA_Z 0.1 // The minimum delta-z required between two points for them to be counted as an obstacle
#define MAX_NUMBER_OF_OBSTACLES 5000 //The maximum number of obstacles that can be stored in memory every main controller loop iteration
#define RCV_WIDTH 0.8 // In meters
#define RCV_LENGTH 2.4 // In meters
#define RCV_HEIGHT 0.7 // In meters
#define OBSTACLE_SAFETY_DISTANCE 0.1
/* The minimum distance in meters from any obstacle that the pathplanner will keep. Don't set to less than GROUND_GRID_RESOLUTION,
 * since the distance to an obstacle from a pathNode has an an error margin of +/- GROUND_GRID_RESOLUTION
 */


#endif /* PARAMETERS_H_ */
