#ifndef PARAMETERS_H_
#define PARAMETERS_H_

/* This header file includes a lot of macros that govern the way the vehicle runs
 *
 * Ground grid is quadratic 2-D XY plane with the vehicle in the center...
 */


#define CONTROLLER_UPDATE_RATE 10
#define GRAPHICS_FRAME_RATE 60
#define GROUND_GRID_RESOLUTION 0.05 // The length in meters of the side of one ground grid field (square)
#define GROUND_GRID_FIELDS_PER_SIDE 800 // The number of ground grid fields per side of the ground grid
#define MIN_OBSTACLE_DELTA_Z 0.1 // The minimum delta-z required between two points for them to be counted as an obstacle
#define MAX_NUMBER_OF_OBSTACLES 5000 //The maximum number of obstacles that can be stored in memory every main controller loop iteration
#define RCV_WIDTH 0.8 // In meters
#define RCV_LENGTH 2.4 // In meters
#define RCV_HEIGHT 0.7 // In meters
#define OBSTACLE_SAFETY_DISTANCE 0.05
/* The minimum distance in meters from {the midpoint of} any obstacle that the pathplanner will keep. An obstacle is represented as a
 * square (with size GROUND_GRID_RESOLUTION*GROUND_GRID_RESOLUTION) in the ground grid. Hence any point closer to sqrt(2)/2 * GROUND_GRID_RESOLUTION
 * to the midpoint of an obstacle will be in danger of touching that obstacle. Therefore the safety distance must be greater than
 * sqrt(2)/2 * GROUND_GRID_RESOLUTION
 */


#endif /* PARAMETERS_H_ */
