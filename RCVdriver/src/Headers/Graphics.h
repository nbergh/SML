#ifndef GRAPHICS_H_
#define GRAPHICS_H_

// This header contains a function for creating a window and displaying the lidar point cloud

void startGraphics(LidarDataPoint* lidarDataPoints, ObstaclePoint* obstacleSquares, int* currentNrOfObstacles, int frameRate);
// Starts the glut main loop, and doesn't return until the program is exited by pressing 'q'

#endif /* GRAPHICS_H_ */
