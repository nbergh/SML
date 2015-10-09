/*
 ============================================================================
 Name        : LidarObstacleVisualizer.cpp
 Author      : Niklas Bergh
 Version     :
 Copyright   :
 Description : Program identifies and displays obstacles based on Lidar data
 ============================================================================
 */
#include <GL/freeglut.h>
#include <stdio.h>

#include "Headers/StructDefinitions.h"
#include "Headers/LidarUDPReceiver.h"
#include "Headers/CudaFunctions.h"
#include "Headers/CudaErrorCheckFunctions.h"

#define LIDAR_UDP_PORT 2368 // UDP listening port
#define FRAMERATE 50 // The framerate for the openGL graphics
#define MAX_OBSTACLE_DETECTION_RANGE 20 // The maximal distance (in meters) that an obstacle can be identified in meters
#define OBSTACLE_RESOLUTION 0.01 // The resolution of the displayed obstacle matrix (in meters)
#define MAX_NUMBER_OF_OBSTACLE_POINTS 5000 //The maximal number of obstacle points that can be displayed
#define OBSTACLE_MIN_DELTA_Z 0.2 // The difference in z coordinates (in meters) required for two points with the same x and y coordinates to be registered as an obstacle

MemoryPointers* memoryPointers;
CameraPosition* cameraPosition;
KeysAndMouseState* keysAndMouseState;

void allocateMemory() {
	memoryPointers = new MemoryPointers;
	memset(memoryPointers,0,sizeof(MemoryPointers));
	cameraPosition = new CameraPosition;
	memset(cameraPosition,0,sizeof(CameraPosition));
	keysAndMouseState = new KeysAndMouseState;
	memset(keysAndMouseState,0,sizeof(KeysAndMouseState));

	// First is rawLidarData. It is a byte array of raw UDP data (minus UDP headers and factory bytes), representing 75 UDP packets (one revolution of the lidar sensor). Size is 1200*75 = 90000 bytes
	memoryPointers->sizeOfRawLidarData = 90000;
	CUDA_CHECK_RETURN(cudaMallocHost((void**)&memoryPointers->rawLidarData,memoryPointers->sizeOfRawLidarData));
	CUDA_CHECK_RETURN(cudaMalloc((void**)&memoryPointers->rawLidarDataOnDevice,memoryPointers->sizeOfRawLidarData));

	// Second is the locationLidarData. This is an array of LidarDataPoint structs, that contain values for x,y,z for 28800 points (one revolution)
	memoryPointers->sizeOfLocationLidarData = 28800*sizeof(LidarDataPoint);
	CUDA_CHECK_RETURN(cudaMallocHost((void**)&memoryPointers->locationLidarData,memoryPointers->sizeOfLocationLidarData));
	CUDA_CHECK_RETURN(cudaMalloc((void**)&memoryPointers->locationLidarDataOnDevice,memoryPointers->sizeOfLocationLidarData));

	 // Third is an array of ObstaclePoint structs, where each x,y in each ObstaclePoint represents an obstacle
	memoryPointers->sizeOfObstacleData = MAX_NUMBER_OF_OBSTACLE_POINTS*sizeof(ObstaclePoint);
	CUDA_CHECK_RETURN(cudaMallocHost((void**)&memoryPointers->obstacleData,memoryPointers->sizeOfObstacleData));
	CUDA_CHECK_RETURN(cudaMalloc((void**)&memoryPointers->obstacleDataOnDevice,memoryPointers->sizeOfObstacleData));

	// Finally the obstacle matrix is allocated on the device
	memoryPointers->sizeOfObstacleMatrix = (2 * MAX_OBSTACLE_DETECTION_RANGE / OBSTACLE_RESOLUTION) * (2 * MAX_OBSTACLE_DETECTION_RANGE / OBSTACLE_RESOLUTION);
	CUDA_CHECK_RETURN(cudaMalloc((void**)&memoryPointers->obstacleMatrixOnDevice,memoryPointers->sizeOfObstacleMatrix));
}

void freeMemory() {
	free(memoryPointers->rawLidarData);
	CUDA_CHECK_RETURN(cudaFree(memoryPointers->rawLidarDataOnDevice));
	free(memoryPointers->locationLidarData);
	CUDA_CHECK_RETURN(cudaFree(memoryPointers->locationLidarDataOnDevice));
	free(memoryPointers->obstacleData);
	CUDA_CHECK_RETURN(cudaFree(memoryPointers->obstacleDataOnDevice));
	CUDA_CHECK_RETURN(cudaFree(memoryPointers->obstacleMatrixOnDevice));
	free(memoryPointers);
	free(cameraPosition);
	free(keysAndMouseState);
}

void updateFrame(int data) {
	glutTimerFunc(1000/FRAMERATE,updateFrame,0); // Call again in 1000/FRAMERATE milliseconds

	//timeval curTime;
	//gettimeofday(&curTime,NULL);
	//printf("%d\n",curTime.tv_usec);
	//printf("%s%f%s%f\n","Pitch: ",cameraPosition->pitch,", yaw: ",cameraPosition->yaw);
	//printf("%s%f%s%f%s%f\n","X: ",cameraPosition->x,", y: ",cameraPosition->y,", z: ",cameraPosition->z);

	// Do the CUDA calculation on the lidar data:
	translateLidarDataFromRawToXYZ(memoryPointers);

	glutPostRedisplay();
}

void setUpDisplay() {
	// Load the Projection Matrix
	glMatrixMode(GL_PROJECTION);glLoadIdentity();
	// Set the viewport to be the entire window
	glViewport(0, 0, glutGet(GLUT_SCREEN_WIDTH), glutGet(GLUT_SCREEN_HEIGHT));
	// Set the correct perspective.
	gluPerspective(45.0f, glutGet(GLUT_SCREEN_WIDTH)/(double) glutGet(GLUT_SCREEN_HEIGHT), 0.1f, 100.0f);

	// Set the pointer to location lidar data
	glVertexPointer( 3, GL_FLOAT, sizeof(LidarDataPoint), &memoryPointers->locationLidarData->x ); // Set the vertex pointer
	glPointSize( 2.0 );
}


void drawDisplay(void) {
	// This function is called by openGL when it decides that the window needs to be redrawn

	// Load the modelview matrix and change it according to the position of the camera
	glMatrixMode(GL_MODELVIEW);glLoadIdentity();
	// Change modelview according to the camera position (inverted because when we want to move the camera somewhere, we move the model in the other direction)
	glRotatef(-cameraPosition->roll, 1.0f, 0.0f, 0.0f);
	glRotatef(-(cameraPosition->yaw), 0.0f, 0.0f, 1.0f);
	glTranslatef(-cameraPosition->x, -cameraPosition->y, -cameraPosition->z);

	// Clear Color and Depth Buffers
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glEnableClientState( GL_VERTEX_ARRAY );
    glDrawArrays( GL_POINTS, 0, 28800 ); // Draws all the points from the LIDAR
    glDisableClientState( GL_VERTEX_ARRAY );

    glFlush();

	glutSwapBuffers();
}

void handleKeyboard(unsigned char key, int x, int y) {
	// Exits the application
	if (key=='q') {
		glutLeaveMainLoop();
	}
}

void handleMouseMove(int x, int y) {
	if (keysAndMouseState->leftButtonIsPressed) {
		cameraPosition->yaw = cameraPosition->oldYaw + 0.2f*(keysAndMouseState->mousePosXwhenPressed-x);
		cameraPosition->roll = cameraPosition->oldRoll + 0.2f*(keysAndMouseState->mousePosYwhenPressed-y);
	}
}

void handleMouseClick(int button, int state, int x, int y) {
	float zoomStep =0.1f;

	if (button == GLUT_LEFT_BUTTON) {
		if (state == GLUT_DOWN) {
			// Left mouse button is pressed
			keysAndMouseState->leftButtonIsPressed=true;
			keysAndMouseState->mousePosXwhenPressed=x;
			keysAndMouseState->mousePosYwhenPressed=y;
		}
		else  {
			// Left mouse button is released
			keysAndMouseState->leftButtonIsPressed=false;
			cameraPosition->oldRoll = cameraPosition->roll;
			cameraPosition->oldYaw = cameraPosition->yaw;
		}
	}
	else if (button == 3) {
		// Scroll up / zoom in
		cameraPosition->x -= zoomStep*cos((cameraPosition->roll-90)*2*M_PI/360)*sin(cameraPosition->yaw*2*M_PI/360);
		cameraPosition->y += zoomStep*cos((cameraPosition->roll-90)*2*M_PI/360)*cos(cameraPosition->yaw*2*M_PI/360);
		cameraPosition->z += zoomStep*sin((cameraPosition->roll-90)*2*M_PI/360);
	}
	else if(button == 4) {
		// Zoom out
		cameraPosition->x += zoomStep*cos((cameraPosition->roll-90)*2*M_PI/360)*sin(cameraPosition->yaw*2*M_PI/360);
		cameraPosition->y -= zoomStep*cos((cameraPosition->roll-90)*2*M_PI/360)*cos(cameraPosition->yaw*2*M_PI/360);
		cameraPosition->z -= zoomStep*sin((cameraPosition->roll-90)*2*M_PI/360);
	}
}

int main(int argc, char** argv)
{
	// Allocate all the memory the program will need
	allocateMemory();

	// Set up the UDP connection with the lidar sensor:
	LidarUDPReceiver lidarUDPReceiver(LIDAR_UDP_PORT);
	pthread_t udpReceiverThreadID = lidarUDPReceiver.startReceiverThread(memoryPointers->rawLidarData);

	memoryPointers->locationLidarData[0].x=0.1;
	memoryPointers->locationLidarData[0].y=0.1;
	memoryPointers->locationLidarData[0].z=0.1;
	memoryPointers->locationLidarData[1].x=0.2;
	memoryPointers->locationLidarData[1].y=0.2;
	memoryPointers->locationLidarData[1].z=0.2;

	// Init glut and create window
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowPosition(0,0);
	glutInitWindowSize(100,100);
	glutCreateWindow("Lidar 3D visualization");
	glutFullScreen();

	// Set up the openGL projection matrix and vertex array pointer
	setUpDisplay();

	// Register callbacks
	glutDisplayFunc(drawDisplay);
	glutKeyboardFunc(handleKeyboard);
	glutMouseFunc(handleMouseClick);
	glutMotionFunc(handleMouseMove);
	glutTimerFunc(0,updateFrame,0); // The frame update function (all the work is carried out here)

	// Set glut and opengl options:
	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE,GLUT_ACTION_GLUTMAINLOOP_RETURNS);
	glEnable(GL_DEPTH_TEST);

	// Enter GLUT event processing cycle
	glutMainLoop();

	// Free allocated data
	//freeMemory();

	// Exit the UDP receiver thread
	lidarUDPReceiver.setThreadExitFlag();
	pthread_join(udpReceiverThreadID,NULL);
	printf("Main exited\n");
	return 0;
}

