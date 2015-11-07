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
#include <unistd.h>

#include "Headers/StructDefinitions.h"
#include "Headers/LidarUDPReceiver.h"
#include "Headers/CudaFunctions.h"
#include "Headers/CudaErrorCheckFunctions.h"

#define LIDAR_UDP_PORT 2368 // UDP listening port
#define FRAMERATE 60 // The framerate for the openGL graphics
#define MAX_OBSTACLE_DETECTION_DISTANCE 20 // The maximal distance (in meters) that an obstacle can be identified in
#define OBSTACLE_POINT_SIDE_LENGTH 0.05 // The length of one side in the square that represents an obstacle point (decreasing this value by a factor of X will require X² times more device memory)
#define MIN_OBSTACLE_DELTA_Z 0.1 // The difference in z coordinates (in meters) required for two points with the same x and y coordinates to be registered as an obstacle
#define MAX_NUMBER_OF_OBSTACLES 5000 //The maximal number of obstacle points that can be displayed
#define POINTS_IN_PATH 40 // The number of points to be used in the vehicle path

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
	cameraPosition->z=5; // Start with the camera at z=5;

	// First is rawLidarData. It is a byte array of raw UDP data (minus UDP headers and factory bytes), representing 75 UDP packets (one revolution of the lidar sensor). Size is 1200*75 = 90000 bytes
	memoryPointers->sizeOfRawLidarData = 90000;
	CUDA_CHECK_RETURN(cudaMallocHost((void**)&memoryPointers->rawLidarData,memoryPointers->sizeOfRawLidarData));
	CUDA_CHECK_RETURN(cudaMalloc((void**)&memoryPointers->rawLidarDataOnDevice,memoryPointers->sizeOfRawLidarData));

	// Second is the locationLidarData. This is an array of OpenGlvertex structs, that contain values for x,y,z for 28800 points (one revolution)
	memoryPointers->sizeOfLidarPoints = 28800*sizeof(OpenGLvertex);
	CUDA_CHECK_RETURN(cudaMallocHost((void**)&memoryPointers->lidarPoints,memoryPointers->sizeOfLidarPoints));
	CUDA_CHECK_RETURN(cudaMalloc((void**)&memoryPointers->lidarPointsOnDevice,memoryPointers->sizeOfLidarPoints));

	// Third is the data that hold the obstacles. Each obstacle is represented as a square (4 vertices) in the graphics
	// and 6 ints of index data needed by glDrawElements
	memoryPointers->sizeOfObstacleSquares = MAX_NUMBER_OF_OBSTACLES*4*sizeof(OpenGLvertex);
	CUDA_CHECK_RETURN(cudaMallocHost((void**)&memoryPointers->obstacleSquares,memoryPointers->sizeOfObstacleSquares));
	CUDA_CHECK_RETURN(cudaMalloc((void**)&memoryPointers->obstacleSquaresOnDevice,memoryPointers->sizeOfObstacleSquares));
	memoryPointers->sizeOfObstacleSquareIndexesArray = MAX_NUMBER_OF_OBSTACLES*6*sizeof(GLuint);
	CUDA_CHECK_RETURN(cudaMallocHost((void**)&memoryPointers->obstacleSquareIndexesArray,memoryPointers->sizeOfObstacleSquareIndexesArray));

	// Allocate the two obstacle matrices on the device
	memoryPointers->numberOfMatrixFieldsPerSide = 2 * MAX_OBSTACLE_DETECTION_DISTANCE / OBSTACLE_POINT_SIDE_LENGTH; // As an integer
	memoryPointers->sizeOfObstacleMatrix = memoryPointers->numberOfMatrixFieldsPerSide * memoryPointers->numberOfMatrixFieldsPerSide*sizeof(int);
	CUDA_CHECK_RETURN(cudaMalloc((void**)&memoryPointers->obstacleMatrixForMaxZOnDevice,memoryPointers->sizeOfObstacleMatrix));
	CUDA_CHECK_RETURN(cudaMalloc((void**)&memoryPointers->obstacleMatrixForMinZOnDevice,memoryPointers->sizeOfObstacleMatrix));

	// Now zero out obstacleSquaresOnDevice and intialize obstacleSquareIndexes:
	CUDA_CHECK_RETURN(cudaMemset(memoryPointers->obstacleSquaresOnDevice,0,memoryPointers->sizeOfObstacleSquares));
	intializeObstacleSquareIndexesArray(memoryPointers,MAX_NUMBER_OF_OBSTACLES);

	// Allocate data for the path
	memoryPointers->sizeOfPathPoints = POINTS_IN_PATH * sizeof(OpenGLvertex);
	CUDA_CHECK_RETURN(cudaMallocHost((void**)&memoryPointers->pathPoints,memoryPointers->sizeOfPathPoints));
	memset(memoryPointers->pathPoints,0,memoryPointers->sizeOfPathPoints);

	// Allocate the vehicle state
	memoryPointers->vehicleState = new VehicleState;
	memset(memoryPointers->vehicleState,0,sizeof(VehicleState));
}

void freeMemory() {
	CUDA_CHECK_RETURN(cudaFreeHost((void*)memoryPointers->rawLidarData));
	CUDA_CHECK_RETURN(cudaFree(memoryPointers->rawLidarDataOnDevice));
	CUDA_CHECK_RETURN(cudaFreeHost((void*)memoryPointers->lidarPoints));
	CUDA_CHECK_RETURN(cudaFree(memoryPointers->lidarPointsOnDevice));
	CUDA_CHECK_RETURN(cudaFreeHost((void*)memoryPointers->obstacleSquares));
	CUDA_CHECK_RETURN(cudaFree(memoryPointers->obstacleSquaresOnDevice));
	CUDA_CHECK_RETURN(cudaFreeHost((void*)memoryPointers->obstacleSquareIndexesArray));
	CUDA_CHECK_RETURN(cudaFree(memoryPointers->obstacleMatrixForMaxZOnDevice));
	CUDA_CHECK_RETURN(cudaFree(memoryPointers->obstacleMatrixForMinZOnDevice));
	CUDA_CHECK_RETURN(cudaFreeHost(memoryPointers->pathPoints));
	CUDA_CHECK_RETURN(cudaFreeHost(memoryPointers->vehicleState));
	free(memoryPointers);
	free(cameraPosition);
	free(keysAndMouseState);
}

void moveCameraStep(float stepLength, int direction) {
	if (direction==0) {
		// Forwards
		cameraPosition->x -= stepLength*cos((cameraPosition->roll-90)*2*M_PI/360)*sin(cameraPosition->yaw*2*M_PI/360);
		cameraPosition->y += stepLength*cos((cameraPosition->roll-90)*2*M_PI/360)*cos(cameraPosition->yaw*2*M_PI/360);
		cameraPosition->z += stepLength*sin((cameraPosition->roll-90)*2*M_PI/360);
	}
	else if(direction==1) {
		//Backwards
		cameraPosition->x += stepLength*cos((cameraPosition->roll-90)*2*M_PI/360)*sin(cameraPosition->yaw*2*M_PI/360);
		cameraPosition->y -= stepLength*cos((cameraPosition->roll-90)*2*M_PI/360)*cos(cameraPosition->yaw*2*M_PI/360);
		cameraPosition->z -= stepLength*sin((cameraPosition->roll-90)*2*M_PI/360);
	}
	else if(direction==2) {
		//Strafe left
		cameraPosition->x -= (2*M_PI/360)*sin((cameraPosition->yaw+90)*2*M_PI/360);
		cameraPosition->y += (2*M_PI/360)*cos((cameraPosition->yaw+90)*2*M_PI/360);
	}
	else if(direction==3) {
		//Strafe right
		cameraPosition->x += (2*M_PI/360)*sin((cameraPosition->yaw+90)*2*M_PI/360);
		cameraPosition->y -= (2*M_PI/360)*cos((cameraPosition->yaw+90)*2*M_PI/360);
	}
	else if(direction==4) {
		// Strafe up
		cameraPosition->x += stepLength*cos((cameraPosition->roll-180)*2*M_PI/360)*sin(cameraPosition->yaw*2*M_PI/360);
		cameraPosition->y -= stepLength*cos((cameraPosition->roll-180)*2*M_PI/360)*cos(cameraPosition->yaw*2*M_PI/360);
		cameraPosition->z -= stepLength*sin((cameraPosition->roll-180)*2*M_PI/360);
	}
	else if(direction==5) {
		// Strafe down
		cameraPosition->x -= stepLength*cos((cameraPosition->roll-180)*2*M_PI/360)*sin(cameraPosition->yaw*2*M_PI/360);
		cameraPosition->y += stepLength*cos((cameraPosition->roll-180)*2*M_PI/360)*cos(cameraPosition->yaw*2*M_PI/360);
		cameraPosition->z += stepLength*sin((cameraPosition->roll-180)*2*M_PI/360);
	}
}

void updateCameraPositionAccordingToKeys(float timeSinceLastCall) {
	float stepLength = 2.0f*timeSinceLastCall;

	if (keysAndMouseState->forwardKeyIsPressed) {moveCameraStep(stepLength,0);}
	else if (keysAndMouseState->backwardKeyIsPressed) {moveCameraStep(stepLength,1);}
	else if (keysAndMouseState->leftStrafeKeyIsPressed) {moveCameraStep((stepLength/8.0),2);}
	else if (keysAndMouseState->rightStrafeKeyIsPressed) {moveCameraStep((stepLength/8.0),3);}
	else if (keysAndMouseState->upStrafeKeyIsPressed) {moveCameraStep((stepLength/2.0),4);}
	else if (keysAndMouseState->downStrafeKeyIsPressed) {moveCameraStep((stepLength/2.0),5);}
}

void handleKeyDown(unsigned char key, int x, int y) {
	if (key=='q') {glutLeaveMainLoop();} // Exit the program
	else if (key=='w') {keysAndMouseState->forwardKeyIsPressed=true;}
	else if (key=='s') {keysAndMouseState->backwardKeyIsPressed=true;}
	else if (key=='a') {keysAndMouseState->leftStrafeKeyIsPressed=true;}
	else if (key=='d') {keysAndMouseState->rightStrafeKeyIsPressed=true;}
	else if (key=='r') {keysAndMouseState->upStrafeKeyIsPressed=true;}
	else if (key=='f') {keysAndMouseState->downStrafeKeyIsPressed=true;}
}

void handleKeyUp(unsigned char key, int x, int y) {
	if (key=='w') {keysAndMouseState->forwardKeyIsPressed=false;}
	else if (key=='s') {keysAndMouseState->backwardKeyIsPressed=false;}
	else if (key=='a') {keysAndMouseState->leftStrafeKeyIsPressed=false;}
	else if (key=='d') {keysAndMouseState->rightStrafeKeyIsPressed=false;}
	else if (key=='r') {keysAndMouseState->upStrafeKeyIsPressed=false;}
	else if (key=='f') {keysAndMouseState->downStrafeKeyIsPressed=false;}
}

void handleMouseMove(int x, int y) {
	if (keysAndMouseState->leftButtonIsPressed) {
		cameraPosition->yaw = cameraPosition->oldYaw + 0.2f*(keysAndMouseState->mousePosXwhenPressed-x);
		cameraPosition->roll = cameraPosition->oldRoll + 0.2f*(keysAndMouseState->mousePosYwhenPressed-y);
	}
}

void handleMouseClick(int button, int state, int x, int y) {
	float scrollStepLength =0.1f;

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
		// Scroll up / move camera forwards
		moveCameraStep(scrollStepLength,0);
	}
	else if(button == 4) {
		// Zoom out / move camera backwards
		moveCameraStep(scrollStepLength,1);
	}
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
	identifyObstaclesInLidarData(memoryPointers,OBSTACLE_POINT_SIDE_LENGTH,MIN_OBSTACLE_DELTA_Z,MAX_NUMBER_OF_OBSTACLES);

	float timeSinceLastCall=0.01;
	updateCameraPositionAccordingToKeys(timeSinceLastCall);

	glutPostRedisplay();
}

void drawDisplay(void) {
	// This function is called by openGL when it decides that the window needs to be redrawn

	// Load the modelview matrix and change it according to the position of the camera
	glMatrixMode(GL_MODELVIEW);glLoadIdentity();
	// Change modelview according to the camera position (inverted because when we want to move the camera somewhere, we move the model in the other direction)
	glRotatef(-cameraPosition->roll, 1,0,0);
	glRotatef(-cameraPosition->yaw, 0,0,1);
	glTranslatef(-cameraPosition->x, -cameraPosition->y, -cameraPosition->z);

	// Clear Color and Depth Buffers and enable vertex drawing
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnableClientState( GL_VERTEX_ARRAY );

	// Draw the lidar points:
	glPointSize(2.0);
	glColor3f(1.0f,1.0f,1.0f); // Set to white
	glVertexPointer(3, GL_FLOAT, sizeof(OpenGLvertex), &memoryPointers->lidarPoints->x);
    glDrawArrays( GL_POINTS, 0, 28800 ); // Draws all the points from the LIDAR

    // Draw the path:
    glColor3f(1.0f,1.0f,0.0f); // Set to green
	glVertexPointer(3, GL_FLOAT, sizeof(OpenGLvertex), &memoryPointers->pathPoints->x);
    glDrawArrays( GL_POINTS, 0, POINTS_IN_PATH ); // Draw the path points

    glDisableClientState( GL_VERTEX_ARRAY );

    // Draw the obstacle squares:
    glColor3f(1.0f,0.0f,0.0f); // Set the color of all the obstaclesquares to red
    glVertexPointer(3,GL_FLOAT,sizeof(OpenGLvertex),&memoryPointers->obstacleSquares->x);
    glDrawElements(GL_TRIANGLES,6*memoryPointers->currentNrOfObstacles,GL_UNSIGNED_INT,memoryPointers->obstacleSquareIndexesArray);

    //glFlush();
	glutSwapBuffers();
}

void setUpDisplay() {
	// Load the Projection Matrix
	glMatrixMode(GL_PROJECTION);glLoadIdentity();
	// Set the viewport to be the entire window
	glViewport(0, 0, glutGet(GLUT_SCREEN_WIDTH), glutGet(GLUT_SCREEN_HEIGHT));
	// Set the correct perspective.
	gluPerspective(45.0f, glutGet(GLUT_SCREEN_WIDTH)/(double) glutGet(GLUT_SCREEN_HEIGHT), 0.1f, 100.0f);
}

int main(int argc, char** argv)
{
	// Allocate all the memory the program will need
	allocateMemory();

	// Set up the UDP connection with the lidar sensor:
	LidarUDPReceiver lidarUDPReceiver(LIDAR_UDP_PORT);
	pthread_t udpReceiverThreadID = lidarUDPReceiver.startReceiverThread(memoryPointers->rawLidarData);

//	// Init glut and create window
//	glutInit(&argc, argv);
//	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
//	glutInitWindowPosition(0,0);
//	glutInitWindowSize(1000,1000);
//	glutCreateWindow("Lidar 3D visualization");
//	glutFullScreen();
//
//	// Set up the openGL projection matrix
//	setUpDisplay();
//
//	// Register callbacks
//	glutDisplayFunc(drawDisplay);
//	glutKeyboardFunc(handleKeyDown);
//	glutKeyboardUpFunc(handleKeyUp);
//	glutMouseFunc(handleMouseClick);
//	glutMotionFunc(handleMouseMove);
//	glutTimerFunc(0,updateFrame,0); // The frame update function (all the work is carried out here)
//
//	// Set glut and opengl options:
//	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE,GLUT_ACTION_GLUTMAINLOOP_RETURNS);
//	glEnable(GL_DEPTH_TEST);
//
//	// Enter GLUT event processing cycle
//	glutMainLoop();



	// Exit the UDP receiver thread
	lidarUDPReceiver.setThreadExitFlag();
	pthread_join(udpReceiverThreadID,NULL);

	// Free allocated data
	freeMemory();
	printf("Main exited\n");
	return 0;
}

