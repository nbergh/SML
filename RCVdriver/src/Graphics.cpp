#include "Headers/Graphics.h"
#include "Headers/Parameters.h"

#include <GL/freeglut.h>
#include <math.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

// This class does all the graphics used by RCVdriver

Graphics* Graphics::thisPointer;

Graphics::Graphics(const LidarExportData& lidarExportData, const PathExportData& pathExportData) :
		lidarExportData(lidarExportData),pathExportData(pathExportData),
		cameraPosition(),keysAndMouseState() {

	thisPointer=this; // Set static pointer to this, for glut callback function access to this
	startGraphicsThread();
}

Graphics::~Graphics() {
	// Stop the thread and wait for it to exit
	stopGraphicsThread=true;
	pthread_join(graphicsThreadID,NULL);
}

void Graphics::startGraphicsThread() {
	stopGraphicsThread=false;
	//Start the receiver thread
	if(pthread_create(&graphicsThreadID,NULL,graphicsThreadFunction,this)) {
		printf("%s%s\n","Unable to create thread: ", strerror(errno));
		exit(-1);
	}
}

void* Graphics::graphicsThreadFunction(void* arg) {
	thisPointer->cameraPosition.z = 5; // Set camera position to start at z=5

	// Init glut and create window
	int argc = 0;
	char* argv = NULL;
	glutInit(&argc, &argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowPosition(0,0);
	glutInitWindowSize(1000,1000);
	glutCreateWindow("Lidar 3D visualization");
//		glutFullScreen();

	// Register callbacks
	glutDisplayFunc(drawDisplay);
	glutKeyboardFunc(handleKeyDown);
	glutKeyboardUpFunc(handleKeyUp);
	glutMouseFunc(handleMouseClick);
	glutMotionFunc(handleMouseMove);
	glutTimerFunc(0,updateFrame,0); // The frame update function

	// Set glut and opengl options:
	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE,GLUT_ACTION_GLUTMAINLOOP_RETURNS);
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_FLAT);

	// Set up the openGL projection matrix
	glMatrixMode(GL_PROJECTION);glLoadIdentity(); // Load the Projection Matrix
	glViewport(0, 0, glutGet(GLUT_SCREEN_WIDTH), glutGet(GLUT_SCREEN_HEIGHT)); 	// Set the viewport to be the entire window
	gluPerspective(45, glutGet(GLUT_SCREEN_WIDTH)/(double) glutGet(GLUT_SCREEN_HEIGHT), 0.1, 100); // Set the correct perspective.

	// Enter GLUT event processing cycle
	glutMainLoop();

	// When glutMainLoop exits (which it does when the user press 'q' when the window is active):

	printf("%s\n","Graphics thread exited");
	pthread_exit(NULL);
}

void Graphics::updateCameraPositionAccordingToKeys(float timeSinceLastCall) {
	float stepLength = 2*timeSinceLastCall;

	if (keysAndMouseState.forwardKeyIsPressed) {moveCameraStep(stepLength,0);}
	else if (keysAndMouseState.backwardKeyIsPressed) {moveCameraStep(stepLength,1);}
	else if (keysAndMouseState.leftStrafeKeyIsPressed) {moveCameraStep((stepLength/8.0),2);}
	else if (keysAndMouseState.rightStrafeKeyIsPressed) {moveCameraStep((stepLength/8.0),3);}
	else if (keysAndMouseState.upStrafeKeyIsPressed) {moveCameraStep((stepLength/2.0),4);}
	else if (keysAndMouseState.downStrafeKeyIsPressed) {moveCameraStep((stepLength/2.0),5);}
}

void Graphics::moveCameraStep(float stepLength, int direction) {
	if (direction==0) {
		// Forwards
		cameraPosition.x -= stepLength*cos((cameraPosition.roll-90)*2*M_PI/360)*sin(cameraPosition.yaw*2*M_PI/360);
		cameraPosition.y += stepLength*cos((cameraPosition.roll-90)*2*M_PI/360)*cos(cameraPosition.yaw*2*M_PI/360);
		cameraPosition.z += stepLength*sin((cameraPosition.roll-90)*2*M_PI/360);
	}
	else if(direction==1) {
		//Backwards
		cameraPosition.x += stepLength*cos((cameraPosition.roll-90)*2*M_PI/360)*sin(cameraPosition.yaw*2*M_PI/360);
		cameraPosition.y -= stepLength*cos((cameraPosition.roll-90)*2*M_PI/360)*cos(cameraPosition.yaw*2*M_PI/360);
		cameraPosition.z -= stepLength*sin((cameraPosition.roll-90)*2*M_PI/360);
	}
	else if(direction==2) {
		//Strafe left
		cameraPosition.x -= (2*M_PI/360)*sin((cameraPosition.yaw+90)*2*M_PI/360);
		cameraPosition.y += (2*M_PI/360)*cos((cameraPosition.yaw+90)*2*M_PI/360);
	}
	else if(direction==3) {
		//Strafe right
		cameraPosition.x += (2*M_PI/360)*sin((cameraPosition.yaw+90)*2*M_PI/360);
		cameraPosition.y -= (2*M_PI/360)*cos((cameraPosition.yaw+90)*2*M_PI/360);
	}
	else if(direction==4) {
		// Strafe up
		cameraPosition.x += stepLength*cos((cameraPosition.roll-180)*2*M_PI/360)*sin(cameraPosition.yaw*2*M_PI/360);
		cameraPosition.y -= stepLength*cos((cameraPosition.roll-180)*2*M_PI/360)*cos(cameraPosition.yaw*2*M_PI/360);
		cameraPosition.z -= stepLength*sin((cameraPosition.roll-180)*2*M_PI/360);
	}
	else if(direction==5) {
		// Strafe down
		cameraPosition.x -= stepLength*cos((cameraPosition.roll-180)*2*M_PI/360)*sin(cameraPosition.yaw*2*M_PI/360);
		cameraPosition.y += stepLength*cos((cameraPosition.roll-180)*2*M_PI/360)*cos(cameraPosition.yaw*2*M_PI/360);
		cameraPosition.z += stepLength*sin((cameraPosition.roll-180)*2*M_PI/360);
	}
}

void Graphics::updateFrame(int arg) {
	// Very simple frame updater function, just registeres a callback to itself, and tells openGL to call drawDisplay as soon as possible
	if (thisPointer->stopGraphicsThread) {glutLeaveMainLoop();}
	glutTimerFunc(1000.0/GRAPHICS_FRAME_RATE,updateFrame,0);

	float timeSinceLastCall=0.01;
	thisPointer->updateCameraPositionAccordingToKeys(timeSinceLastCall);

	glutPostRedisplay();
}

void Graphics::drawDisplay() {
	// This function is called by openGL when it decides that the window needs to be redrawn

	// Load the modelview matrix and change it according to the position of the camera
	glMatrixMode(GL_MODELVIEW);glLoadIdentity();
	// Change modelview according to the camera position (inverted because when we want to move the camera somewhere, we move the model in the other direction)
	glRotatef(-thisPointer->cameraPosition.roll, 1,0,0);
	glRotatef(-thisPointer->cameraPosition.yaw, 0,0,1);
	glTranslatef(-thisPointer->cameraPosition.x, -thisPointer->cameraPosition.y, -thisPointer->cameraPosition.z);

	// Clear Color and Depth Buffers and enable vertex drawing
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnableClientState( GL_VERTEX_ARRAY );

	// Draw the lidar points:
	glColor3f(1,1,1); // Set to white
	glPointSize(2.0);
	glVertexPointer(3, GL_FLOAT, sizeof(LidarDataPoint), &thisPointer->lidarExportData.lidarDataPoints->x);
	glDrawArrays( GL_POINTS, 0, 28800 ); // Draws all the points from the LIDAR

	// Draw the obstacle squares:
	glColor3f(1,0,0); // Set the color of all the obstaclesquares to red
	glVertexPointer(2,GL_FLOAT,sizeof(ObstaclePoint),&thisPointer->lidarExportData.obstacleSquares->x);
	glDrawArrays(GL_QUADS,0,4*thisPointer->lidarExportData.currentNrOfObstacles);

	//Draw the paths:
	glEnableClientState(GL_COLOR_ARRAY);
	glColorPointer(3,GL_UNSIGNED_BYTE,sizeof(PathPointInLocalXY),&thisPointer->pathExportData.macroPathXY->b);
	glVertexPointer(2,GL_FLOAT,sizeof(PathPointInLocalXY),&thisPointer->pathExportData.macroPathXY->x);
	glDrawArrays(GL_LINE_STRIP,thisPointer->pathExportData.currentIndexInMacroPath-1,thisPointer->pathExportData.lengthOfMacroPath-(thisPointer->pathExportData.currentIndexInMacroPath-1));

	glColorPointer(3,GL_UNSIGNED_BYTE,sizeof(PathPointInLocalXY),&thisPointer->pathExportData.microPathXY->b);
	glVertexPointer(2,GL_FLOAT,sizeof(PathPointInLocalXY),&thisPointer->pathExportData.microPathXY->x);
	glDrawArrays(GL_LINE_STRIP,thisPointer->pathExportData.currentIndexInMicroPath-1,thisPointer->pathExportData.lengthOfMicroPath-(thisPointer->pathExportData.currentIndexInMicroPath-1));

	glDisableClientState(GL_COLOR_ARRAY);
	glDisableClientState(GL_VERTEX_ARRAY);

	//glFlush();
	glutSwapBuffers();
}

void Graphics::handleKeyDown(unsigned char key, int x, int y) {
//	if (key=='q') {glutLeaveMainLoop();} // Exit the program

	if (key=='w') {thisPointer->keysAndMouseState.forwardKeyIsPressed=true;}
	else if (key=='s') {thisPointer->keysAndMouseState.backwardKeyIsPressed=true;}
	else if (key=='a') {thisPointer->keysAndMouseState.leftStrafeKeyIsPressed=true;}
	else if (key=='d') {thisPointer->keysAndMouseState.rightStrafeKeyIsPressed=true;}
	else if (key=='r') {thisPointer->keysAndMouseState.upStrafeKeyIsPressed=true;}
	else if (key=='f') {thisPointer->keysAndMouseState.downStrafeKeyIsPressed=true;}
}

void Graphics::handleKeyUp(unsigned char key, int x, int y) {
	if (key=='w') {thisPointer->keysAndMouseState.forwardKeyIsPressed=false;}
	else if (key=='s') {thisPointer->keysAndMouseState.backwardKeyIsPressed=false;}
	else if (key=='a') {thisPointer->keysAndMouseState.leftStrafeKeyIsPressed=false;}
	else if (key=='d') {thisPointer->keysAndMouseState.rightStrafeKeyIsPressed=false;}
	else if (key=='r') {thisPointer->keysAndMouseState.upStrafeKeyIsPressed=false;}
	else if (key=='f') {thisPointer->keysAndMouseState.downStrafeKeyIsPressed=false;}
}

void Graphics::handleMouseMove(int x, int y) {
	if (thisPointer->keysAndMouseState.leftButtonIsPressed) {
		thisPointer->cameraPosition.yaw = thisPointer->cameraPosition.oldYaw + 0.2*(thisPointer->keysAndMouseState.mousePosXwhenPressed-x);
		thisPointer->cameraPosition.roll = thisPointer->cameraPosition.oldRoll + 0.2*(thisPointer->keysAndMouseState.mousePosYwhenPressed-y);
	}
}

void Graphics::handleMouseClick(int button, int state, int x, int y) {
	float scrollStepLength =0.1;

	if (button == GLUT_LEFT_BUTTON) {
		if (state == GLUT_DOWN) {
			// Left mouse button is pressed
			thisPointer->keysAndMouseState.leftButtonIsPressed=true;
			thisPointer->keysAndMouseState.mousePosXwhenPressed=x;
			thisPointer->keysAndMouseState.mousePosYwhenPressed=y;
		}
		else  {
			// Left mouse button is released
			thisPointer->keysAndMouseState.leftButtonIsPressed=false;
			thisPointer->cameraPosition.oldRoll = thisPointer->cameraPosition.roll;
			thisPointer->cameraPosition.oldYaw = thisPointer->cameraPosition.yaw;
		}
	}
	else if (button == 3) {
		// Scroll up / move camera forwards
		thisPointer->moveCameraStep(scrollStepLength,0);
	}
	else if(button == 4) {
		// Zoom out / move camera backwards
		thisPointer->moveCameraStep(scrollStepLength,1);
	}
}


