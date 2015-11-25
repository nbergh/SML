#include <GL/freeglut.h>
#include <math.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include "Headers/Parameters.h"
#include "Headers/Structs.h"
#include "Headers/Graphics.h"

// This class does all the graphics used by RCVdriver


Graphics::Graphics() {
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
	Graphics* thisPointer = (Graphics*)arg;

	bool& stopGraphicsThread = thisPointer->stopGraphicsThread;



	printf("%s\n","Graphics thread exited");
	pthread_exit(NULL);
}


namespace { // Graphics namespace
	struct CameraPosition {
		float x;
		float y;
		float z;
		float roll; // The rotation along the x axis
		float yaw; // The rotation around the z axis
		float oldRoll; // The value of roll as it was when it was last modified
		float oldYaw; // The value of yaw as it was when it was last modified
	};
	struct KeysAndMouseState {
		// Keys and mouse state for openGL visualization
		bool leftButtonIsPressed,forwardKeyIsPressed,backwardKeyIsPressed,leftStrafeKeyIsPressed,rightStrafeKeyIsPressed,upStrafeKeyIsPressed,downStrafeKeyIsPressed;
		int mousePosXwhenPressed;
		int mousePosYwhenPressed;
	};

	void moveCameraStep(float stepLength, int direction);
	void updateCameraPositionAccordingToKeys(float timeSinceLastCall);
	void handleKeyDown(unsigned char key, int x, int y);
	void handleKeyUp(unsigned char key, int x, int y);
	void handleMouseMove(int x, int y);
	void handleMouseClick(int button, int state, int x, int y);
	void updateFrame(int arg);
	void drawDisplay(void);

	CameraPosition* cameraPosition;
	KeysAndMouseState* keysAndMouseState;
	LidarDataPoint* lidarDataPoints;
	ObstaclePoint* obstacleSquares;
	int currentNrOfObstacles;

	void startGraphics(const LidarDataPoint* lidarDataPoints, const ObstaclePoint* obstacleSquares, const int& currentNrOfObstacles) {
		keysAndMouseState = new KeysAndMouseState();
		cameraPosition = new CameraPosition();
		cameraPosition->z = 5; // Set camera position to start at z=5

		// Init glut and create window
		//glutInit(&argc, argv);
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
		glutTimerFunc(0,updateFrame,GRAPHICS_FRAME_RATE); // The frame update function

		// Set glut and opengl options:
		glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE,GLUT_ACTION_GLUTMAINLOOP_RETURNS);
		glEnable(GL_DEPTH_TEST);

		// Set up the openGL projection matrix
		glMatrixMode(GL_PROJECTION);glLoadIdentity(); // Load the Projection Matrix
		glViewport(0, 0, glutGet(GLUT_SCREEN_WIDTH), glutGet(GLUT_SCREEN_HEIGHT)); 	// Set the viewport to be the entire window
		gluPerspective(45.0f, glutGet(GLUT_SCREEN_WIDTH)/(double) glutGet(GLUT_SCREEN_HEIGHT), 0.1f, 100.0f); // Set the correct perspective.

		// Enter GLUT event processing cycle
		glutMainLoop();

		// When glutMainLoop exits (which it does when the user press 'q' when the window is active):
		delete keysAndMouseState;
		delete cameraPosition;
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

	void updateFrame(int arg) {
		// Very simple frame updater function, just registeres a callback to itself, and tells openGL to call drawDisplay as soon as possible
		glutTimerFunc(1000.0/GRAPHICS_FRAME_RATE,updateFrame,0);

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
		glColor3f(1.0f,1.0f,1.0f); // Set to white
		glPointSize(2.0);
		glVertexPointer(3, GL_FLOAT, sizeof(LidarDataPoint), &lidarDataPoints->x);
		glDrawArrays( GL_POINTS, 0, 28800 ); // Draws all the points from the LIDAR

		// Draw the obstacle squares:
		glColor3f(1.0f,0.0f,0.0f); // Set the color of all the obstaclesquares to red
		glVertexPointer(3,GL_FLOAT,sizeof(ObstaclePoint),&obstacleSquares->x);
		//glDrawElements(GL_TRIANGLES,6*memoryPointers->currentNrOfObstacles,GL_UNSIGNED_INT,memoryPointers->obstacleSquareIndexesArray);
		// TODO switch to glDrawArrays(GL_TRIANGLE_STRIP,,,,)

		glDisableClientState( GL_VERTEX_ARRAY );

		//glFlush();
		glutSwapBuffers();
	}
}
