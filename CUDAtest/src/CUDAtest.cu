#include <GL/glut.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#define FRAMERATE 50
#define WINDOW_HEIGHT 500
#define WINDOW_WIDTH 500

struct CameraPosition {
	float x;
	float y;
	float z;
	float pitch; // The rotation along the x axis
	float yaw; // The rotation along the y axis
	float oldPitch; // The value of pitch as it was when it was last modified
	float oldYaw; // The value of yaw as it was when it was last modified
};

struct KeysAndMouseState {
	bool leftButtonIsPressed;
	bool hasScrolledUp;
	bool hasScrolledDown;
	int mousePosXwhenPressed;
	int mousePosYwhenPressed;
};

struct Point {
	// A simple struct representing one lidar data point
    float x, y,z;
};

CameraPosition* cameraPosition;
KeysAndMouseState* keysAndMouseState;
Point* points;

void updateFrame(int data) {
	glutTimerFunc(1000/FRAMERATE,updateFrame,0); // Call again in 1000/FRAMERATE milliseconds

	// This functions moves the camera according to the user input
	float zoomStep =1.0f;

	if (keysAndMouseState->hasScrolledUp) {
		keysAndMouseState->hasScrolledUp=false;
		cameraPosition->z += zoomStep*cos(cameraPosition->pitch*2*M_PI/360)*cos(cameraPosition->yaw*2*M_PI/360);
		cameraPosition->x += zoomStep*cos(cameraPosition->pitch*2*M_PI/360)*sin(cameraPosition->yaw*2*M_PI/360);
		cameraPosition->y += zoomStep*sin(cameraPosition->pitch*2*M_PI/360);
	}
	else if (keysAndMouseState->hasScrolledDown) {
		keysAndMouseState->hasScrolledDown=false;
		cameraPosition->z -= zoomStep*cos(cameraPosition->pitch*2*M_PI/360)*cos(cameraPosition->yaw*2*M_PI/360);
		cameraPosition->x -= zoomStep*cos(cameraPosition->pitch*2*M_PI/360)*sin(cameraPosition->yaw*2*M_PI/360);
		cameraPosition->y -= zoomStep*sin(cameraPosition->pitch*2*M_PI/360);
	}

	//printf("%s%f%s%f\n","Pitch: ",cameraPosition->pitch,", yaw: ",cameraPosition->yaw);
	//printf("%s%f%s%f%s%f\n","X: ",cameraPosition->x,", y: ",cameraPosition->y,", z: ",cameraPosition->z);

	glutPostRedisplay();
}


void display(void) {
	// Load the Projection Matrix
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	// Set the viewport to be the entire window
	glViewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
	// Set the correct perspective.
	gluPerspective(45.0f, 1, 0.1f, 100.0f);

	// Get Back to the Modelview
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	// Change modelview according to the camera position (inverted because when we want to move the camera somewhere, we move the model in the other direction)
	glRotatef(-cameraPosition->pitch, 1.0f, 0.0f, 0.0f);
	glRotatef(-(cameraPosition->yaw-180), 0.0f, 1.0f, 0.0f);
	glTranslatef(-cameraPosition->x, -cameraPosition->y, -cameraPosition->z);

	// Clear Color and Depth Buffers
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Draw code:
    glEnableClientState( GL_VERTEX_ARRAY );
    glVertexPointer( 3, GL_FLOAT, sizeof(Point), &points[0].x );
    glPointSize( 3.0 );
    glDrawArrays( GL_POINTS, 0, 2 );
    glDisableClientState( GL_VERTEX_ARRAY );

    glFlush();

	glutSwapBuffers();
}

void registerMouseMove(int x, int y) {
	if (keysAndMouseState->leftButtonIsPressed) {
		cameraPosition->yaw = cameraPosition->oldYaw + 0.2f*(keysAndMouseState->mousePosXwhenPressed-x);
		cameraPosition->pitch = cameraPosition->oldPitch + 0.2f*(keysAndMouseState->mousePosYwhenPressed-y);
	}
}

void registerMouseClick(int button, int state, int x, int y) {
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
			cameraPosition->oldPitch = cameraPosition->pitch;
			cameraPosition->oldYaw = cameraPosition->yaw;
		}
	}
	else if (button == 3) {
		// Scroll up / zoom in
		keysAndMouseState->hasScrolledUp=true;
	}
	else if(button == 4) {
		// Zoom out
		keysAndMouseState->hasScrolledDown=true;
	}

}

int main(int argc, char **argv) {
	// Initialize structs:
	cameraPosition = new CameraPosition;
	memset(cameraPosition,0,sizeof(CameraPosition));
	keysAndMouseState = new KeysAndMouseState;
	memset(keysAndMouseState,0,sizeof(KeysAndMouseState));

	// Initialize some points:
	points = new Point[2];
	points[0].x=0.1;
	points[0].y=0.1;
	points[0].z=0.1;
	points[1].x=0.2;
	points[1].y=0.2;
	points[1].z=0.2;

	// Init glut and create window
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowPosition(0,0);
	glutInitWindowSize(WINDOW_WIDTH,WINDOW_HEIGHT);
	glutCreateWindow("OpenGL 3D world");

	// Register callbacks
	glutDisplayFunc(display);
	glutMouseFunc(registerMouseClick);
	glutMotionFunc(registerMouseMove);
	glutTimerFunc(0,updateFrame,0);

	// OpenGL init
	glEnable(GL_DEPTH_TEST);

	// enter GLUT event processing cycle
	glutMainLoop();

	return 1;
}
