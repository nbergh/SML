#ifndef GRAPHICS_H_
#define GRAPHICS_H_

#include "Structs.h"

#include <pthread.h>

class Graphics {
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

	static Graphics* thisPointer; // Cant be a reference

	const LidarExportData& lidarExportData;
	const PathExportData& pathExportData;
	CameraPosition cameraPosition;
	KeysAndMouseState keysAndMouseState;
	bool stopGraphicsThread;
	pthread_t graphicsThreadID;

	void startGraphicsThread();
	void moveCameraStep(float stepLength, int direction);
	void updateCameraPositionAccordingToKeys(float timeSinceLastCall);
	static void* graphicsThreadFunction(void* arg);
	// GLUT callbacks:
	static void updateFrame(int arg);
	static void drawDisplay();
	static void handleKeyDown(unsigned char key, int x, int y);
	static void handleKeyUp(unsigned char key, int x, int y);
	static void handleMouseMove(int x, int y);
	static void handleMouseClick(int button, int state, int x, int y);

public:
	Graphics(const LidarExportData& lidarExportData, const PathExportData& pathExportData);
	~Graphics();

};

#endif /* GRAPHICS_H_ */
