#ifndef GRAPHICS_H_
#define GRAPHICS_H_

#include <pthread.h>

class Graphics {
	bool stopGraphicsThread;
	pthread_t graphicsThreadID;

	void startGraphicsThread();
	static void* graphicsThreadFunction(void* arg);

public:
	Graphics();
	~Graphics();

};

#endif /* GRAPHICS_H_ */
