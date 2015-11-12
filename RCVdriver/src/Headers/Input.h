#ifndef INPUT_H_
#define INPUT_H_

#include <pthread.h>
#include "PathPlanning.h"

class Input {
	//Externally located:
	VehicleStatus& vehicleStatus;
	PathPlanning& pathPlanning; // Pointer to the pathPlanning object, so the parser can call its public functions

	//Internally located:
	bool stopParserThread;
	pthread_t parserThreadID;

	void startParserThread();
	friend void* parserThreadFunction(void* arg);

public:
	Input(VehicleStatus& vehicleStatus, PathPlanning& pathPlanning);
	~Input();
};



#endif /* INPUT_H_ */
