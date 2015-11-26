#ifndef INPUT_H_
#define INPUT_H_

#include <pthread.h>
#include "PathPlanning.h"

class Input {
	//Externally located:
	VehicleStatus& vehicleStatus;
	PathPlanning& pathPlanning; // Pointer to the pathPlanning object, so the parser can call its public functions

	//Internally located:
	bool stopParserThread, exitProgram;
	pthread_t parserThreadID;

	void startParserThread();
	bool compareStrings (const char* string1, const char* string2, int lengthOfString);
	void combineStrings (const char* string1,const char* string2, char* location, int maxLength);
	static void* parserThreadFunction(void* arg);

public:
	Input(VehicleStatus& vehicleStatus, PathPlanning& pathPlanning);
	~Input();
	bool getExitProgramBool() {return exitProgram;}
};



#endif /* INPUT_H_ */
