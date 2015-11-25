#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "Headers/Input.h"

Input::Input(VehicleStatus& vehicleStatus, PathPlanning& pathPlanning):
		vehicleStatus(vehicleStatus),
		pathPlanning(pathPlanning),
		stopMainControllerLoop(false) {
	startParserThread();
}

Input::~Input() {
	// Stop the parser thread, and wait for it to exit
	stopParserThread=true;
	pthread_join(parserThreadID,NULL);
}

void Input::startParserThread() {
	stopParserThread=false;

	//Start the receiver thread
	if(pthread_create(&parserThreadID,NULL,parserThreadFunction,this)) {
		printf("%s%s\n","Unable to create thread: ", strerror(errno));
		exit(-1);
	}
}

bool Input::compareStrings (const char* string1, const char* string2, int lengthOfString) {
	// Compares string1 and string2 for lengthOfString characters, and returns true if they are the same, false otherwise
	for (int i=0;i<lengthOfString;i++) {
		if (*(string1+i) != *(string2+i)) {return false;}
	}
	return true;
}

void Input::combineStrings (const char* string1,const char* string2, char* location, int maxLength) {
	// Combines string1 and string2 into one string, ignoring newlines in string2 and adding null at the end
	int strIndex1=0,strIndex2=0;
	while(strIndex1<maxLength-1 && *(string1+strIndex1)!=0) {
		*(location+strIndex1) = *(string1+strIndex1);
		strIndex1++;
	}
	while(strIndex1+strIndex2<maxLength-1 && *(string2+strIndex2)!=10) {
		*(location+strIndex1+strIndex2) = *(string2+strIndex2);
		strIndex2++;
	}
	*(location+strIndex1+strIndex2)=0; // Null terminated string
}

void* Input::parserThreadFunction(void* arg) {
	// Read command from stdin.
	Input* thisPointer = (Input*) arg;

	PathPlanning& pathPlanning = thisPointer->pathPlanning;
	VehicleStatus& vehicleStatus = thisPointer->vehicleStatus;
	bool& stopParserThread = thisPointer->stopParserThread;
	char outputCommand[50], inputCommand[50];

	while(!stopParserThread) {
		printf("%s","RCV>");
		if (fgets(inputCommand,50,stdin)==0) {
			printf("%s%s\n","Read error: ",strerror(errno));
			continue;
		}
		if ((*inputCommand)==10) {continue;} // Empty command

		if (thisPointer->compareStrings(inputCommand,"loadpath ",9)) {
			// Call setMacroPath in pathPlanning
			thisPointer->combineStrings("./Paths/",inputCommand+9,outputCommand,50);
			pathPlanning.setMacroPath(outputCommand);
			continue;
		}
		if (thisPointer->compareStrings(inputCommand,"start",5)) {
			if(!vehicleStatus.isRunning) {printf("%s\n","RCV started");}
			vehicleStatus.isRunning=true;
			continue;
		}
		if (thisPointer->compareStrings(inputCommand,"stop",4)) {
			if (vehicleStatus.isRunning) {printf("%s\n","RCV stopped");}
			vehicleStatus.isRunning=false;
			continue;
		}
		if (thisPointer->compareStrings(inputCommand,"exit",4)) {
			printf("%s\n","Program exiting");
			thisPointer->stopMainControllerLoop=true;
			break;
		}

		printf("%s%s","Unknown command: ",inputCommand);
	}

	printf("%s\n","Parser thread exited");
	pthread_exit(NULL);
}

bool Input::getStopMainControllerLoop() {
	return stopMainControllerLoop;
}


