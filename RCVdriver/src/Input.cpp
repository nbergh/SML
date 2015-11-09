#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "Headers/Input.h"

void* parserThreadFunction(void* objectPointer);
bool compareStrings (const char* string1, const char* string2, int lengthOfString);

Input::Input() {
	// Allocate all the data first
	inputCommand = new char[50];
	startParserThread();
}

Input::~Input() {
	// Stop the parser thread, and wait for it to exit
	stopParserThread=true;
	pthread_join(parserThreadID,NULL);

	delete[] inputCommand;
}

void Input::startParserThread() {
	stopParserThread=false;

	//Start the receiver thread
	if(pthread_create(&parserThreadID,NULL,parserThreadFunction,this)) {
		printf("%s%s\n","Unable to create thread: ", strerror(errno));
		exit(-1);
	}
}

void* parserThreadFunction(void* objectPointer) {
	// Read command from stdin. Has to be a non-member, since the argument function to pthread_create only accepts non-member functions
	char *inputCommand = ((Input*)objectPointer)->inputCommand;
	bool *stopParserThread = &((Input*)objectPointer)->stopParserThread;

	while(!(*stopParserThread)) {
		printf("%s","RCV>");
		if (fgets(inputCommand,50,stdin)==0) {
			printf("%s%s\n","Read error: ",strerror(errno));
			continue;
		}
		if ((*inputCommand)==10) {continue;} // Empty command

		if (compareStrings(inputCommand,"loadpath ",9)) {
			printf("%s",inputCommand+9);
			continue;
		}

		printf("%s%s","Unknown command: ",inputCommand);
	}

	printf("%s\n","Parser thread exited");
	pthread_exit(NULL);
}

bool compareStrings (const char* string1, const char* string2, int lengthOfString) {
	// Compares string1 and string2 for lengthOfString characters, and returns true if they are the same, false otherwise
	for (int i=0;i<lengthOfString;i++) {
		if (*(string1+i) != *(string2+i)) {return false;}
	}
	return true;
}











