#ifndef INPUT_H_
#define INPUT_H_

#include <pthread.h>

class Input {
	char* inputCommand;
	bool stopParserThread;
	pthread_t parserThreadID;

	void startParserThread();
	friend void* parserThreadFunction(void* arg);

public:
	Input();
	~Input();

};



#endif /* INPUT_H_ */
