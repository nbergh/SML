#ifndef UDPRECEIVER_H_
#define UDPRECEIVER_H_

// An abstract base-class that runs a thread receiving data on UDP

#include <pthread.h>
#include <sys/socket.h>
#include <arpa/inet.h>

class UDPReceiver {

protected:
	bool stopReceiverThread; // Flag for exiting the thread
	pthread_t receiverThreadID;

	// Socket variables:
	int udpSocketID;
	struct sockaddr_in socketAddressMe, socketAddressOther; // Socket addresses
	socklen_t socketLength;

	// Const and dest protected so base class cannot be instantiated or deleted
	UDPReceiver(const int port);
	~UDPReceiver();

};

#endif /* UDPRECEIVER_H_ */
