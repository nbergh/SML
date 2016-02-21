#ifndef UDPRECEIVER_H_
#define UDPRECEIVER_H_

// An abstract base-class that runs a thread receiving data on UDP

#include <pthread.h>
#include <sys/socket.h>
#include <arpa/inet.h>

class UDPReceiver {
	const int updPort, packetSize;
	char* packetBuffer;
	bool stopReceiverThread; // Flag for exiting the thread
	pthread_t receiverThreadID;

	// Socket variables:
	int udpSocketID;
	struct sockaddr_in socketAddressMe, socketAddressOther; // Socket addresses
	socklen_t socketLength;

	// Functions:
	void tryToReceivePacket();
	static void* receiverThreadFunction(void *arg); // Thread function
	virtual bool isValidPacket(const char* packetBuffer) = 0; // This functions is implemented in a derived class, and must return true if the packet received is valid, and false otherwise
	virtual void actionWhenReceived(const char* packetBuffer) = 0; // This function is implemented in a derived class, and decides what to do when a packet has been received.

protected:
	// Const and dest protected so base class cannot be instantiated or deleted
	UDPReceiver(const int UDPport, const int packetSize); // Port that packets should be received on, and the size (constant) of the packets that should be received
	~UDPReceiver();
	void startReceiverThread();
};

#endif /* UDPRECEIVER_H_ */
