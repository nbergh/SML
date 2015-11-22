#ifndef LIDARUDPRECEIVER_H
#define LIDARUDPRECEIVER_H

#include <pthread.h>
#include <sys/socket.h>
#include <arpa/inet.h>

class LidarUDPReceiver {
	char *rawLidarData, tempPacketBuffer[1206];
	bool stopReceiverThread; // Flag for exiting the thread
	pthread_t receiverThreadID;

	// Socket variables:
	int udpSocketID;
	struct sockaddr_in socketAddressMe, socketAddressOther; // Socket addresses
	socklen_t socketLength;

	void startReceiverThread();
	void tryToReceiveLidarPacket();
	bool isLidarPacket(const char* packet);
	static void* receiverThreadFunction(void *arg); // Thread function

public:
	LidarUDPReceiver(char* rawLidarData);
	~LidarUDPReceiver();
};

#endif
