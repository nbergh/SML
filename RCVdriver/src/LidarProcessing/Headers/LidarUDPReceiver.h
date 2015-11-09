#ifndef LIDARUDPRECEIVER_H
#define LIDARUDPRECEIVER_H

#include <pthread.h>
#include <sys/socket.h>
#include <arpa/inet.h>

class LidarUDPReceiver {
	char *tempPacketBuffer, *rawLidarData;
	bool stopReceiverThread; // Flag for exiting the thread
	pthread_t receiverThreadID;

	// Socket variables:
	int udpSocketID;
	struct sockaddr_in socketAddressMe, socketAddressOther; // Socket addresses
	socklen_t socketLength;

	static void* threadEntryFunction(void* thisPointer); // A static function that is required for pthread_create
	void tryToReceiveLidarPacket();
	bool isLidarPacket(char* packet);

	void receiverThreadFunction(); // Thread function

public:
	LidarUDPReceiver(int udpPort);
	~LidarUDPReceiver();
	void startReceiverThread(char* rawLidarData);
};

#endif
