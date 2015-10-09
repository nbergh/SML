#ifndef LIDARUDPRECEIVER_H
#define LIDARUDPRECEIVER_H

#include <pthread.h>
#include <sys/socket.h>
#include <arpa/inet.h>

class LidarUDPReceiver {
	int udpSocket;
	struct sockaddr_in socketAddressMe, socketAddressOther; // Socket adresses
	socklen_t socketLength;

	bool exitThread; // Flag for exiting the thread
	char *tempPacketBuffer, *rawLidarData;

	static void* threadEntryFunction(void* thisPointer); // A static function that is required for pthread_create
	void tryToReceiveLidarPacket();
	void receiveLidarData();
	bool isLidarPacket(char* packet);

	public:
		LidarUDPReceiver(int udpPort);
		~LidarUDPReceiver();
		void setThreadExitFlag();
		pthread_t startReceiverThread(char* rawLidarData);
};

#endif
