#include"Headers/LidarUDPReceiver.h"

#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define UDP_PORT 2368
//Constructor
LidarUDPReceiver::LidarUDPReceiver(char* rawLidarData) {
	// First allocate the tempPacket buffer, and create the UDP socket:
	this->rawLidarData=rawLidarData;

	udpSocketID = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (udpSocketID==-1) {
		printf("%s%s\n","Unable to create socket: ", strerror(errno));
		exit(-1);
	}

	// Set up the socket adress
	socketLength = sizeof(socketAddressMe);
	memset((char *) &socketAddressMe, 0, sizeof(socketAddressMe)); // Zero out socketAddressMe
	socketAddressMe.sin_family = AF_INET;
	socketAddressMe.sin_port = htons(UDP_PORT);
	socketAddressMe.sin_addr.s_addr = htonl(INADDR_ANY);

	// Bind the socket to socketAddress
	if(bind(udpSocketID , (struct sockaddr*)&socketAddressMe, sizeof(socketAddressMe)) == -1) {
		printf("%s%s\n","Unable to bind socket to socketAddress: ", strerror(errno));
		exit(-1);
	}

	// Test to see if lidar data can be received:
	printf("%s%d%s\n","Trying to receive lidar data on port ", UDP_PORT, "...");
	tryToReceiveLidarPacket();
	printf("%s\n","...success!");
	startReceiverThread();
}

LidarUDPReceiver::~LidarUDPReceiver() {
	// Stop the thread and wait for it to exit
	stopReceiverThread=true;
	pthread_join(receiverThreadID,NULL);

	close(udpSocketID);
}

void LidarUDPReceiver::tryToReceiveLidarPacket() {
	// A blocking method that tries to receive a lidar UDP packet. Returns if successful, keeps listening on socket if not
	int receivedPacketLength=0;
	while(true) {
		receivedPacketLength = recvfrom(udpSocketID,tempPacketBuffer,1206,0,(struct sockaddr *) &socketAddressOther, &socketLength);
		if (receivedPacketLength==-1) {
			printf("%s%s\n","Unable to receive UDP packet: ", strerror(errno));
			exit(-1);
		}
		if (isLidarPacket(tempPacketBuffer)) {return;}
	}
}

void LidarUDPReceiver::startReceiverThread() {
	// Starts the receiver thread; an infinite loop that continuously updates rawLidarData
	stopReceiverThread=false;

	//Start the receiver thread
	if(pthread_create(&receiverThreadID,NULL,receiverThreadFunction,this)) {
		printf("%s%s\n","Unable to create thread: ", strerror(errno));
		exit(-1);
	}
}

bool LidarUDPReceiver::isLidarPacket(const char* packet) {
	if (packet[0]==-1 && packet[1]==-18 && packet[1205]== 34) {return true;}
	return false;
}


void* LidarUDPReceiver::receiverThreadFunction(void* arg) {
	LidarUDPReceiver* thisPointer = (LidarUDPReceiver*)arg;

	bool& stopReceiverThread = thisPointer->stopReceiverThread;
	char *rawLidarData = thisPointer->rawLidarData, *tempPacketBuffer = thisPointer->tempPacketBuffer ;

	// Start listening to data:
	int receivedPacketLength=0,currentRawBufferOffset=0;

	//timeval curTime,oldTime;

	while(!stopReceiverThread) {
		thisPointer->tryToReceiveLidarPacket();
		// Copy the packet data from tempPacketBuffer to rawLidarBuffer, minus the last 6 bytes that are factory bytes
		memcpy(rawLidarData+currentRawBufferOffset,tempPacketBuffer,1200);
		currentRawBufferOffset+=1200;
		if (currentRawBufferOffset==90000) {currentRawBufferOffset=0;};

		//gettimeofday(&curTime,NULL); // Check if packets are received fast enough (time between packets should be 1326 microseconds
		//printf("%d\n",curTime.tv_usec-oldTime.tv_usec);
		//oldTime=curTime;
	}
	printf("%s\n","Lidar UDP receiver thread exited");
	pthread_exit(NULL);
}













