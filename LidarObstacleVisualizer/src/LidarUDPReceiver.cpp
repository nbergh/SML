#include"Headers/LidarUDPReceiver.h"

#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
//#include <sys/time.h>

void* LidarUDPReceiver::threadEntryFunction(void* thisPointer) {
	// A static entry function for receiveUDP
	((LidarUDPReceiver*)thisPointer)->receiveLidarData();
	return NULL;
}

bool LidarUDPReceiver::isLidarPacket(char* packet) {
	if (packet[0]==-1 && packet[1]==-18 && packet[1205]== 34) {return true;}
	return false;
}

void LidarUDPReceiver::tryToReceiveLidarPacket() {
	// A blocking method that tries to receive a lidar UDP packet. Returns if successful, keeps listening on socket if not
	int receivedPacketLength=0;
	while(true) {
		receivedPacketLength = recvfrom(udpSocket,tempPacketBuffer,1206,0,(struct sockaddr *) &socketAddressOther, &socketLength);
		if (receivedPacketLength==-1) {
			printf("Unable to receive UDP packet: %s\n", strerror(errno));
			exit(-1);
		}
		if (isLidarPacket(tempPacketBuffer)) {return;}
	}
}

void LidarUDPReceiver::receiveLidarData() {
	printf("UDP receiver thread started\n");

	// Start listening to data:
	int receivedPacketLength=0,currentRawBufferOffset=0;

	//timeval curTime,oldTime;

	while(!exitThread) {
		tryToReceiveLidarPacket();
		// Copy the packet data from tempPacketBuffer to rawLidarBuffer, minus the last 6 bytes that are factory bytes
		memcpy(rawLidarData+currentRawBufferOffset,tempPacketBuffer,1200);
		currentRawBufferOffset+=1200;
		if (currentRawBufferOffset==90000) {currentRawBufferOffset=0;};

		//gettimeofday(&curTime,NULL); // Check if packets are received fast enough (time between packets should be 1326 microseconds
		//printf("%d\n",curTime.tv_usec-oldTime.tv_usec);
		//oldTime=curTime;
	}
	printf("UDP receiver thread exited\n");
	pthread_exit(NULL);
}

pthread_t LidarUDPReceiver::startReceiverThread(char* rawLidarData) {
	// Starts the receiver thread; an infinite loop that continuously updates rawLidarData
	this->rawLidarData=rawLidarData;
	exitThread=false;
	pthread_t threadID;

	//Start the receiver thread
	if(pthread_create(&threadID,NULL,threadEntryFunction,this)) {
		printf("Unable to create thread: %s\n", strerror(errno));
		exit(-1);
	}
	return threadID;
}

void LidarUDPReceiver::setThreadExitFlag() {
	// Sets the thread exit flag to true, so that the thread can exit
	exitThread=true;
}

//Constructor
LidarUDPReceiver::LidarUDPReceiver(int udpPort) {
	// First create the UDP socket:
	tempPacketBuffer = new char[1206];
	udpSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (udpSocket==-1) {
		printf("Unable to create socket: %s\n", strerror(errno));
		exit(-1);
	}

	// Set up the socket adress
	socketLength = sizeof(socketAddressMe);
	memset((char *) &socketAddressMe, 0, sizeof(socketAddressMe)); // Zero out socketAddressMe
	socketAddressMe.sin_family = AF_INET;
	socketAddressMe.sin_port = htons(udpPort);
	socketAddressMe.sin_addr.s_addr = htonl(INADDR_ANY);

	// Bind the socket to socketAddress
	if(bind(udpSocket , (struct sockaddr*)&socketAddressMe, sizeof(socketAddressMe)) == -1) {
		printf("Unable to bind socket to socketAddress: %s\n", strerror(errno));
		exit(-1);
	}

	// Test to see if lidar data can be received:
	printf("Trying to receive lidar data on port %d%s", udpPort, "...\n");
	tryToReceiveLidarPacket();
	printf("...success!\n");
}

LidarUDPReceiver::~LidarUDPReceiver() {
	delete[] tempPacketBuffer;
	close(udpSocket);
}














