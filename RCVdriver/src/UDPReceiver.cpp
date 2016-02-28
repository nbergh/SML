#include "Headers/UDPReceiver.h"

#include <stdlib.h> // For exit
#include <errno.h>
#include <stdio.h>
#include <string.h> // for memset
#include <unistd.h> // for close

UDPReceiver::UDPReceiver(int udpPort, int packetSize) : updPort(udpPort), packetSize(packetSize) {
	packetBuffer = new char[packetSize];
	stopReceiverThread = false;
	receiverThreadID = 0;

	udpSocketID = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (udpSocketID==-1) {
		printf("%s%s\n","Unable to create socket: ", strerror(errno));
		exit(-1);
	}

	// Set up the socket adress
	socketLength = sizeof(socketAddressMe);
	memset((char *) &socketAddressMe, 0, sizeof(socketAddressMe)); // Zero out socketAddressMe
	socketAddressMe.sin_family = AF_INET;
	socketAddressMe.sin_port = htons(updPort);
	socketAddressMe.sin_addr.s_addr = htonl(INADDR_ANY);

	// Bind the socket to socketAddress
	if(bind(udpSocketID , (struct sockaddr*)&socketAddressMe, sizeof(socketAddressMe)) == -1) {
		printf("%s%s\n","Unable to bind socket to socketAddress: ", strerror(errno));
		exit(-1);
	}
}

UDPReceiver::~UDPReceiver() {
	// Stop the receiver thread and close the socket:
	stopReceiverThread=true;
	pthread_join(receiverThreadID,NULL);
	close(udpSocketID);

	delete[] packetBuffer;
}

void UDPReceiver::startReceiverThread() {
	// Test to see if UDP data can be received:
	printf("%s%d%s","Trying to receive UDP data on port ", updPort, "... ");
	fflush(stdout);
	tryToReceivePacket();
	printf("%s","success\n");

	//Start the receiver thread
	if(pthread_create(&receiverThreadID,NULL,receiverThreadFunction,this)) {
		printf("%s%s\n","Unable to create thread: ", strerror(errno));
		exit(-1);
	}
}

void UDPReceiver::tryToReceivePacket() {
	// A blocking method that tries to receive a UDP packet. Returns if successful, keeps listening on socket if not
	int receivedPacketLength=0;
	while(true) {
		receivedPacketLength = recvfrom(udpSocketID,packetBuffer,packetSize,0,(struct sockaddr *) &socketAddressOther, &socketLength);
		if (receivedPacketLength==-1) {
			printf("%s%s\n","Unable to receive UDP packet: ", strerror(errno));
			exit(-1);
		}
		if (isValidPacket(packetBuffer)) {return;}
	}
}

void* UDPReceiver::receiverThreadFunction(void* arg) {
	UDPReceiver* thisPointer = (UDPReceiver*)arg;

	bool& stopReceiverThread = thisPointer->stopReceiverThread;

	while(!stopReceiverThread) {
		thisPointer->tryToReceivePacket();
		thisPointer->actionWhenReceived(thisPointer->packetBuffer);
	}
	printf("%s\n","UDP receiver thread exited");
	pthread_exit(NULL);
}

