#include "Headers/UDPReceiver.h"

#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

UDPReceiver::UDPReceiver(const int port) : receiverThreadID(0), stopReceiverThread(false) {

	udpSocketID = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (udpSocketID==-1) {
		printf("%s%s\n","Unable to create socket: ", strerror(errno));
		exit(-1);
	}

	// Set up the socket adress
	socketLength = sizeof(socketAddressMe);
	memset((char *) &socketAddressMe, 0, sizeof(socketAddressMe)); // Zero out socketAddressMe
	socketAddressMe.sin_family = AF_INET;
	socketAddressMe.sin_port = htons(port);
	socketAddressMe.sin_addr.s_addr = htonl(INADDR_ANY);

	// Bind the socket to socketAddress
	if(bind(udpSocketID , (struct sockaddr*)&socketAddressMe, sizeof(socketAddressMe)) == -1) {
		printf("%s%s\n","Unable to bind socket to socketAddress: ", strerror(errno));
		exit(-1);
	}
}
