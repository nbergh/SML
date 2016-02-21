#include <stdio.h>
#include <algorithm>

#include "Headers/PositionEstimation.h"

int PositionEstimation::GPSUDPReceiver::tokenizeString(const char* packetBuffer, int startPos, char* token, int tokenMaxLength) {
	int tokenLength=0;
	while (packetBuffer[startPos+tokenLength]!=',' && packetBuffer[startPos+tokenLength]!=0) {tokenLength++;}

	for (int i=0;i<std::min(tokenLength,(tokenMaxLength-1));i++) {
		token[i]=packetBuffer[startPos+i];
	}
	token[std::min(tokenLength,(tokenMaxLength-1))]=0;

	return tokenLength;
}

bool PositionEstimation::GPSUDPReceiver::isValidPacket(const char* packetBuffer) {
	if (packetBuffer[0]=='$' && packetBuffer[1]=='G' && packetBuffer[2]=='P' && packetBuffer[3]=='H' && packetBuffer[4]=='D' && packetBuffer[5]=='T') {return true;}
	return false;
}

void PositionEstimation::GPSUDPReceiver::actionWhenReceived(const char* packetBuffer) {
	// This function is called every time a Trimble packet is received

	// Format of GPS string is: "$GPHDT,123.456,T*00..$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A.."
	// See http://www.trimble.com/OEM_ReceiverHelp/V4.44/en/NMEA-0183messages_RMC.html and
	// http://www.trimble.com/OEM_ReceiverHelp/V4.44/en/NMEA-0183messages_HDT.html for explanation

	char token[20];
	int startPos=0, tokenLength=0;

	for (int i=0;i<11;i++) {
		tokenLength=tokenizeString(packetBuffer,startPos,token,20);
		startPos+=(tokenLength+1);
		if (tokenLength==0) {continue;} // Empty token

		if (i==1) {
			sscanf(token,"%f",&rawTrimbleData.heading);
		}
		else if (i==3) {
			sscanf(token,"%f",&rawTrimbleData.utcWhenReceived);
		}
		else if (i==5) {
			sscanf(token,"%lf",&rawTrimbleData.latC);
		}
		else if (i==6 && token[0]=='S') {
			rawTrimbleData.latC*=-1;
		}
		else if (i==7) {
			sscanf(token,"%lf",&rawTrimbleData.longC);
		}
		else if (i==8 && token[0]=='W') {
			rawTrimbleData.longC*=-1;
		}
		else if (i==9) {
			sscanf(token,"%f",&rawTrimbleData.groundSpeed);
		}
		else if (i==10) {
			sscanf(token,"%f",&rawTrimbleData.trackAngle);
		}
	}
}

void PositionEstimation::updatePosition() {
	// This function fuses the trimble and IMU

	// currentHeading in vehiclePosition must always be the angle in radians from true NORTH to the current vehicle
	// longitudinal centerline. CurrentHeading must always be between -Pi and Pi

	vehiclePosition.currentHeading = gpsUDPReceiver.rawTrimbleData.trackAngle;
	vehiclePosition.groundSpeed = gpsUDPReceiver.rawTrimbleData.groundSpeed;
	vehiclePosition.currentPosition.latc = gpsUDPReceiver.rawTrimbleData.latC;
	vehiclePosition.currentPosition.longc = gpsUDPReceiver.rawTrimbleData.longC;



}









