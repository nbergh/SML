#ifndef POSITIONESTIMATION_H_
#define POSITIONESTIMATION_H_

#include "Structs.h"
#include "UDPReceiver.h"

#define GPS_UDP_PORT 50000

class PositionEstimation {
	class GPSUDPReceiver : public UDPReceiver {
		// Class that receives UDP packet from the Trimble and from the IMU
		friend class PositionEstimation;
		struct RawTrimbleData {
			double latC,longC; // The lat and long coordinates in form of degrees * 100 (59.2345 represented as 5923.45)
			float utcWhenReceived; // The current UTC time in format HHMMSS.XX
			float groundSpeed, heading, trackAngle; // The current speed in knots, trackAngle and heading in degrees 0-360 (heading is more accurate than trackAngle, but requires two trimbles)
		};
		struct RawIMUdata {
			// Fields for the IMU
		};
		RawTrimbleData rawTrimbleData = {};
		RawIMUdata rawIMUdata = {};

		virtual bool isValidPacket(const char* packetBuffer);
		virtual void actionWhenReceived(const char* packetBuffer);
		int tokenizeString(const char* packetBuffer, int startPos, char* token, int tokenMaxLength);

	public:
		GPSUDPReceiver() : UDPReceiver(GPS_UDP_PORT,200) {startReceiverThread();}
		~GPSUDPReceiver() {}
	};

	GPSUDPReceiver gpsUDPReceiver;
	VehiclePosition vehiclePosition = {};

public:
	PositionEstimation() {}
	~PositionEstimation() {}
	void updatePosition();
	const VehiclePosition& getCurrentVehiclePosition() {return vehiclePosition;}
};



#endif /* POSITIONESTIMATION_H_ */
