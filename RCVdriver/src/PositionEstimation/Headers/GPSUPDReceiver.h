#ifndef GPSUPDRECEIVER_H_
#define GPSUPDRECEIVER_H_

#include "../../Headers/UDPReceiver.h"

class GPSUDPReceiver : public UDPReceiver {

	virtual bool isValidPacket(const char* packetBuffer);
	virtual void actionWhenReceived(const char* packetBuffer);

public:
	GPSUDPReceiver();
	~GPSUDPReceiver();
};

#endif /* GPSUPDRECEIVER_H_ */
