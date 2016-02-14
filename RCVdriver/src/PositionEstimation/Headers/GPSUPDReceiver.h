#ifndef GPSUPDRECEIVER_H_
#define GPSUPDRECEIVER_H_

#include "../../Headers/UDPReceiver.h"

class GPSUDPReceiver : public UDPReceiver {

	virtual bool isValidPacket(const char* packetBuffer); // final
	virtual void actionWhenReceived(const char* packetBuffer); // final

public:
	GPSUDPReceiver();
	~GPSUDPReceiver();
};

#endif /* GPSUPDRECEIVER_H_ */
