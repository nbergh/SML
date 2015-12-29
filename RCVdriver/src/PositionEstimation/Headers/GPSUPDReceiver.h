#ifndef GPSUPDRECEIVER_H_
#define GPSUPDRECEIVER_H_

#include "../../Headers/UDPReceiver.h"

class GPSUDPReceiver : public UDPReceiver {
	void startReceiverThread();

public:
	GPSUDPReceiver(const int port);
	~GPSUDPReceiver();
};

#endif /* GPSUPDRECEIVER_H_ */
