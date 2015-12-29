#ifndef POSITIONESTIMATION_H_
#define POSITIONESTIMATION_H_

#include "../../Headers/Structs.h"
#include "GPSUPDReceiver.h"

class PositionEstimation {
	VehiclePosition vehiclePosition; // The state of the vehicle
//	GPSUDPReceiver gpsUDPReceiver;

public:
	PositionEstimation();
	~PositionEstimation();
	const VehiclePosition& getCurrentVehiclePosition() {return vehiclePosition;}
	void updatePosition();
};



#endif /* POSITIONESTIMATION_H_ */
