#ifndef POSITIONESTIMATION_H_
#define POSITIONESTIMATION_H_

#include "../../Headers/Structs.h"

class PositionEstimation {
	VehicleState* vehicleState; // The state of the vehicle

public:
	PositionEstimation();
	~PositionEstimation();
	const VehicleState& getCurrentVehicleState();
};



#endif /* POSITIONESTIMATION_H_ */
