#ifndef POSITIONESTIMATION_H_
#define POSITIONESTIMATION_H_

#include "Structs.h"

class PositionEstimation {


public:
	PositionEstimation();
	~PositionEstimation();
	GPSposition *getCurrentVehiclePosition();

};



#endif /* POSITIONESTIMATION_H_ */
