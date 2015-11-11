#include "Headers/PositionEstimation.h"

PositionEstimation::PositionEstimation() {
	vehicleState = new VehicleState;
}

PositionEstimation::~PositionEstimation() {
	delete vehicleState;
}

const VehicleState& PositionEstimation::getCurrentVehicleState() {
	return *vehicleState;
}
