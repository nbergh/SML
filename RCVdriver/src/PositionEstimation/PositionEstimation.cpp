#include "Headers/PositionEstimation.h"

/* currentHeading in vehicleState must always be the angle in radians from true NORTH to the current vehicle
 * longitudinal centerline. CurrentHeading must always be between -Pi and Pi
 */

PositionEstimation::PositionEstimation() {
	vehicleState = new VehicleState;
}

PositionEstimation::~PositionEstimation() {
	delete vehicleState;
}

const VehicleState& PositionEstimation::getCurrentVehicleState() {
	return *vehicleState;
}
