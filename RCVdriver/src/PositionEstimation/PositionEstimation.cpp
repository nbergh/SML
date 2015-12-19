#include "Headers/PositionEstimation.h"

/* currentHeading in vehicleState must always be the angle in radians from true NORTH to the current vehicle
 * longitudinal centerline. CurrentHeading must always be between -Pi and Pi
 */

PositionEstimation::PositionEstimation() :
	vehiclePosition() {

	// Very temp:
	vehiclePosition.currentPosition.latc=59.35096;
	vehiclePosition.currentPosition.longc=18.06808;
}

PositionEstimation::~PositionEstimation() {
}

void PositionEstimation::updatePosition() {
}
