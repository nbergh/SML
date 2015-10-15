package Simulation.ControllerAPI;
import java.util.LinkedList;

import Simulation.VehicleStates;
import Simulation.HelperClasses.Rotation;

final class SensorHandler {
	private final VehicleStates.VehicleState myState;
	private final VehicleStates.VehicleState[] vehicleStateArray;
	
	public SensorHandler(VehicleStates.VehicleState myState, VehicleStates.VehicleState[] vehicleStateArray) {
		this.myState = myState;
		this.vehicleStateArray = vehicleStateArray;
	}
	
	int[] getRadarInput() {
		// This function returns the distance to the closest vehicle (within radar range) and its vehicleID in form {a,b}
		VehicleStates.VehicleState nextVehicle;
		double myPosXRotated = Rotation.getX(myState.posX, myState.posY, -myState.yaw);
		double myPosYRotated = Rotation.getY(myState.posX, myState.posY, -myState.yaw);
		
		double nextVehiclePosXRotated=0, nextVehiclePosYRotated=0;
		int closestDistance=myState.radarRangeFront, distance=-1, closestID=-1;
		
		for (int k=0;k<vehicleStateArray.length;k++) {
			if (k==myState.vehicleID) {continue;} // Ignore my own vehicle			
			nextVehicle = vehicleStateArray[k];
			
			if ( Math.abs(nextVehicle.posX - myState.posX) <= (myState.vehicleLength/2 + myState.radarRangeFront + nextVehicle.vehicleLength/2) && Math.abs(nextVehicle.posY - myState.posY) <= (myState.vehicleLength/2 + myState.radarRangeFront + nextVehicle.vehicleLength/2) ) {
				// Only if this is true can the vehicle be in radar range
				nextVehiclePosXRotated = Rotation.getX(nextVehicle.posX, nextVehicle.posY, -myState.yaw);
				nextVehiclePosYRotated = Rotation.getY(nextVehicle.posX, nextVehicle.posY, -myState.yaw);
				
				if ( (nextVehiclePosXRotated-myPosXRotated)>0 && (nextVehiclePosXRotated-myPosXRotated)<= (myState.vehicleLength/2 + myState.radarRangeFront + nextVehicle.vehicleLength/2) && Math.abs(nextVehiclePosYRotated-myPosYRotated)<=(myState.radarRangeSide+nextVehicle.vehicleLength/2) ) {
					// Check if vehicle is in front of me, and that its Y coordinates are within radar range
					distance = getDistance(nextVehicle, closestDistance);
					if (distance<closestDistance) {
						closestDistance = distance;
						closestID = k;
					}					
				}				
			}
		}
	
		int[] retvals = new int[2];
		retvals[0] = -1;
		retvals[1] = -1;
		
		if (closestDistance<myState.radarRangeFront) {
			retvals[0]=closestDistance;
			retvals[1]=closestID;
			if (closestID==-1) {
				System.out.println("Crash: Radar error for vehicle: " + myState.vehicleID);
			}
		}		
		return retvals;
	}
	
	private int getDistance(VehicleStates.VehicleState vehicle, int closestDistance) {
		// Check longitudinally and laterally over the radar sensor area
		double currentX=0, currentY=0;
		
		for (int longCoord=0;longCoord<=closestDistance;longCoord++) { 
			for (int latCoord=-myState.radarRangeSide;latCoord<=myState.radarRangeSide;latCoord++) {
				currentX = myState.posX + Rotation.getX(myState.vehicleLength/2 + longCoord, latCoord, myState.yaw);
				currentY = myState.posY + Rotation.getY(myState.vehicleLength/2 + longCoord, latCoord, myState.yaw);
				
				if (isPointCoveredByVehicle(currentX,currentY,vehicle)) {
					return longCoord;
				}				
			}
		}				
		return closestDistance;
	}
	
	private boolean isPointCoveredByVehicle(double posX, double posY, VehicleStates.VehicleState vehicle) {
		// This function checks if the point x, y is occupied by any part of a vehicle
		 posX-=vehicle.posX; // decrement to the origin
		 posY-=vehicle.posY;
		
		double posXRotated = Rotation.getX(posX, posY, -vehicle.yaw); // then rotate
		double posYRotated = Rotation.getY(posX, posY, -vehicle.yaw);
		
		if (Math.abs(posXRotated)<=vehicle.vehicleLength/2 && Math.abs(posYRotated)<= vehicle.vehicleWidth/2) {
			return true;
		}
		
		return false;
	}
	
	LinkedList<VehicleStates.VehicleState> getWifiInput() {
		// This function returns a LinkedList of vehicle states of all vehicles that are within wifi-range.
		// Entries in the list must not be manipulated since they references the actual states of the vehicles
		LinkedList<VehicleStates.VehicleState> returnedStates = new LinkedList<VehicleStates.VehicleState>();
		double myPosX = myState.posX, myPosY = myState.posY;
		double nextVehiclePosX=0,nextVehiclePosY=0;
		
		for (int i=0;i<vehicleStateArray.length;i++) {
			if (i==myState.vehicleID) {continue;} // Ignore my own vehicle
			
			nextVehiclePosX = vehicleStateArray[i].posX;
			nextVehiclePosY = vehicleStateArray[i].posY;		

			if (Math.hypot(myPosX-nextVehiclePosX, myPosY-nextVehiclePosY) < myState.wifiRange) {
				// Vehicle is within wifi-range
				returnedStates.add(vehicleStateArray[i]);					
			}				
		}		
		return returnedStates;
	}	
}
