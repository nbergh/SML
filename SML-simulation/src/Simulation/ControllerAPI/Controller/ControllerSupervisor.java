package Simulation.ControllerAPI.Controller;
import java.util.LinkedList;

import Simulation.ControllerTalker;
import Simulation.Map;
import Simulation.VehicleStates;
import Simulation.ControllerAPI.ControllerAPI;
import Simulation.HelperClasses.Rotation;
import Simulation.Messages.I2VRoadworksMessage;
import Simulation.Messages.Message;
import Simulation.Messages.SafeToMergeReplyMessage;
import Simulation.Messages.SafeToMergeRequestMessage;

public class ControllerSupervisor extends ControllerAPI {	
	MergingController mergingController;
	
	public ControllerSupervisor(ControllerTalker controllerTalker, VehicleStates.VehicleState myState) {
		super(controllerTalker, myState);
		
		mergingController = new MergingController();		
	}

	@Override
	public double[] updateControlSignals() { // This is the main function that gets called regularly by the controller-thread	
		// Get path data. CurrentPathNodes.get(0) is the closest node behind the vehicle, and currentPathNodes.get(1) is 
		// the closest node ahead of the vehicle
		LinkedList<Map.MapNode> currentPathNodes = getCurrentPathNodes();
		int targetX = currentPathNodes.get(1).posX, targetY = currentPathNodes.get(1).posY;
		double roadAngle = getAngleBetweenNodes(currentPathNodes.get(0),currentPathNodes.get(1));
		
		// Calculate the path-errors:
		double latError=Rotation.getY(targetX, targetY, -roadAngle) - Rotation.getY(myState.posX, myState.posY, -roadAngle);
		double yawError=roadAngle - myState.yaw;

		if (yawError>Math.PI) {yawError-=2*Math.PI;} // Yaw error can never be greater than 180 degrees
		if (yawError<-Math.PI) {yawError+=2*Math.PI;}		
		
		// Read sensors:
		LinkedList<VehicleStates.VehicleState> vehiclesWithinWifiRange =  getV2VWifiInput();
		int[] radarInput = getRadarInput();
		
		// Calculate the acceleration and the yaw-speed signals:
		double[] signals = {0,0};
		signals[1]=getYawSpeedSignal(latError,yawError);
		
		if (radarInput[0]!=-1) {
			// Another vehicle has been detevted by the radar, so use the collisionAvoidanceController to decrease the acceleration signal 
			signals[0]= getAccCollisionAvoidanceSignal((double) radarInput[0],getNextVehicleSpeed(vehiclesWithinWifiRange, radarInput[1]));
		}
		else {
			// Just maintain the baseline speed
			signals[0]=getAccSpeedMaintainingSignal(getMyBaseSpeed());
		}
		
		// Now read my messages
		LinkedList<Message> myMessages = getMyMessages();
		
		if (myMessages.peek() instanceof I2VRoadworksMessage) {
			// I am in a merging-zone in that is adjacent to a road-works site, so i need to use a different controller in 
			// order to do a controlled merge
			double newAccSignal = mergingController.giveMergeControllerAccSignal((I2VRoadworksMessage) myMessages.pop(),myMessages,vehiclesWithinWifiRange,radarInput,roadAngle);
//			
			if (newAccSignal!=0) {signals[0]=newAccSignal;}
		}	
		else {
			// If no I2VRoadworsMessage has been received, then reset the merging controller
			mergingController.resetController();
		}		
		// Clear all messages at the end of the function
		clearMyMessages();
		
		return signals;
		
	}	
	
	private double getAccCollisionAvoidanceSignal(double nextVehicleDistance, double nextVehicleSpeed) {
		if (nextVehicleDistance < 0) {
			System.out.println("Crash: illegal distance for collision avoidance " + myState.vehicleID);
			System.exit(-1);
		}
		
		double desiredSpeed = getMyBaseSpeed();
		double distanceFactor = nextVehicleDistance/40;
		if (distanceFactor>1) {distanceFactor=1;}
		desiredSpeed *= distanceFactor; // Linear braking
//		desiredSpeed -= 10/(nextVehicleDistance+0.2); // Nonlinear braking
		
		desiredSpeed += (nextVehicleSpeed - myState.speed); 
		
		return desiredSpeed - myState.speed;
	}
	
	private double getAccSpeedMaintainingSignal(double speed) {
		return (speed - myState.speed) / 10;
	}
	
	private double getYawSpeedSignal(double latError, double yawError) {
//		latError+=20;
		return 0.01*(0.2*latError + 10 * yawError);
	}

	private double getMyBaseSpeed() {
		// A base speed for the vehicle
		return  0.6 + 0.4*((double) myState.vehicleID / (double) 100);
	}
	
	private double getNextVehicleSpeed(LinkedList<VehicleStates.VehicleState> vehiclesWithinWifiRange, int vehicleID) {
		// Find the speed of the next vehicle in the vehiclesWithinWifiRange linked list
		double nextVehicleSpeed=0;
		for (VehicleStates.VehicleState vehicle : vehiclesWithinWifiRange) {
			if (vehicle.vehicleID==vehicleID) {nextVehicleSpeed = vehicle.speed;}
		}
		return nextVehicleSpeed;
	}
	

	// These methods are used by the simulation for visualization of the merging and resetting of the merging controller
	@Override
	public VehicleStates.VehicleState getMergeControllerPartnerVehicle() {
		return mergingController.myPartnerVehicle;
	}
	@Override
	public boolean isClearToMerge() {
		return mergingController.isClearToMerge;
	}
	@Override
	public void resetMergingController() {
		mergingController.resetController();		
	}
	
	
	private final class MergingController {
		// This class handles all the logic of making the vehicle do a controlled merge into the other lane
		private final SafeToMergeReplyMessage mySafeToMergeReplyMessage;
		private final SafeToMergeRequestMessage mySafeToMergeRequestMessage;
		
		private VehicleStates.VehicleState myPartnerVehicle;
		private boolean isClearToMerge=false; 
		
		MergingController() {
			mySafeToMergeReplyMessage = new SafeToMergeReplyMessage(myState.vehicleID);
			mySafeToMergeRequestMessage = new SafeToMergeRequestMessage(myState.vehicleID,myState);
			// Predefined messages that will be used for v2v communication
		}
	
		double giveMergeControllerAccSignal(I2VRoadworksMessage I2VMessage, LinkedList<Message> myMessages, LinkedList<VehicleStates.VehicleState> vehiclesWithinWifiRange, int[] radarInput, double roadAngle){
			// This function updates the longitudinal controller, or returns 0 if no update is to be made
			
			// Start by checking if i am in the mergingZone fromZone (the lane that vehicles changes from), or the toZone
			// (the lane that vehicles change to)
			if (isInMergingFromZone(I2VMessage)) {
				// Vehicle is in merging fromZone
				
				if (myPartnerVehicle==null) {
					myPartnerVehicle=findPartnerVehicle(-20,roadAngle,vehiclesWithinWifiRange);
					
					if (myPartnerVehicle==null) {
						// If no vehicles are nearby, then it is safe to change lane
						changeLane(-20,roadAngle);
						isClearToMerge=true;
						return 0;
					}					
				}
				// Send merge request to partner vehicle
				mySafeToMergeRequestMessage.requestedDistance = (int) (myState.vehicleLength+20); 
				// This is the distance the vehicle wants the opening to be in the other lane
				sendMessageTo(mySafeToMergeRequestMessage, myPartnerVehicle.vehicleID);
				
				if (isClearToMerge) {return 0;}
				
				double minDistanceFromPartnerVehicleBeforeMerging=15;
				// This value is the number of pixels (distance) that the vehicle must be ahead of its partner vehicle before
				// it starts to change the lane
				
				for (Message message : myMessages) {				
					// Check if i have received a safe to merge confirmation
					if (message instanceof SafeToMergeReplyMessage && ((SafeToMergeReplyMessage) message).safeToMerge && getDistanceToPartnerVehicle(roadAngle) > minDistanceFromPartnerVehicleBeforeMerging) {
						changeLane(-20,roadAngle);
						isClearToMerge=true;
					}				
				}
				
				if (radarInput[0]!=-1) { // Dont change the longitudinal controller if the radar input reads something, always avoid collisions
					// Check if the collission avoidance controller wants to brake, if then brake
//					return 0;
					double newAcc = getAccCollisionAvoidanceSignal((double) radarInput[0],getNextVehicleSpeed(vehiclesWithinWifiRange, radarInput[1]) );
					if (newAcc<0) {return newAcc;} 				
				} 
				
				// Adjust my longitudinal controller so that i keep a position in front of myPartnerVehicle				
				double distanceToMyPartnerVehicle = getDistanceToPartnerVehicle(roadAngle);
				
				// The controller should keep a distance ahead of the partnerVehicle
				return -0.003*(distanceToMyPartnerVehicle-minDistanceFromPartnerVehicleBeforeMerging*1.2) + 0.1*(myPartnerVehicle.speed-myState.speed);
			}
			else if (isInMergingToZone(I2VMessage)) {
				// I am in merging toZone
				myPartnerVehicle=null;
				isClearToMerge=false;
				double requestedDistance=-1, distanceToNextVehicle=-1;
				int nextVehicleID=-1;
				
				if (radarInput[0]!=-1 ){
					distanceToNextVehicle = radarInput[0];
					nextVehicleID = radarInput[1];
				}
				else {
					// If there is no radar reading, then do a wifi-scan and approximate the distance to the next car
					// The reason for this is that radar reading is expensive at long range, so do a simpler reading
					// that is less accurate but has longer range
					
					if (!vehiclesWithinWifiRange.isEmpty()) {
						double distance=0, shortestDistance=-1;
						for (VehicleStates.VehicleState vehicle : vehiclesWithinWifiRange) {
							distance = Rotation.getX(vehicle.posX, vehicle.posY, -roadAngle) - Rotation.getX(myState.posX, myState.posY, -roadAngle);
							if (distance > 0) {								
								// Adjust distance
								distance -= (myState.vehicleLength/2 + vehicle.vehicleLength/2); 
								if (distance < 0) {distance=0;}
							}
							if (distance>=0 && Math.abs(Rotation.getY(vehicle.posX, vehicle.posY, -roadAngle ) - Rotation.getY(myState.posX, myState.posY, -roadAngle)) < 5) {
								if (distance < shortestDistance || shortestDistance==-1) {
									shortestDistance = distance;
									distanceToNextVehicle=shortestDistance;
									nextVehicleID = vehicle.vehicleID;
								}							
							}
						}
						// Not as accurate, but pretty good
					}					
				}
		
				// Find out which vehicles has sent safe to merge requests, so that i can adjust my distance to the next
				// vehicle, so that others vehicle fit in in front of me. I will only allow one vehicle to get in front of
				// me at a time: the vehicle that is furtest ahead of the other vehicles that has sent me safe to merge
				// requests
				VehicleStates.VehicleState theFirstPartnerVehicle=null;
				for (Message message : myMessages) {		
					if (message instanceof SafeToMergeRequestMessage) {
						// There may be multiple SafeToRequestMessage in the inbox, but only allow one car in at a time						
						
						if (theFirstPartnerVehicle==null || Rotation.getX(((SafeToMergeRequestMessage) message).myState.posX, ((SafeToMergeRequestMessage) message).myState.posY, -roadAngle) > Rotation.getX(theFirstPartnerVehicle.posX, theFirstPartnerVehicle.posY, -roadAngle)) {
							theFirstPartnerVehicle=((SafeToMergeRequestMessage) message).myState;
							requestedDistance = ((SafeToMergeRequestMessage) message).requestedDistance;
						}											
					}				
				}				
				if (theFirstPartnerVehicle==null) {return 0;} // Nobody has partnered with me

				if (distanceToNextVehicle==-1 || distanceToNextVehicle > requestedDistance) {
					mySafeToMergeReplyMessage.safeToMerge=true;
				}
				else {
					mySafeToMergeReplyMessage.safeToMerge=false;
				}
				
				sendMessageTo(mySafeToMergeReplyMessage, theFirstPartnerVehicle.vehicleID);
				
				// Now adjust my longitudinal controller so that i keep a distance of whatever the other vehicles needs to merge
				// into my lane
				if (distanceToNextVehicle==-1) {return 0;} // No change needed since we already have the right distance
				else {
					int controllerDistance = (int) (0.6*(distanceToNextVehicle-0.3*requestedDistance));
					if (controllerDistance<0) {controllerDistance=0;}
					
					return getAccCollisionAvoidanceSignal(controllerDistance,getNextVehicleSpeed(vehiclesWithinWifiRange, nextVehicleID));				
				}						
			}
			else {
				// I am not in the merging zone so dont change the controller
				return 0;
			}
		}
		
		void resetController() {
			// This function is called when the vehicle is not in a merging zone
			myPartnerVehicle=null;
			isClearToMerge=false;
//			hasChangedLane=false;
			clearMyMessages();
		}
		
		private void changeLane(int lateralOffset, double roadAngle) {
			// This functions finds the closest node to the lateralOffset of my position and replans the path from there 
			
			double posXRotated = Rotation.getX(myState.posX, myState.posY, -roadAngle);
			double posYRotated = Rotation.getY(myState.posX, myState.posY, -roadAngle);
			
			posYRotated+=lateralOffset;
			
			double newX = Rotation.getX(posXRotated, posYRotated, roadAngle);
			double newY = Rotation.getY(posXRotated, posYRotated, roadAngle);
			
			// Change the path
			replanPathFromFixedPosition(findNodeClosestTo(newX,newY));
		}
		
		private VehicleStates.VehicleState findPartnerVehicle(double lateralOffset, double roadAngle, LinkedList<VehicleStates.VehicleState> vehiclesWithinWifiRange) {
			if (vehiclesWithinWifiRange.isEmpty()) {return null;}
			
			VehicleStates.VehicleState closestVehicle=null;
			
			double distance=0, shortestDistance=-1;
			for (VehicleStates.VehicleState vehicle : vehiclesWithinWifiRange) {
				distance = Rotation.getX(myState.posX, myState.posY, -roadAngle) - Rotation.getX(vehicle.posX, vehicle.posY, -roadAngle);
				distance += (myState.vehicleLength/2 + vehicle.vehicleLength/2);
				// The distance here is calculated from the front of my vehicle to the back of the next vehicle
				
				if (distance>0 && Math.abs(Rotation.getY(myState.posX, myState.posY, -roadAngle) - Rotation.getY(vehicle.posX, vehicle.posY, -roadAngle ) + lateralOffset) < 5) {
					if (distance < shortestDistance || shortestDistance==-1) {
						shortestDistance = distance;
						closestVehicle = vehicle;
					}							
				}
			}
			
			return closestVehicle;
		}
		
		private double getDistanceToPartnerVehicle(double roadAngle) {
			if (myPartnerVehicle==null) {
				System.out.println("Crash: myPartnerVehicle, distance required by not initialized " + myState.vehicleID);
				System.exit(-1);
			}
			
			double distance = Rotation.getX(myState.posX, myState.posY, -roadAngle) - Rotation.getX(myPartnerVehicle.posX, myPartnerVehicle.posY, -roadAngle);
			distance -= (myState.vehicleLength/2 + myPartnerVehicle.vehicleLength/2);
//			if (distance<0) {distance=0;} // Allow negative distances
			
			return distance;
		}
	
		private boolean isInMergingFromZone(I2VRoadworksMessage I2VMessage) {
			if (myState.posX > I2VMessage.fromZoneStartX && myState.posX < I2VMessage.fromZoneEndX && myState.posY > I2VMessage.fromZoneStartY && myState.posY < I2VMessage.fromZoneEndY) {
				return true;
			}
			return false;
		}
		
		private boolean isInMergingToZone(I2VRoadworksMessage I2VMessage) {
			if (myState.posX > I2VMessage.toZoneStartX && myState.posX < I2VMessage.toZoneEndX && myState.posY > I2VMessage.toZoneStartY && myState.posY < I2VMessage.toZoneEndY) {
				return true;
			}
			return false;
		}		
	}
}
