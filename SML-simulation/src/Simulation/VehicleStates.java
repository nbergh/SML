package Simulation;
import java.util.Random;

public final class VehicleStates {
	private final Start start;
	private final Random random;
	final VehicleState[] vehicleStateArray; 
	
	VehicleStates(Start start) {
		this.start = start;
		this.random = new Random(); // For intialization of randomized vehicle states
		this.vehicleStateArray = new VehicleState[start.params.vehiceParameters.numberOfVehicles];
		
		for (int i=0;i<start.params.vehiceParameters.numberOfVehicles;i++) {
			vehicleStateArray[i] = new VehicleState(i);
		}	
		setVehicleStartingStates(); // Write the start positions and start speeds to the vehicleStateArray		
	}
	
	void setVehicleStartingStates() {
		int startNode=0;
		
		// This function sets all the positions and speeds of the vehicles to the starting position		
		for (int i=0;i<start.params.vehiceParameters.numberOfVehicles;i++) {
			vehicleStateArray[i].wifiRange = start.params.vehiceParameters.wifiRange;
			vehicleStateArray[i].radarRangeFront = start.params.vehiceParameters.radarRangeFront;
			vehicleStateArray[i].radarRangeSide = start.params.vehiceParameters.radarRangeSide;
			vehicleStateArray[i].controlSignalYawSpeed = 0;
			vehicleStateArray[i].controlSignalAcc = 0;
			
			// If the user has set starting conditions manually, then set the vehicleStateArray to those values
			if (i<start.params.vehiceParameters.vehicleStartStates.length/5) {
				vehicleStateArray[i].vehicleWidth = (int) start.params.vehiceParameters.vehicleStartStates[5*i];
				vehicleStateArray[i].vehicleLength = (int) start.params.vehiceParameters.vehicleStartStates[5*i+1];
				
				startNode = (int) start.params.vehiceParameters.vehicleStartStates[5*i+2];
				
				vehicleStateArray[i].initialStartNode = startNode;
				vehicleStateArray[i].initialTargetNode = (int) start.params.vehiceParameters.vehicleStartStates[5*i+3];
				vehicleStateArray[i].posX = start.map.getMapGraph()[startNode].posX;
				vehicleStateArray[i].posY = start.map.getMapGraph()[startNode].posY;
				vehicleStateArray[i].yaw = start.map.getAngleBetweenNodes(start.map.getMapGraph()[startNode], start.map.getMapGraph()[startNode].neighbors.get(0));
				vehicleStateArray[i].speed = start.params.vehiceParameters.vehicleStartStates[5*i+4];
			}
			else {
				// If there are more vehicles than the user set manual parameters for, then set the rest of the parameters to random values
//				random.setSeed(System.currentTimeMillis()); // New random seeds
				vehicleStateArray[i].vehicleWidth = (int) (random.nextDouble()*(start.params.vehiceParameters.maxVehicleWidth-start.params.vehiceParameters.minVehicleWidth) + start.params.vehiceParameters.minVehicleWidth);
				vehicleStateArray[i].vehicleLength = (int) (random.nextDouble()*(start.params.vehiceParameters.maxVehicleLength-start.params.vehiceParameters.minVehicleLength) + start.params.vehiceParameters.minVehicleLength);
				
				startNode = random.nextInt(start.params.mapParameters.numberOfMapNodes);

				vehicleStateArray[i].initialTargetNode=-1; // This means that the target node will be random in the PathHandler
				vehicleStateArray[i].initialStartNode=startNode;			
				vehicleStateArray[i].posX = start.map.getMapGraph()[startNode].posX;
				vehicleStateArray[i].posY = start.map.getMapGraph()[startNode].posY;
				
				vehicleStateArray[i].yaw = start.map.getAngleBetweenNodes(start.map.getMapGraph()[startNode], start.map.getMapGraph()[startNode].neighbors.get(0));
				vehicleStateArray[i].speed = random.nextDouble()*(start.params.vehiceParameters.maxSpeed-start.params.vehiceParameters.maxSpeed/2) + start.params.vehiceParameters.maxSpeed/2; // Random speed is between max and max/2
			}			
		}
	}	
	
	void updateStates() {
		// This function represents the model (of a vehicle). Acc and yawSpeed are the control signals, and the function updates the positions based on them
		for (int i=0;i<start.params.vehiceParameters.numberOfVehicles;i++) {
			
			// First cap the control signals if they are to high/low
			if (vehicleStateArray[i].controlSignalAcc > start.params.vehiceParameters.maxAcc) {vehicleStateArray[i].controlSignalAcc = start.params.vehiceParameters.maxAcc;}
			else if (vehicleStateArray[i].controlSignalAcc < start.params.vehiceParameters.minAcc) {vehicleStateArray[i].controlSignalAcc = start.params.vehiceParameters.minAcc;}
			
			if (vehicleStateArray[i].controlSignalYawSpeed > start.params.vehiceParameters.maxYawSpeed) {vehicleStateArray[i].controlSignalYawSpeed = start.params.vehiceParameters.maxYawSpeed;}
			else if (vehicleStateArray[i].controlSignalYawSpeed < start.params.vehiceParameters.minYawSpeed) {vehicleStateArray[i].controlSignalYawSpeed = start.params.vehiceParameters.minYawSpeed;}
			
			// Next update speed and yawSpeed based on control-signals
			vehicleStateArray[i].speed += vehicleStateArray[i].controlSignalAcc;
			vehicleStateArray[i].yaw += vehicleStateArray[i].controlSignalYawSpeed;
			
			// Cap the speed if it is to high/low
			if (vehicleStateArray[i].speed > start.params.vehiceParameters.maxSpeed) {vehicleStateArray[i].speed = start.params.vehiceParameters.maxSpeed;}
			else if (vehicleStateArray[i].speed < start.params.vehiceParameters.minSpeed) {vehicleStateArray[i].speed = start.params.vehiceParameters.minSpeed;}						
			
			// Adjust yaw so that it is between 0 and 2*pi
			if (vehicleStateArray[i].yaw < 0) {vehicleStateArray[i].yaw+=2*Math.PI;}
			else if (vehicleStateArray[i].yaw > 2*Math.PI) {vehicleStateArray[i].yaw-=2*Math.PI;}			
			
			// Update the positions based on the speeds
			vehicleStateArray[i].posX += vehicleStateArray[i].speed * Math.cos(vehicleStateArray[i].yaw);
			vehicleStateArray[i].posY += vehicleStateArray[i].speed * Math.sin(vehicleStateArray[i].yaw);
			
			// Cap the position if it is outside the map
			if (vehicleStateArray[i].posX < 0) {vehicleStateArray[i].posX = 0;}
			else if (vehicleStateArray[i].posX >= start.params.mapParameters.mapHeight) {vehicleStateArray[i].posX = start.params.mapParameters.mapHeight-1;}
			
			if (vehicleStateArray[i].posY < 0) {vehicleStateArray[i].posY = 0;}
			else if (vehicleStateArray[i].posY >= start.params.mapParameters.mapWidth) {vehicleStateArray[i].posY = start.params.mapParameters.mapWidth-1;}			
		}
	}

	public final class VehicleState {
		public final int vehicleID, colorCode;
		public int vehicleWidth=0, vehicleLength=0; // closestNodeWhenInitialized is used for path-finding
		public int initialStartNode=-1, initialTargetNode=-1; 
		// initialStartNode is the node the vehicle should head to at startup. It is the start of the path the vehicle should follow
		// initalTargetNode is the end of the path the vehicle should follow
		public double posX=0, posY=0, yaw=0, speed=0;
		double controlSignalAcc=0, controlSignalYawSpeed=0; // Acc and yawAcc are the control signals
		public int wifiRange=0, radarRangeFront=0,radarRangeSide=0; // Sensor ranges
		public boolean markThisVehicle=false;
		
		VehicleState(int vehicleID) {
			this.vehicleID=vehicleID; // Identification number of the vehicle
			this.colorCode = vehicleID%11; // 11 available colors
		}				
	}
	
}
