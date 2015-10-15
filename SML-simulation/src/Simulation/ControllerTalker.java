package Simulation;

import Simulation.ControllerAPI.ControllerAPI;
import Simulation.ControllerAPI.Controller.ControllerSupervisor;


public class ControllerTalker {
	// This is the controller manager class. It handles communication between the simulation and the controllers

	private final Start start;
	private final ControllerAPI[] controllerArray;
	// This list contains all the messages that are sent from the simulation to the vehicles. The messages will be sent to 
	// the vehicles if they are within range of the infrastructure transmitter
	
	ControllerTalker(Start start) {
		this.start=start;			
		controllerArray = new ControllerAPI[start.params.vehiceParameters.numberOfVehicles];
		
		// Initialize the vehicle controllers
		for (int i=0;i<start.params.vehiceParameters.numberOfVehicles;i++) {
			controllerArray[i] = new ControllerSupervisor(this, start.vehicleStates.vehicleStateArray[i]); // Change controller here if needed			
		}
	}
	
	void setNewControlSignals() {	
		// This method gets called by the controller updating thread regularly
		VehicleStates.VehicleState currentVehicle;
		Map.MapTransmitter currentTransmitter;
		double[] newControls = new double[2];		
		for (int i=0;i<start.params.vehiceParameters.numberOfVehicles;i++) {
			currentVehicle = start.vehicleStates.vehicleStateArray[i];
			// The simulation should send I2V messages if the vehicle is within range of a fixed position transmitter
			for (int j=0;j<start.params.mapTransmitterParameters.numberOfTransmitters;j++) {
				currentTransmitter = start.map.getTransmitters()[j];
				
				if (Math.hypot(currentTransmitter.posX-currentVehicle.posX, currentTransmitter.posY-currentVehicle.posY) < currentTransmitter.range) {
					controllerArray[i].addMessageToMyList(currentTransmitter.getTransmitterMessage());
				}
			}
						
			// Then update the controls
			newControls = controllerArray[i].updateControlSignals();
			start.vehicleStates.vehicleStateArray[i].controlSignalAcc=newControls[0];
			start.vehicleStates.vehicleStateArray[i].controlSignalYawSpeed=newControls[1];
		}		
	}

	void resetAllPaths() {
		// Resets all the paths
		for (int i=0;i<controllerArray.length;i++) {
			controllerArray[i].resetMyPath();
		}
	}
	
	void resetAllMergingControllers() {
		for (int i=0;i<controllerArray.length;i++) {
			controllerArray[i].resetMergingController();
		}
	}
	
	public Map getTheMap() {
		return start.map;
	}
	
	public VehicleStates.VehicleState[] getTheStates() {
		return start.vehicleStates.vehicleStateArray;
	}	
	
	public ControllerAPI getController(int index) {
		return controllerArray[index];
	}
	
}
