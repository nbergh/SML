package Simulation.Messages;

import Simulation.VehicleStates;

public class SafeToMergeRequestMessage extends Message {
	public final VehicleStates.VehicleState myState;
	public int requestedDistance=0;
	
	public SafeToMergeRequestMessage(int sender, VehicleStates.VehicleState myState) {
		super(sender);
		this.myState=myState;
	}
}
