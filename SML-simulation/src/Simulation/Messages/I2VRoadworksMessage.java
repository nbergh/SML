package Simulation.Messages;


public final class I2VRoadworksMessage extends Message {
	 // The coordinates for the fromZone and toZone for the merging
	public int fromZoneStartX, fromZoneEndX, fromZoneStartY, fromZoneEndY;
	public int toZoneStartX, toZoneEndX, toZoneStartY, toZoneEndY;
	
	public I2VRoadworksMessage(int sender) {
		super(sender);			
	}		
}	