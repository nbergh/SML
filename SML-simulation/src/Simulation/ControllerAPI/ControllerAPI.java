package Simulation.ControllerAPI;
import java.util.LinkedList;

import Simulation.ControllerTalker;
import Simulation.Map;
import Simulation.VehicleStates;
import Simulation.VehicleStates.VehicleState;
import Simulation.Messages.Message;

public abstract class ControllerAPI {
	// This class implements the API for the controller to use
	
	private final ControllerTalker controllerTalker; // This is the controller manager class and is provided by the simulation
	private final PathHandler myPathHandler; // An object that handles path planning and updating
	private final SensorHandler mySensorHandler; // And object that handles the sensor readings
	private final MessageHandler myMessageHandler; // Handler for the messages
	protected final VehicleStates.VehicleState myState; // My vehicle state
	
	protected ControllerAPI(ControllerTalker controllerTalker, VehicleStates.VehicleState myState) {
		this.controllerTalker=controllerTalker;
		this.myPathHandler = new PathHandler(myState, controllerTalker.getTheMap());
		this.mySensorHandler = new SensorHandler(myState, controllerTalker.getTheStates());
		this.myMessageHandler = new MessageHandler();
		this.myState = myState;		
	}
	
	public abstract double[] updateControlSignals(); 
	// This is the main method that gets called regularly. Returns an array of size 2 where the first value
	// is the acceleration control signal and the second is the yawSpeed control signal. 
	
	public abstract void resetMergingController();
	
	public abstract VehicleState getMergeControllerPartnerVehicle();
	
	public abstract boolean isClearToMerge();
	
	protected double getAngleBetweenNodes(Map.MapNode fromNode, Map.MapNode toNode) {
		return controllerTalker.getTheMap().getAngleBetweenNodes(fromNode, toNode);
	}
	
	protected double getDistanceBetweenNodes(Map.MapNode fromNode, Map.MapNode toNode) {
		return controllerTalker.getTheMap().getDistanceBetweenNodes(fromNode, toNode);
	}
	
	protected Map.MapNode getMyPathEndNode() {
		return myPathHandler.getMyPathEndNode();
	}
	
	protected void setMyNextTarget(int nodeIndex) {
		myPathHandler.setMyNextTarget(nodeIndex);
	}
	
	protected void setMyNextTargetAndReplanPath(int nodeIndex) {
		myPathHandler.setMyNextTarget(nodeIndex);
		myPathHandler.replanPathFromMyPosition();
	}
	
	protected void replanPathFromFixedPosition(Map.MapNode fromNode) {
		myPathHandler.replanPathFromFixedPosition(fromNode);
	}
	
	protected LinkedList<Map.MapNode> getCurrentPathNodes() {
		return myPathHandler.getCurrentPathNodes();
	}	
	
	protected LinkedList<VehicleStates.VehicleState> getV2VWifiInput() {
		return mySensorHandler.getWifiInput();
	}
	
	protected int[] getRadarInput() {
		return mySensorHandler.getRadarInput();
	}
	
	protected LinkedList<Message> getMyMessages() {
		return myMessageHandler.getMyMessages();
	}
	
	protected Map.MapNode findNodeClosestTo(double x, double y) {
		return controllerTalker.getTheMap().findNodeClosestTo(x, y);
	}
	
	protected void sendMessageTo(Message message, int vehicleID) {
		controllerTalker.getController(vehicleID).addMessageToMyList(message);
	}
	
	protected void clearMyMessages() {
		myMessageHandler.clearMyMessages();
	}

	// These are used by the simulation:
	public void addMessageToMyList(Message message) {
		myMessageHandler.addMessageToMyList(message);
	}
	
	public LinkedList<Map.MapNode> getThePath() {
		return myPathHandler.getThePath();
	}
	
	public void resetMyPath() {
		myPathHandler.resetMyPath();
	}	
}
