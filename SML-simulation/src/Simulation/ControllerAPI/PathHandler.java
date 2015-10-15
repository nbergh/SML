package Simulation.ControllerAPI;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.PriorityQueue;
import java.util.Random;

import Simulation.Map;
import Simulation.VehicleStates;
import Simulation.HelperClasses.Rotation;

class PathHandler {
	// This class handles everything that has to do with paths in the map that the vehicle should follow
	private final Map map;
	private final VehicleStates.VehicleState myState;
	private LinkedList<Map.MapNode> myPath;
	private final Random random;
	private int myNextTarget; // This is the target node for the path. -1 means that the target is random
	
	PathHandler(VehicleStates.VehicleState myState, Map map) {
		this.map=map;
		this.myState=myState;
		this.myPath = new LinkedList<Map.MapNode>();
		this.random = new Random();
		this.myNextTarget = myState.initialTargetNode;
		replanPathFromFixedPosition(map.getMapGraph()[myState.initialStartNode]);
	}
	
	void resetMyPath() {		
		replanPathFromFixedPosition(map.getMapGraph()[myState.initialStartNode]);
	}
	
	void setMyNextTarget(int nodeIndex) {
		myNextTarget=nodeIndex;
	}
	
	Map.MapNode getMyPathEndNode() {
		return myPath.getLast();
	}
	
	void replanPathFromFixedPosition(Map.MapNode fromNode) {
		generateMyPath(fromNode);
	}
	
	void replanPathFromMyPosition() {
		generateMyPath(myPath.peek());		
	}

	LinkedList<Map.MapNode> getCurrentPathNodes() {
		updatePath();		
		
		LinkedList<Map.MapNode> currentNodes= new LinkedList<Map.MapNode>();
		currentNodes.add(myPath.get(0));
		currentNodes.add(myPath.get(1));
		
		return currentNodes;
	}

	LinkedList<Map.MapNode> getThePath() {
		return myPath;
	}
	 
	private void generateMyPath(Map.MapNode fromNode) {	
		if (myNextTarget==-1 || fromNode.nodeID==myNextTarget) {
			// Generate a new random node target
			myNextTarget = fromNode.nodeID;
			while (myNextTarget==fromNode.nodeID) {myNextTarget = random.nextInt(map.getMapGraph().length);} // Start and goal must be different			
		}
		myPath = generateAPath(fromNode, map.getMapGraph()[myNextTarget]);	
		myNextTarget=-1; // Reset the nextTarget node to random
	}
	
	private LinkedList<Map.MapNode> generateAPath(Map.MapNode fromNode, Map.MapNode toNode) {				
		if (fromNode.nodeID == toNode.nodeID) {
			System.out.println("Crash: fromNode and toNode are the same in pathplanner, ID: " + fromNode.nodeID);
			System.exit(-1);
		}			
		
		// This function generates a LinkedList of node coordinates that a vehicle can follow
		int targetX = toNode.posX, targetY = toNode.posY;
		
		//Reset the openList and closedList booleans in the mapGraph
		for (int i=0;i<map.getMapGraph().length;i++) {
			map.getMapGraph()[i].isOnAStarClosedList=false;
			map.getMapGraph()[i].isOnAStarOpenList=false;
			map.getMapGraph()[i].previousNodeInPath=null;
		}
		
		// A-star algorithm members:
		PriorityQueue<Map.MapNode> minHeap = new PriorityQueue<Map.MapNode>(10, new Comparator<Map.MapNode>(){ // A min-heap
			@Override 
			public int compare(Map.MapNode node1, Map.MapNode node2) {
				if (node1.distanceFromStartNode+node1.heuristics > node2.distanceFromStartNode + node2.heuristics) {return 1;}
				else {return -1;}						
			}
		});
		
		Map.MapNode baseNode = fromNode;
		
		while (true) {
			baseNode.isOnAStarClosedList=true;
			baseNode.isOnAStarOpenList=false;
			
			if (baseNode.nodeID==toNode.nodeID) {
				//We are done	
				return buildPath(toNode);
			}
			
			for (Map.MapNode neighbor : baseNode.neighbors)	{
				if (neighbor.isOnAStarClosedList) {continue;}
	
				if(!neighbor.isOnAStarOpenList) {
					neighbor.isOnAStarOpenList=true;
					neighbor.previousNodeInPath = baseNode;
					neighbor.distanceFromStartNode = baseNode.distanceFromStartNode + map.getDistanceBetweenNodes(baseNode, neighbor);
					neighbor.heuristics = Math.abs(targetX-neighbor.posX) + Math.abs(targetY-neighbor.posY);
					minHeap.add(neighbor);
				}
				else if (baseNode.distanceFromStartNode + map.getDistanceBetweenNodes(baseNode, neighbor) < neighbor.distanceFromStartNode){
					// Neighbor is on openList and path is also shorter if it goes through baseNode
					neighbor.distanceFromStartNode = baseNode.distanceFromStartNode + map.getDistanceBetweenNodes(baseNode, neighbor);
					neighbor.previousNodeInPath = baseNode;
				}				
			}
			baseNode = minHeap.poll();
			if (baseNode==null) {
				System.out.println("Crash: cant find path between nodes: " + fromNode.nodeID + " " + toNode.nodeID);
				System.exit(-1);
				break;
			}
		}	
		return null;
	}
	
	private LinkedList<Map.MapNode> buildPath(Map.MapNode targetNode) {
		LinkedList<Map.MapNode> path = new LinkedList<Map.MapNode>();
		Map.MapNode currentNode=targetNode;
		
		while(currentNode.previousNodeInPath != null) {
			path.addFirst(currentNode);
			currentNode = currentNode.previousNodeInPath;
		}
		path.addFirst(currentNode);
		
		return path;
	}
	
	private void updatePath() {		
		if (myPath.size()<2) {
			// This should never happen
			System.out.println("Crash: one-length path for vehicle " + myState.vehicleID);
			System.exit(-1);
		}		
		// In order for the vehicle to be able to align itself with the road it needs two reference points, one that is on the road behind the 
		// vehicle and one that is on the road in front of the vehicle. These points are stored in myPath at position 0 (behind the vehicle) and
		// position 1 (in front of the vehicle). When a vehicle has passed the point at position 1, then that point becomes the new position 0
		// This is accomplished here by rotating the position of the vehicle and the position of node 1 with the angle of the road so that thei
		// longitudinal coordinates can be calculated and compared. If needed node 1 then becomes node 0 by popping the path list
		
		double roadAngle=0, myLongPosRotated=0, nextNodeLongPosRotated=0;		
		while (true) {
			// Rotate coordinates clockwise, use the X/Long coordinate
			
			roadAngle = map.getAngleBetweenNodes(myPath.get(0), myPath.get(1));
			myLongPosRotated = Rotation.getX(myState.posX, myState.posY, -roadAngle);
			nextNodeLongPosRotated = Rotation.getX(myPath.get(1).posX, myPath.get(1).posY, -roadAngle);
			
			// Has the vehicle passed the node at the top of the myPath list?
			if (myLongPosRotated < nextNodeLongPosRotated) {break;}			
			myPath.pop();
			
			if (myPath.size()==1) {
				// The vehicle has reached its target
				generateMyPath(myPath.peek());
				break;
			} 	
		}		
	}
}
