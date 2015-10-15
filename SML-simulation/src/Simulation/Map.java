package Simulation;

import java.util.LinkedList;
import java.awt.*;

import javax.swing.*;

import Simulation.Messages.*;

public final class Map {
	private final Start start; // Pointer to the start class
	private final MapGraphics mapGraphics;
	private final MapNode[] mapGraph; // An adjacency array of all the nodes in the map.
	private final MapTransmitter[] mapTransmitters; // An array containing map transmitter objects (like roadside transmitters)
	
	Map(Start start) {
		this.start = start;
		this.mapGraphics = new MapGraphics();			
		this.mapGraph = createTheMap();
		this.mapTransmitters = createTransmitters();
	}
	
	private MapNode[] createTheMap() {		
		MapNode[] map = new MapNode[start.params.mapParameters.numberOfMapNodes];
		
		MapNode currentNode;
		
		int newX=0,newY=0, rowLength=start.params.mapParameters.mapNodesRowLength;
		for (int i=0;i<start.params.mapParameters.numberOfMapNodes;i++) {
			newX = start.params.mapParameters.mapNodes[rowLength*i];
			newY = start.params.mapParameters.mapNodes[rowLength*i+1];
			
			currentNode = new MapNode(i,newX,newY);
			map[i] = currentNode;
		}
		
		// Set the neighbors and distances
		MapNode neighbourNode;
		double neighbourAngle=0,neighbourDistance=0;
		for (int i=0;i<start.params.mapParameters.numberOfMapNodes;i++) {			
			currentNode = map[i];
			
			for (int j=0;j<rowLength;j++) {
				if (j==0 || j==1) {continue;} // This fields are the X and Y coordinates
				
				if (start.params.mapParameters.mapNodes[rowLength*i+j] > -1) {
					// Neighbor found
					neighbourNode = map[start.params.mapParameters.mapNodes[rowLength*i+j]];
					
					currentNode.neighbors.add(neighbourNode);
					
					neighbourAngle = Math.atan2(neighbourNode.posY-currentNode.posY, neighbourNode.posX-currentNode.posX);
					if (neighbourAngle<0) {neighbourAngle+=2*Math.PI;}	
					
					currentNode.neighborAngles.add(neighbourAngle);
					
					neighbourDistance = Math.hypot(neighbourNode.posX-currentNode.posX, neighbourNode.posY-currentNode.posY);
					currentNode.neighborDistances.add(neighbourDistance);
				}
			}
		}
		return map;
	}
	
	public MapNode[] getMapGraph() {
		return mapGraph;
	}
	
	public MapNode findNodeClosestTo(double x, double y) {
		// Complexity O(n), so dont use this unless you really have to. 
		MapNode currentNode,closestNode=null;
		double closestDistance=0,distance=0;
		
		for (int i=0;i<start.params.mapParameters.numberOfMapNodes;i++) {
			currentNode = mapGraph[i];
			
			distance =  Math.hypot(x-currentNode.posX, y-currentNode.posY);
			if (distance < closestDistance || closestDistance==0) {
				closestDistance = distance;
				closestNode = currentNode;
			}
		}
		
		return closestNode;
	}
	
	public double getAngleBetweenNodes(MapNode fromNode, MapNode toNode) {	
		return getAngleOrDistanceBetweenNodes(fromNode, toNode, 0);
	}
	
	public double getDistanceBetweenNodes(MapNode fromNode, MapNode toNode) {
		return getAngleOrDistanceBetweenNodes(fromNode, toNode, 1);
	}
	
	private double getAngleOrDistanceBetweenNodes(MapNode fromNode, MapNode toNode, int choice) {
		int index=0;
		
		for (MapNode neighbor : fromNode.neighbors) {
			if (neighbor.nodeID == toNode.nodeID) {
				if (choice==0) {return fromNode.neighborAngles.get(index);}
				else if (choice==1) {return fromNode.neighborDistances.get(index);}
			}
			index++;
		}
		
		System.out.println("Crash: nodes are not neighbours " + fromNode.nodeID + " " + toNode.nodeID);
		System.exit(-1);
		return 0;
	}		
	
	public final class MapNode {
		// A node for the mapGraph
		public final int nodeID, posX, posY; // Position and ID of the mapnode
		public final LinkedList<MapNode> neighbors;	// Neighbours of the node
		private final LinkedList<Double> neighborAngles; // Angles (in relation to the X axis) to the neighbours (ordered the same way as the neighbours list) 
		private final LinkedList<Double> neighborDistances; // Distances to the neighbours
		boolean markThisNode=false; // Marks the node in the graphics simulator for debugging purposes
		
		// Variables used for path-finding:
		public boolean isOnAStarOpenList = false, isOnAStarClosedList=false;
		public double distanceFromStartNode=0,heuristics=0;
		public MapNode previousNodeInPath=null;
		
		MapNode(int nodeID, int posX, int posY) {
			this.posX=posX;
			this.posY=posY;
			this.nodeID=nodeID;
			
			neighbors = new LinkedList<MapNode>();
			neighborAngles = new LinkedList<Double>();
			neighborDistances = new LinkedList<Double>();
		}	
	}
	
	// Map transmitters:
	public MapTransmitter[] getTransmitters() {
		return mapTransmitters;
	}
	
	private MapTransmitter[] createTransmitters() {
		MapTransmitter[] transmitters = new MapTransmitter[start.params.mapTransmitterParameters.numberOfTransmitters];
		
		for (int i=0;i<start.params.mapTransmitterParameters.numberOfTransmitters;i++) {
			transmitters[i] = new MapTransmitter(
					start.params.mapTransmitterParameters.transmitterParameters[11*i],
					start.params.mapTransmitterParameters.transmitterParameters[11*i+1],
					start.params.mapTransmitterParameters.transmitterParameters[11*i+2],
					start.params.mapTransmitterParameters.transmitterParameters[11*i+3],
					start.params.mapTransmitterParameters.transmitterParameters[11*i+4],
					start.params.mapTransmitterParameters.transmitterParameters[11*i+5],
					start.params.mapTransmitterParameters.transmitterParameters[11*i+6],
					start.params.mapTransmitterParameters.transmitterParameters[11*i+7],
					start.params.mapTransmitterParameters.transmitterParameters[11*i+8],
					start.params.mapTransmitterParameters.transmitterParameters[11*i+9],
					start.params.mapTransmitterParameters.transmitterParameters[11*i+10]);
		}
		
		return transmitters;
	}
	
	public final class MapTransmitter {
		// This class represents a transmitter with a fixed position in the map
		private final I2VRoadworksMessage myMessage;
		public final int posX,posY,range,fromZoneStartX,fromZoneEndX,fromZoneStartY,fromZoneEndY,toZoneStartX,toZoneEndX,toZoneStartY,toZoneEndY;
		
		MapTransmitter(int posX, int posY, int range, int fromZoneStartX, int fromZoneEndX, int fromZoneStartY, int fromZoneEndY, int toZoneStartX, int toZoneEndX, int toZoneStartY, int toZoneEndY) {
			// Stupid, i know
			this.posX = posX;
			this.posY = posY;
			this.range=range;
			this.fromZoneStartX = fromZoneStartX;
			this.fromZoneEndX = fromZoneEndX;
			this.fromZoneStartY = fromZoneStartY;
			this.fromZoneEndY = fromZoneEndY;
			this.toZoneStartX = toZoneStartX;
			this.toZoneEndX = toZoneEndX;
			this.toZoneStartY = toZoneStartY;
			this.toZoneEndY = toZoneEndY;
			this.myMessage = createMyMessage();
		}
		
		private I2VRoadworksMessage createMyMessage() {
			I2VRoadworksMessage message = new I2VRoadworksMessage(-1); // Sender is -1, meaning the simulator
			
			message.fromZoneStartX = fromZoneStartX;
			message.fromZoneEndX = fromZoneEndX;
			message.fromZoneStartY = fromZoneStartY;
			message.fromZoneEndY = fromZoneEndY;
			message.toZoneStartX = toZoneStartX;
			message.toZoneEndX = toZoneEndX;
			message.toZoneStartY = toZoneStartY;
			message.toZoneEndY = toZoneEndY;		
			
			return message;
		}
		
		public I2VRoadworksMessage getTransmitterMessage() {
			return myMessage;
		}
	}
	
	// Graphics class:
	
	MapGraphics getMapGraphics() {
		return mapGraphics;
	}	
	void repaintGraphics() {
		mapGraphics.repaint();
	}	
	private final class MapGraphics extends JComponent {
		// This class does all the graphics for the map and vehicle displaying	
		
		// Swing has the origin in the upper left corner of the screen with the x axis growing rightwards and the y axis growing downwards
		// This simulation has the x axis growing downwards and the y axis growing rightwards, so in order to paint everything the right way
		// all calls to drawLine and fillrect must switch the coordinates in the function arguments when calling them
		LinkedList<MapNode> path;
		
		MapGraphics() {}

		@Override
		public void paint(Graphics G) {
			// Paint the between the nodes
			MapNode currentNode;
			G.setColor(Color.WHITE);			
			for (int i=0;i<start.params.mapParameters.numberOfMapNodes;i++) {
				currentNode = mapGraph[i];
				// Every edge in the mapGraph will be drawn with a white line by checking every nodes neighbours
				for (MapNode neighbourNode : currentNode.neighbors) {
					G.drawLine(currentNode.posY, currentNode.posX, neighbourNode.posY, neighbourNode.posX);
				}				
			}
			
			// Paint the nodes
			G.setColor(Color.BLACK);
			for (int i=0;i<start.params.mapParameters.numberOfMapNodes;i++) {
				if (mapGraph[i].markThisNode) {
					// Mark the node for debugging
					G.setColor(Color.RED);
					G.drawLine(mapGraph[i].posY,mapGraph[i].posX, mapGraph[i].posY, mapGraph[i].posX);
					G.drawLine(mapGraph[i].posY+1, mapGraph[i].posX, mapGraph[i].posY+1, mapGraph[i].posX);
					G.drawLine(mapGraph[i].posY, mapGraph[i].posX+1, mapGraph[i].posY, mapGraph[i].posX+1);
					G.drawLine(mapGraph[i].posY+1, mapGraph[i].posX+1, mapGraph[i].posY+1, mapGraph[i].posX+1);
					G.setColor(Color.BLACK);
					continue;
				} 				
				G.drawLine(mapGraph[i].posY, mapGraph[i].posX, mapGraph[i].posY, mapGraph[i].posX);
			}	
			
			// Paint two corners in the map
			G.drawLine(0,0,0,0);
			G.drawLine(start.params.mapParameters.mapWidth, start.params.mapParameters.mapHeight, start.params.mapParameters.mapWidth, start.params.mapParameters.mapHeight);

			// Paint the mapTransmitters
			for (int i=0;i<start.params.mapTransmitterParameters.numberOfTransmitters;i++) {
				G.setColor(Color.LIGHT_GRAY);
				G.drawLine(mapTransmitters[i].posY,mapTransmitters[i].posX,mapTransmitters[i].posY,mapTransmitters[i].posX);
				G.drawLine(mapTransmitters[i].posY+1,mapTransmitters[i].posX,mapTransmitters[i].posY+1,mapTransmitters[i].posX);
				G.drawLine(mapTransmitters[i].posY,mapTransmitters[i].posX+1,mapTransmitters[i].posY,mapTransmitters[i].posX+1);
				G.drawLine(mapTransmitters[i].posY+1,mapTransmitters[i].posX+1,mapTransmitters[i].posY+1,mapTransmitters[i].posX+1);
				
				// Draw the transmitter range as a circle
				G.drawOval(mapTransmitters[i].posY - mapTransmitters[i].range, mapTransmitters[i].posX - mapTransmitters[i].range, 2*mapTransmitters[i].range, 2*mapTransmitters[i].range);
			}
			
			// Paint the vehicles
			Graphics2D g2d = (Graphics2D) G.create();
			int thisCarPosX=0, thisCarPosY=0, thisCarLength=0, thisCarWidth=0; // Paramaters that make the painting possible
			double thisCarYaw=0;
			boolean markThisVehicle=false;
			
			for (int k=0;k<start.params.vehiceParameters.numberOfVehicles;k++) {
				switch (start.vehicleStates.vehicleStateArray[k].colorCode) {
				// Set color of the car
				case 0: 
					g2d.setColor(Color.BLUE);
					break;
				case 1: 
					g2d.setColor(Color.MAGENTA);
					break;
				case 2: 
					g2d.setColor(Color.RED);
					break;
				case 3: 
					g2d.setColor(Color.GREEN);
					break;
				case 4: 
					g2d.setColor(Color.CYAN);
					break;
				case 5: 
					g2d.setColor(Color.YELLOW);
					break;									
				case 6: 
					g2d.setColor(Color.ORANGE);
					break;
				case 7: 
					g2d.setColor(Color.PINK);
					break;
				case 8: 
					g2d.setColor(Color.LIGHT_GRAY);
					break;	
				case 9: 
					g2d.setColor(Color.WHITE);
					break;	
				case 10: 
					g2d.setColor(Color.BLACK);
					break;						
				}		
				
				// Draw the vehicle path
				if (k<5) {
//					path = start.controllerTalker.getController(k).getThePath();				
//					int currentX=0,currentY=0,nextX=0,nextY=0;
//					for (int i=0;i<path.size()-1;i++) {
//						currentX = path.get(i).posX;
//						currentY = path.get(i).posY;
//						nextX = path.get(i+1).posX;
//						nextY = path.get(i+1).posY;
//							
//						g2d.drawLine(currentY,currentX,nextY,nextX);
//					}	
				}					
				
				thisCarPosX = (int) start.vehicleStates.vehicleStateArray[k].posX; // Round to closest int
				thisCarPosY = (int) start.vehicleStates.vehicleStateArray[k].posY; // Round to closest int
				thisCarWidth = start.vehicleStates.vehicleStateArray[k].vehicleWidth;
				thisCarLength = start.vehicleStates.vehicleStateArray[k].vehicleLength;
				thisCarYaw = start.vehicleStates.vehicleStateArray[k].yaw;
				markThisVehicle = start.vehicleStates.vehicleStateArray[k].markThisVehicle;
				
		        g2d.rotate(-thisCarYaw);
		        g2d.fillRect((int) ( -thisCarPosX*Math.sin(thisCarYaw) + thisCarPosY*Math.cos(thisCarYaw))-thisCarWidth/2,(int) ( thisCarPosX*Math.cos(thisCarYaw) + thisCarPosY*Math.sin(thisCarYaw) )-thisCarLength/2, thisCarWidth, thisCarLength);
		        g2d.rotate(thisCarYaw);
		        
		        // Mark the vehicle
		        if (markThisVehicle) {
		        	G.setColor(Color.BLACK);
					G.drawLine(thisCarPosY,thisCarPosX,thisCarPosY,thisCarPosX+20);	
		        }
		        
				// Mark car center point
				G.setColor(Color.BLACK);
				G.drawLine(thisCarPosY,thisCarPosX,thisCarPosY,thisCarPosX);	
				
				// Draw the vehicle number next to the vehicle
				G.drawString(Integer.toString(k), thisCarPosY, thisCarPosX+15);
				
				// Draw a line between partner vehicles in the merging scenario
				VehicleStates.VehicleState partnerVehicle = start.controllerTalker.getController(k).getMergeControllerPartnerVehicle();
				if (partnerVehicle!=null) {
					if(start.controllerTalker.getController(k).isClearToMerge()) {
						G.setColor(Color.GREEN);
					}
					else {
						G.setColor(Color.RED);
					}
					
					G.drawLine(thisCarPosY, thisCarPosX, (int) partnerVehicle.posY, (int) partnerVehicle.posX);
				}

				// Draw the vehicles radar area
//				g2d.setColor(Color.WHITE);
//				g2d.rotate(-thisCarYaw);
//				g2d.fillRect((int) ( -thisCarPosX*Math.sin(thisCarYaw) + thisCarPosY*Math.cos(thisCarYaw))-thisCarWidth/2,(int) ( thisCarPosX*Math.cos(thisCarYaw) + thisCarPosY*Math.sin(thisCarYaw) )+thisCarLength/2, thisCarWidth, start.vehicleStates.vehicleStateArray[k].radarRangeFront);   
//				g2d.rotate(thisCarYaw);			
				
				// Draw the vehicles wifi area
//				G.setColor(Color.LIGHT_GRAY);
//				G.drawOval(thisCarPosY - start.params.vehiceParameters.wifiRange, thisCarPosX - start.params.vehiceParameters.wifiRange, 2*start.params.vehiceParameters.wifiRange, 2*start.params.vehiceParameters.wifiRange);
				
			}
			g2d.dispose();			
		}
	}	
}
