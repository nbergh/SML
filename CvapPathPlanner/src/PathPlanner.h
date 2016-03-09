#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_

#include <fstream>

namespace PARAMETERS {
	const int NR_OCCUPANCY_GRID_CELLS_X_WISE = 750;
	const int NR_OCCUPANCY_GRID_CELLS_Y_WISE = 1000;
	const int ASTAR_EXPANSION_STEP_LENGTH = 3;
	const float ROBOT_WIDTH=0.8; // In meters
	const float ROBOT_LENGTH=1.5; // In meters
	const int HEAP_SIZE = 10000;
}

class PathPlanner {
public:
	PathPlanner(float occupancyGridCellSize);
	~PathPlanner();
	void planNewPath(unsigned char** costMap, int costMapNrOfCellsXwise, int costMapNrOfCellsYwise, float costMapLowestX, float costMapLowestY, float startX, float startY, float goalX, float goalY);

private:
	struct aStarNode {
		float x,y,distanceFromStartNode,pathCostFromStartNode,heuristic;
		bool isOnOpenSet,isOnClosedSet;
		int heapArrayIndex;
		aStarNode* prevNodeInPath;
	};
	struct robotCollisionCheckingPoint {
		float x,y;
	};
	struct mapData {
		unsigned char** costMap;
		int nrOfCellsXwise,nrOfCellsYwise;
		float lowestXinMap,lowestYinMap,pathStartX,pathStartY,pathGoalX,pathGoalY;
	};
	class MinHeap {
	public:
		MinHeap() : currentNrOfNodesInHeap(0) {}
		~MinHeap() {}

		void clearMinHeap() {currentNrOfNodesInHeap=0;}
		void addNode(aStarNode& node); // Returns false if the heap is full. Also stores heapArrayIndex in the aStarNode
		void bubbleNode(aStarNode& node); // Updates the heap based on the new (lower) heuristic, and updates heapArrayIndex in the node
		aStarNode* popNode();
		int getAvailableSpace() {return PARAMETERS::HEAP_SIZE-currentNrOfNodesInHeap;}

	private:
		aStarNode* heapArray[PARAMETERS::HEAP_SIZE]; // The heap array, with pointers to aStarNodes
		int currentNrOfNodesInHeap;
	};

	aStarNode** grid;
	mapData mapData;
	MinHeap minHeap;
	robotCollisionCheckingPoint* robotCollisionCheckingPoints;
	int nrOfCollisionCheckingPoints;

	std::ofstream debugSearchOut; // The astar search is written in matlab format to this file

	const float occupancyGridCellSize; //In meters

	void zeroOutGrid();
	void intializeRobotCollisionCheckingPoints();
	aStarNode* getNodeFromGrid(float x, float y);
	void discoverNeighbor(aStarNode& baseNode, int index, float goalX, float goalY);
	float getCostForNode(const aStarNode* neighborNode, float robotHeading);
	int getValueFromGlobalMap(float localX, float localY);

	// Debug functions:
	void debugPrintDeleteExpansion(const aStarNode& baseNode, const aStarNode& neighborNode);
	void debugPrintExpansion(const aStarNode& baseNode, const aStarNode& neighborNode);
	void debugPrintPath();
	void debugTestHeap();
};



#endif /* PATHPLANNER_H_ */
