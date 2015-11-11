#ifndef PATHPLANNING_H_
#define PATHPLANNING_H_

#include "Structs.h"

#define HEAP_SIZE 5000 // The heap has limited space
#define HASH_TABLE_ENTRIES 5000 // The hash table has unlimited space, but a limited number of possible hashes

struct aStarNode {
	/* Everything is initialized to zero on struct initialization
	 * x and y are later set by HashTable::addAstarNode()
	 * heapArrayIndex is set by MinHeap::bubbleNode() or MinHeap::popNode()
	 * Everything else is set by PathPlanning::discoverNeighbor()
	 */
	float x, y, distanceFromStartNode, heuristic; // TODO make x,y const
	bool isOnOpenSet, isOnClosedSet, pathIsReversingFromPrevNode;
	double angleFromPreviosNodeInPath;
	int heapArrayIndex;
	const aStarNode *previousNodeInPath;
};

class PathPlanning {
	class HashTable {
		struct HashBucket {
			aStarNode node; // aStarNodes are stored here
			HashBucket *nextBucket; // A linked list of buckets that share the same index
		};

		HashBucket* hashArray[HASH_TABLE_ENTRIES]; // An array of hashBucket pointers

		int getIndex(float x, float y) const;
		aStarNode *getAstarNode(float x, float y) const;
		void clearBucketList(HashBucket* bucket) const;

		public:
			HashTable();
			~HashTable();

			void clearHashTable();
			aStarNode *addAstarNode(float x, float y);
	};
	class MinHeap {
		aStarNode* heapArray[HEAP_SIZE]; // The heap array, with pointers to aStarNodes
		int currentNrOfNodesInHeap;

		public:
			MinHeap();
			~MinHeap();

			void clearMinHeap();
			bool addNode(aStarNode* node); // Returns false if the heap is full. Also stores heapArrayIndex in the aStarNode
			void bubbleNode(aStarNode* node); // Updates the heap based on the new (lower) heuristic, and updates heapArrayIndex in the node
			aStarNode* popNode();
	};

	// External vars
	const VehicleState& vehicleState; // Current vehicle state
	const ObstaclePoint* obstacleSquaresOnGPU;
	const int& currentNrOfObstacles;

	// Internal vars
	HashTable* hashTable;
	MinHeap* minHeap;
	GPSposition* macroPathGPS; // The main GPS path (start to goal) as a linked list
	GPSposition* microPathGPS; // The path to the next point in the mainGPSpath as a linked list
	PathPoint* microPathXY; // The same path as microPathGPS, but in x,y coordinates
	int lengthOfMacroPath,lengthOfMicroPath,currentIndexInMacroPath,currentIndexInMicroPath; // The attributes of the paths

	// Functions
	bool generateMicroPath(float targetX, float targetY, const ObstaclePoint* obstacleSquaresOnDevice, int nrOfObstacles, float minDistanceToObstacle, float obstacleMatrixResolution);
	bool discoverNeighbor(const aStarNode* baseNode, const aStarNode* targetNode, int index, const ObstaclePoint* obstacleSquaresOnDevice, int nrOfObstacles, float minDistanceToObstacle, float obstacleMatrixResolution) const;
	void translateMicroPathToXY() const;
	void translateXYtoGPSposition(float x, float y, GPSposition& target) const;
	void translateGPSpositionToXY(double latc, double longc, PathPoint& target) const;

	public:
		PathPlanning(const VehicleState& vehicleState, const ObstaclePoint* obstaclePoints, const int& currentNrOfObstacles);
		~PathPlanning();

		bool setMacroPath(const char* filePath);
		void updatePathAndControlSignals() const;
		const PathPoint* getMicroPathXY() const;

};

#endif /* PATHPLANNING_H_ */
