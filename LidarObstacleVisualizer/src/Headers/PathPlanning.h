#ifndef PATHPLANNING_H_
#define PATHPLANNING_H_

#include "StructDefinitions.h"

#define HEAP_SIZE 5000 // The heap has limited space
#define HASH_TABLE_ENTRIES 5000 // The hash table has unlimited space, but a limited number of possible hashes

struct aStarNode {
	/* Everything is initialized to zero on struct initialization
	 * x and y are later set by HashTable::addAstarNode()
	 * heapArrayIndex is set by MinHeap::bubbleNode() or MinHeap::popNode()
	 * Everything else is set by PathPlanning::discoverNeighbor()
	 */
	float x, y, distanceFromStartNode, heuristic;
	bool isOnOpenSet, isOnClosedSet, pathIsReversingFromPrevNode;
	double angleFromPreviosNodeInPath;
	int heapArrayIndex;
	aStarNode* previousNodeInPath;
};

class PathPlanning {
	class HashTable {
		struct HashBucket {
			aStarNode node; // aStarNodes are stored here
			HashBucket* nextBucket; // A linked list of buckets that share the same index
		};

		HashBucket* hashArray[HASH_TABLE_ENTRIES];

		int getIndex(float x, float y); // Hasher function that translates x,y to an index in the hashArray
		void clearBucketList(HashBucket* bucket);
		aStarNode* getAstarNode(float x, float y);

		public:
			HashTable();
			~HashTable();

			void clearHashTable();
			aStarNode* addAstarNode(float x, float y);
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
	VehicleState* vehicleState; // Current vehicle state
	Path* mainGPSpath; // The main GPS path (start to goal) as a linked list

	// Internal vars
	HashTable* hashTable;
	MinHeap* minHeap;
	Path* intraGPSpath; // The path to the next point in the mainGPSpath as a linked list

	// Functions
	void discoverNeighbor(aStarNode* baseNode, aStarNode* targetNode, int index, OpenGLvertex* obstacleSquaresOnDevice, int nrOfObstacles, float minDistanceToObstacle, float obstacleMatrixResolution);
	void generatePath(float targetX, float targetY, OpenGLvertex* obstacleSquaresOnDevice, int nrOfObstacles, float minDistanceToObstacle, float obstacleMatrixResolution);
	void buildPath()

	public:
		PathPlanning(VehicleState* vehicleState, Path* mainGPSpath);
		~PathPlanning();

		void updatePathAndControlSignals(OpenGLvertex* obstacleSquares, int nrOfObstacles);
};

#endif /* PATHPLANNING_H_ */
