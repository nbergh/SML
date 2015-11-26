#ifndef PATHPLANNING_H_
#define PATHPLANNING_H_

#include "Structs.h"

#define HEAP_SIZE 20000 // The heap has limited space
#define HASH_TABLE_ENTRIES 20000 // The hash table has unlimited space, but a limited number of possible hashes

class PathPlanning {
	struct aStarNode {
		/* Everything is initialized to zero on struct initialization
		 * x and y are later set by HashTable::addAstarNode()
		 * heapArrayIndex is set by MinHeap::bubbleNode() or MinHeap::popNode()
		 * Everything else is set by PathPlanning::discoverNeighbor()
		 *
		 * localPathAngleFromPreviousNodeInPath is the angle from {the vector going from
		 * the previousNodeInPath to this node} to {the vector going forward from the vehicle
		 * along its longitudinal centerline}. It is not a heading, since it has no relation
		 * with north, and only has a meaning in the vehicle local coordinate system
		 */
		float x, y, distanceFromStartNode, heuristic; // TODO make x,y const
		double localPathAngleFromPreviousNode;
		bool isOnOpenSet, isOnClosedSet, pathIsReversingFromPrevNode;
		int heapArrayIndex;
		aStarNode* previousNodeInPath;
	};
	class HashTable {
		struct HashBucket {
			aStarNode node; // aStarNodes are stored here
			HashBucket* nextBucket; // A linked list of buckets that share the same index
		};

		HashBucket* hashArray[HASH_TABLE_ENTRIES]; // An array of hashBucket pointers

		int getIndex(int gridX, int gridY) const;
		void clearBucketList(HashBucket* bucket) const;

		public:
			HashTable();
			~HashTable();

			void clearHashTable();
			aStarNode& addAstarNode(float x, float y);
	};
	class MinHeap {
		aStarNode* heapArray[HEAP_SIZE]; // The heap array, with pointers to aStarNodes
		int currentNrOfNodesInHeap;

		public:
			MinHeap();
			~MinHeap();

			void clearMinHeap();
			bool addNode(aStarNode& node); // Returns false if the heap is full. Also stores heapArrayIndex in the aStarNode
			void bubbleNode(aStarNode& node); // Updates the heap based on the new (lower) heuristic, and updates heapArrayIndex in the node
			aStarNode* popNode();
			int getAvailableSpace() {return HEAP_SIZE-currentNrOfNodesInHeap;}
	};

	const LidarExportData& lidarExportData;
	const VehicleState& vehicleState;
	VehicleStatus& vehicleStatus;
	PathExportData pathExportData;
	HashTable hashTable;
	MinHeap minHeap;


	PathPointInGPScords* macroPathGPS; // The main GPS path (start to goal) as a linked list
	PathPointInGPScords* microPathGPS; // The path to the next point in the mainGPSpath as a linked list
	PathPointInLocalXY* microPathXY; // The same path as microPathGPS, but in x,y coordinates
	PathPointInLocalXY* macroPathXY; // The same path as macroPathGPS, but in x,y coordinates
	int lengthOfMacroPath,lengthOfMicroPath,currentIndexInMacroPath,currentIndexInMicroPath; // The attributes of the paths

	// Functions
	bool updatePathIndexes();
	void translateMacroPathToXY();
	bool translateMicroPathToXYandCheckIfMicroPathIsTooCloseToObstacles() const;
	bool generateMicroPath(float targetX, float targetY);
	void discoverNeighbor(aStarNode& baseNode, const aStarNode& targetNode, int index);
	bool checkIfaStarNodeIsTooCloseToObstacles(const aStarNode& node, double localPathAngleFromPreviousNode) const;
	void clearAllPaths(bool includeMacroPath);

	public:
		PathPlanning(const LidarExportData& lidarExportData, const VehicleState& vehicleState, VehicleStatus& vehicleStatus);
		~PathPlanning();

		void updatePathAndControlSignals();
		void setMacroPath(const char* filePath);
		const PathExportData& getPathExportData() {return pathExportData;}

};

#endif /* PATHPLANNING_H_ */
