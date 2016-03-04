#ifndef LIDARPROCESSING_H_
#define LIDARPROCESSING_H_

#include "Structs.h"
#include "UDPReceiver.h"

#define LIDAR_UDP_PORT 2368

class LidarProcessing {
	class LidarUDPReceiver : public UDPReceiver {
		friend class LidarProcessing;

		char* rawLidarPacketBuffer;
		int currentRawBufferOffset=0;

		LidarUDPReceiver(char* rawLidarPacketBuffer) : UDPReceiver(LIDAR_UDP_PORT,1206), rawLidarPacketBuffer(rawLidarPacketBuffer) {startReceiverThread();}
		~LidarUDPReceiver() {}

		virtual bool isValidPacket(const char* packetBuffer);
		virtual void actionWhenReceived(const char* packetBuffer);
	};

	//Pointers to device and host memory:
	size_t sizeofRawLidarPacketBuffer,sizeofLidarPointsListForGraphics,sizeofObstacleSquaresListForGraphics;
	int nrOfCellsInOccupancyGrid;

	char *rawLidarPacketBuffer, *rawLidarPacketBufferOnGPU;
	LidarPointForGraphics *lidarPointsListForGraphics, *lidarPointsListForGraphicsOnGPU;
	ObstaclePointForGraphics *obstaclePointListForGraphics, *obstaclePointListForGraphicsOnGPU;
	int *occupancyGridForMaxZOnGPU, *occupancyGridForMinZOnGPU; // Has to be int because atomicMax and atomicMin only accept integer arguments
	char *occupancyGridOnGPU;

	// Members:
	LidarUDPReceiver* lidarUDPReceiver;
	LidarExportData lidarExportData = {};

	void allocateMemory();
	void translateLidarDataFromRawToXYZ();
	void identifyObstaclesInLidarData();

	public:
		LidarProcessing();
		~LidarProcessing();
		void processLidarData();
		const LidarExportData& getLidarExportData() {return lidarExportData;}
};


#endif /* LIDARPROCESSING_H_ */
