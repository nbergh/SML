/*
 ============================================================================
 Name        : CvapPathPlanner.cu
 Author      : Niklas Bergh
 ============================================================================
 */

#include "PathPlanner.h"

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <sys/time.h>

#define OCCUPANCY_GRID_CELL_SIZE 0.05

void generateRandomMap(unsigned char** map, int maxX, int maxY) {
	for (int i=0;i<maxX;i++) {
		for (int j=0;j<maxY;j++) {
			map[i][j] = rand()%100;
//			if (rand()%10==0) {map[i][j]=255;}
			if (i>=99 && i<=101 && j<200) {map[i][j]=255;}
		}
	}
}

void printObstaclesInMap(unsigned char** map, int maxX, int maxY) {
	std::ofstream myfile;
	myfile.open("/home/sml-linux/Matlab/cvap_map_out.m");

	myfile << "close all, hold on, axis equal" << std::endl;
	float x,y;
	for (int i=0;i<maxX;i++) {
		for (int j=0;j<maxY;j++) {
			if (map[i][j] ==255) {
				x = i*OCCUPANCY_GRID_CELL_SIZE;
				y = j*OCCUPANCY_GRID_CELL_SIZE;
				myfile << "plot([" << x << "," << x + OCCUPANCY_GRID_CELL_SIZE << "],[" << y << "," << y << "], 'r')" << std::endl;
				myfile << "plot([" << x << "," << x << "],[" << y << "," << y + OCCUPANCY_GRID_CELL_SIZE << "],'r')" << std::endl;
				myfile << "plot([" << x + OCCUPANCY_GRID_CELL_SIZE << "," << x + OCCUPANCY_GRID_CELL_SIZE << "],[" << y << "," << y + OCCUPANCY_GRID_CELL_SIZE << "],'r')" << std::endl;
				myfile << "plot([" << x << "," << x + OCCUPANCY_GRID_CELL_SIZE << "],[" << y + OCCUPANCY_GRID_CELL_SIZE << "," << y + OCCUPANCY_GRID_CELL_SIZE << "],'r')" << std::endl;
			}
		}
	}

	myfile.close();
}

//void printMapMatlab(unsigned char** costMap, int mapMaxX, int mapMaxY) {
//	std::ofstream myfile;
//	myfile.open("/home/sml-linux/Matlab/cvapout.m");
//
//	myfile << "Map=[..." << std::endl;
//	for (int i=0;i<mapMaxX;i++) {
//		for (int j=0;j<mapMaxY;j++) {
//			myfile << (int)costMap[i][j] << " ";
//		}
//		myfile << ";" << std::endl;
//	}
//	myfile << "];" << std::endl;
//	myfile.close();
//}

int main(void)
{
	const int mapMaxX=1000;
	const int mapMaxY=800;

	unsigned char** map = new unsigned char*[mapMaxX];
	for (int i=0;i<mapMaxX;i++) {
		map[i] = new unsigned char[mapMaxY];
	}

	generateRandomMap(map,mapMaxX,mapMaxY);
	printObstaclesInMap(map,mapMaxX,mapMaxY);
//	printMapMatlab(map,mapMaxX,mapMaxY);

	PathPlanner pathPlanner(OCCUPANCY_GRID_CELL_SIZE);

	timeval startTime,endTime;
	gettimeofday(&startTime,NULL);

	pathPlanner.planNewPath(map,mapMaxX,mapMaxY,0,0,0,0,15,20);
	gettimeofday(&endTime,NULL);

	int iterationTime = (endTime.tv_sec*1000000 + endTime.tv_usec) - (startTime.tv_sec*1000000 + startTime.tv_usec);

	std::cout << "Time: "<<iterationTime<<std::endl;

}


