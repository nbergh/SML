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

void loadMap(unsigned char**& map, int& nrRows, int& nrCols) {
	std::ifstream mapIn("/home/sml-linux/Matlab/cvap_map_in");
	int inval;
	mapIn >> nrRows;
	mapIn >> nrCols;

	map = new unsigned char*[nrRows];
	for (int i=0;i<nrRows;i++) {
		map[i] = new unsigned char[nrCols];
	}

	for (int i=0;i<nrRows;i++) {
		for (int j=0;j<nrCols;j++) {
			mapIn >> inval;
			map[i][j]=inval;
		}
	}

	mapIn.close();
}

void printMap(unsigned char** costMap, int mapMaxX, int mapMaxY) {
	std::ofstream mapOut;
	mapOut.open("/home/sml-linux/Matlab/cvap_map_out.m");

	mapOut << "close all, clear all, hold on, axis equal" << std::endl;
	mapOut << "Map=[..." << std::endl;
	for (int i=0;i<mapMaxX;i++) {
		for (int j=0;j<mapMaxY;j++) {
			mapOut << (int)costMap[i][j] << " ";
		}
		mapOut << ";" << std::endl;
	}
	mapOut << "];" << std::endl;
	mapOut.close();
}


int main(void)
{
	int nrRows,nrCols;
	unsigned char** map=0;

	loadMap(map,nrRows,nrCols);
//	generateRandomMap(map,mapMaxX,mapMaxY);
//	printMap(map,mapMaxX,mapMaxY);

	PathPlanner pathPlanner(OCCUPANCY_GRID_CELL_SIZE);

	timeval startTime,endTime;
	gettimeofday(&startTime,NULL);

	pathPlanner.planNewPath(map,nrRows,nrCols,0,0,0,0,8,18);
	gettimeofday(&endTime,NULL);

	int iterationTime = (endTime.tv_sec*1000000 + endTime.tv_usec) - (startTime.tv_sec*1000000 + startTime.tv_usec);

	std::cout << "Time: "<<iterationTime<<std::endl;

}


//void printObstaclesInMap(unsigned char** map, int maxX, int maxY) {
//	std::ofstream myfile;
//	myfile.open("/home/sml-linux/Matlab/cvap_map_out.m");
//
//	myfile << "close all, hold on, axis equal" << std::endl;
//	float x,y;
//	for (int i=0;i<maxX;i++) {
//		for (int j=0;j<maxY;j++) {
//			if (map[i][j] ==255) {
//				x = i*OCCUPANCY_GRID_CELL_SIZE;
//				y = j*OCCUPANCY_GRID_CELL_SIZE;
//				myfile << "plot([" << x << "," << x + OCCUPANCY_GRID_CELL_SIZE << "],[" << y << "," << y << "], 'r')" << std::endl;
//				myfile << "plot([" << x << "," << x << "],[" << y << "," << y + OCCUPANCY_GRID_CELL_SIZE << "],'r')" << std::endl;
//				myfile << "plot([" << x + OCCUPANCY_GRID_CELL_SIZE << "," << x + OCCUPANCY_GRID_CELL_SIZE << "],[" << y << "," << y + OCCUPANCY_GRID_CELL_SIZE << "],'r')" << std::endl;
//				myfile << "plot([" << x << "," << x + OCCUPANCY_GRID_CELL_SIZE << "],[" << y + OCCUPANCY_GRID_CELL_SIZE << "," << y + OCCUPANCY_GRID_CELL_SIZE << "],'r')" << std::endl;
//			}
//		}
//	}
//
//	myfile.close();
//}


