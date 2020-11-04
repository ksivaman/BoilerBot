#ifndef __ANALYZELIDAR_H__
#define __ANALYZELIDAR_H__

#include "rplidar_cmd.h"
#include "constants.h"
#include "astar.h"


typedef struct _pp {
    float angle;
    float distance;
} pp;

enum compass{NORTH = 0, SOUTH = 180, EAST = 90, WEST = 270};

void init_lidar(void);
static int comp (const void *a, const void *b);

pp* reformatSamples(rplidar_response_measurement_node_t* samples, int numSamples); // no more used
void reformatScan();

void process(pp* input, int size, float* single_scan_data_proc);

float* quantizeScan(rplidar_response_measurement_node_t* scanSamples, int numSamples); // no more used
float* getLiDARScan();
void updateLiDARScan(float* prevScan);

Point getCurrLoc();

float getFrontDist(float* procScanSamp);
float getBackDist(float* procScanSamp);
void doINeedToStop(float* procScanSamp, float prevFront, float prevBack, int mm, bool* obstacleFlag, bool* distanceDoneFlag);
int angleDiff(float* prevScan, float* newScan);
int absoluteErrorFrom(float * procScanSamp, Point p, int * angle);
Point get_curr_loc(int * angleReturn, Point * secondClose);
Point get_curr_loc_input(enum compass heading, int* angleReturn, Point * secondClose);
void getOffSetFrom(float * procScanSamp, Point p, int * offsetX, int * offsetY);

#endif