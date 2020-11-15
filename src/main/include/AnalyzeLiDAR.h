#ifndef __ANALYZELIDAR_H__
#define __ANALYZELIDAR_H__

#include "rplidar_cmd.h"
#include "constants.h"
#include "astar.h"


typedef struct _pp {
    float angle;
    float distance;
} pp;

enum dir{FORWARD, BACKWARD, RIGHT, LEFT, STOP};
enum compass{NORTH = 0, SOUTH = 180, EAST = 90, WEST = 270};

typedef struct {
    ledc_channel_config_t pwm;
    gpio_num_t motor_1;
    gpio_num_t motor_2;
    gpio_num_t motor_3;
    gpio_num_t motor_4;
    enum compass heading;
    Point currLoc;
} rover;

void init_lidar(void);
static int comp (const void *a, const void *b);

pp* reformatSamples(rplidar_response_measurement_node_t* samples, int numSamples); // no more used
void reformatScan();

void process(pp* input, int size, float* single_scan_data_proc);

float* quantizeScan(rplidar_response_measurement_node_t* scanSamples, int numSamples); // no more used
float* getLiDARScan();
void updateLiDARScan(float* prevScan);

Point getCurrLoc();

void getFrontBackDist(float * frontFinal, float *backFinal);
float getFrontDist(float* procScanSamp);
float getBackDist(float* procScanSamp);
bool doINeedToStop(float* procScanSamp, float prevFront, float prevBack, int mm, bool* obstacleFlag, bool* distanceDoneFlag);
bool isThereObstacle_s(float* procScanSamp, int tolerance, enum dir direction);
bool isThereObstacle_r(float* procScanSamp, int tolerance, enum dir direction);
int angleDiff(float* prevScan, float* newScan);
int absoluteErrorFrom(float * procScanSamp, Point p, int * angle);
Point get_curr_loc(int * angleReturn, Point * secondClose);
Point get_curr_loc_input(enum compass heading, int* angleReturn, Point * secondClose);
void getOffSetFrom(float * procScanSamp, Point p, int * offsetNorth, int * offsetEast);
float offsetHelper(float * lidarScan, int range, int zeroDegIdx, enum compass direction, bool mode);
void getOffsets(rover robot, int *offsetNorth, int * offsetEast);
int getAngleOffset(rover robot);

#endif