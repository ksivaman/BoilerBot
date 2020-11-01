#ifndef __ANALYZELIDAR_H__
#define __ANALYZELIDAR_H__

#include "rplidar_cmd.h"
#include "constants.h"
#include "astar.h"

typedef struct _pp {
    float angle;
    float distance;
} pp;

void init_lidar(void);
static int comp (const void *a, const void *b);
pp* reformatSamples(rplidar_response_measurement_node_t* samples, short numSamples);
void process(pp* input, short size, float* single_scan_data_proc);
float* quantizeScan(rplidar_response_measurement_node_t* scanSamples, short numSamples);
Point getCurrLoc();

#endif