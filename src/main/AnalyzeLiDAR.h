#ifndef __ANALYZELIDAR_H__
#define __ANALYZELIDAR_H__

#include "rplidar_cmd.h"

#define FINAL_NUM_POINTS 360
#define ANGLE 0
#define DIST 1

typedef struct _pp {
    float angle;
    float distance;
} pp;

static int comp (const void *a, const void *b);
pp* reformatSamples(rplidar_response_measurement_node_t* samples, short numSamples);
void process(pp* input, short size, float* single_scan_data_proc);
float* quantizeScan(rplidar_response_measurement_node_t* scanSamples, short numSamples);
char decideLoc(rplidar_response_measurement_node_t* scanSamples, short numSamples, float* surroundingScans[], char numSurrScans);

#endif