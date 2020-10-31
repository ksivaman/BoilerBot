#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <math.h>
#include "include/rplidar_cmd.h"
#include "include/RPLidar_c.h"
#include "include/AnalyzeLiDAR.h"

static int comp (const void *a, const void *b){
    return ((pp*)a)->angle - ((pp*)b)->angle;
}

// Converts 400 raw data samples (distance & angle) from: array of RPLidar library structs --> array of floats
// single_scan_data[i] = x   ------>   i = angle, x = distance
pp* reformatSamples(rplidar_response_measurement_node_t* samples, short numSamples){
    pp* single_scan_data = (pp*) malloc(sizeof(pp) * numSamples);
    for(short i = 0; i < numSamples; i++) {
        single_scan_data[i].angle = (samples[i].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
        single_scan_data[i].distance = samples[i].distance_q2/4.0f;
    }
    
    return single_scan_data;
}

// Downsizes unquantized scan (400 samples) to quantized scan (360 samples)
void process(pp* input, short size, float* single_scan_data_proc){
    for (int i = 0; i < FINAL_NUM_POINTS; i++) {
        float minAngleDiff = __INT_MAX__;
        float minDist = __INT_MAX__;
        for (short j = 0; j < size; j++) {
            if (abs(input[j].angle - i) < minAngleDiff) {
                minAngleDiff = abs(input[j].angle - i);
                minDist = input[j].distance;
            }
        }
        single_scan_data_proc[i] = minDist;
    }
    return;
}

// Quantizes a scan from 400 raw data samples ----> 360 samples that are 0-degree-based
float* quantizeScan(rplidar_response_measurement_node_t* scanSamples, short numSamples){
    pp* single_scan_data = reformatSamples(scanSamples, numSamples);
    qsort(single_scan_data, numSamples, sizeof(pp), comp);
    float* single_scan_data_proc = (float*) malloc(sizeof(float) * FINAL_NUM_POINTS);
    process(single_scan_data, numSamples, single_scan_data_proc);
    // Free single_scan_data and node samples in buffer
    for(short i = 0; i < numSamples; i++) {
        //free(&(single_scan_data[i]));

        // Don't do this, or else it will free the buffer where scans are read from
        //free(&(scanSamples[i]));
    }
    return single_scan_data_proc;
}

// Determine current location by comparing current quantized scan with the quantized initialization scans
char getCurrLoc(rplidar_response_measurement_node_t* scanSamples, short numSamples, int fplan[NROWS][NCOLS], float initScans[NROWS][NCOLS][FINAL_NUM_POINTS], char numInitScans) {    
    // Quantize current scan
    float* procScanSamp = quantizeScan(scanSamples, numSamples);

    // Stats for initialization scan best match
    int minDiff = INT_MAX;
    char bestMatchID = 0;
    
    // For each square in floor plan
    for(int initScanID = 0; initScanID < numInitScans; initScanID++) {
        // Skip invalid squares in floor plan
        if(fplan[initScanID / NCOLS][initScanID % NCOLS] == 1) {
            printf("Skipping Obstacle Square (%d, %d)\n", initScanID / NCOLS, initScanID % NCOLS);
            fflush(stdout);
            continue;
        }
        
        int minDiffForCurrInitScan = INT_MAX;

        // Compare current quantized scan with quantized initialization scan
        // Outer for loop: Accounts for bot not facing directly north (simulates angle alignment by altering the current scan's 0-degree-index)
        //           TODO: modify the zeroDegIdx to be a window based on compass reading
        for(int zeroDegIdx = 0; zeroDegIdx < FINAL_NUM_POINTS; zeroDegIdx++) {      
            int diff = 0;
            // Inner for loop: Compare 360 samples in angle-aligned current scan with those in initialization scan
            for(int i = 0; i < FINAL_NUM_POINTS; i++) {
                diff += pow(abs(procScanSamp[(zeroDegIdx + i) % FINAL_NUM_POINTS] - initScans[initScanID / NCOLS][initScanID % NCOLS][i]), 1);
            }
            if(diff < minDiffForCurrInitScan) {
                minDiffForCurrInitScan = diff;
            }
        }
        if(minDiffForCurrInitScan < minDiff) {
            minDiff = minDiffForCurrInitScan;
            bestMatchID = initScanID;
        }
        printf("\nDiff for (%d, %d): %d\n", initScanID / NCOLS, initScanID % NCOLS, minDiffForCurrInitScan);
        fflush(stdout);

    }
    
    return bestMatchID;
}