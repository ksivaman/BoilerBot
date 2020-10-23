#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <math.h>
#include "rplidar_cmd.h"
#include "RPLidar_c.h"
#include "AnalyzeLiDAR.h"
// #include "driver/gpio.h"
// #include "sdkconfig.h"

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/event_groups.h"
// #include "esp_system.h"
// #include "esp_wifi.h"
// #include "esp_event.h"
// #include "esp_log.h"
// #include "nvs_flash.h"

// #include "lwip/err.h"
// #include "lwip/sys.h"
// #include "esp_http_client.h"

static int comp (const void *a, const void *b){
    return ((pp*)a)->angle - ((pp*)b)->angle;
}

pp* reformatSamples(rplidar_response_measurement_node_t* samples, short numSamples){
    pp* single_scan_data = (pp*) malloc(sizeof(pp) * numSamples);
    for(short i = 0; i < numSamples; i++) {
        single_scan_data[i].angle = (samples[i].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
        single_scan_data[i].distance = samples[i].distance_q2/4.0f;
    }
    
    return single_scan_data;
}

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
        printf("Angle: %d,  Distance: %f\n", i, minDist);
    }
    return;
}

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

// Compare quantized scan of curr loc with the quantized scans of the surrounding squares from the prev loc
char decideLoc(rplidar_response_measurement_node_t* scanSamples, short numSamples, float* surroundingScans[], char numSurrScans) {    
    // Quantize current scan
    float* procScanSamp = quantizeScan(scanSamples, numSamples);

    int minSqDiff = INT_MAX;
    char bestMatchID = 0;
    // Go through the initialization scan of each surrounding square
    for(char surrScanID = 0; surrScanID < numSurrScans; surrScanID++) {
        // Wraparound comparison since LiDAR won't always face same direction
        for(short zeroDegIdx = 0; zeroDegIdx < FINAL_NUM_POINTS; zeroDegIdx++) {
            int sqDiff = 0;
            // Compare 360 samples in current scan with those in surrounding square
            for(short i = 0; i < FINAL_NUM_POINTS; i++) {
                sqDiff += pow(procScanSamp[(zeroDegIdx + i) % FINAL_NUM_POINTS] - surroundingScans[(int)surrScanID][i], 2);
            }
            if(sqDiff < minSqDiff) {
                minSqDiff = sqDiff;
                bestMatchID = surrScanID;
            }
        }        
    }
    //free(procScanSamp);
    
    return bestMatchID;
}