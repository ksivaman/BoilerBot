#include <limits.h>
#include <math.h>
#include "RPLidar_c.h"

#define FINAL_NUM_POINTS 360

// Compare quantized scan of curr loc with the quantized scans of the surrounding squares from the prev loc
char decideLoc(rplidar_response_measurement_node_t* scanSamples, short numSamples, float* surroundingScans[], char numSurrScans) {    
    // Quantize current scan
    float* procScanSamp = quantizeScan(scanSamples, numSamples);

    int minSqDiff = INT_MAX;
    char bestMatchID = 0;
    // Go through the initialization scan of each surrounding square
    for(char surrScanID = 0; surrScanID < numSurrScans; surrScanID++) {
        int sqDiff = 0;
        // Compare 360 samples in current scan with those in surrounding square
        for(short i = 0; i < FINAL_NUM_POINTS; i++) {
            sqDiff += pow(procScanSamp[i] - surroundingScans[surrScanID][i], 2);
        }
        if(sqDiff < minSqDiff) {
            minSqDiff = sqDiff;
            bestMatchID = surrScanID;
        }
    }
    free(procScanSamp);
    
    return bestMatchID;
}