#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <math.h>
#include "include/rplidar_cmd.h"
#include "RPLidar_c.c"
#include "include/AnalyzeLiDAR.h"
#include "include/main.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "include/astar.h"
#include "include/lidar_data.h"


#ifdef CONFIG_IDF_TARGET_ESP32
#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO       (5)
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#endif

static const int RX_BUF_SIZE = 1024;

#define TXD_PIN (GPIO_NUM_10)
#define RXD_PIN (GPIO_NUM_9)

//global variables for lidar stuff
uint8_t* buff = NULL;
rplidar_response_measurement_node_t * node = NULL;
pp* single_scan_reformed = NULL;

void init_lidar(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    buff = (uint8_t*) malloc(sizeof(rplidar_response_measurement_node_t) * NUM_SAMPLES);
    node = (rplidar_response_measurement_node_t*)(buff);
    single_scan_reformed = (pp*) malloc(sizeof(pp) * NUM_SAMPLES);

    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_12_BIT, // resolution of PWM duty
        .freq_hz = 10000,                     // frequency of PWM signal
        .speed_mode = LEDC_HS_MODE,           // timer mode
        .timer_num = LEDC_HS_TIMER,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
    };

    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = 
        {
            .channel    = LEDC_HS_CH0_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_HS_CH0_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_HS_TIMER
        };
    
    // // Set LED Controller with previously prepared configuration
    ledc_channel_config(&ledc_channel);

    // Initialize fade service.
    // ledc_fade_func_install(0);
    vTaskDelay(4000 / portTICK_PERIOD_MS);

    ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, 0);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
    
    // try to detect RPLIDAR...
    vTaskDelay(1000 / portTICK_PERIOD_MS); 

    // stop();
    vTaskDelay(1000 / portTICK_PERIOD_MS); 
    // startScan(false, RPLIDAR_DEFAULT_TIMEOUT*2);
    
    // start motor rotating at max allowed speed
    int duty = 1270;
    ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, duty);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
    vTaskDelay(2000 / portTICK_PERIOD_MS); 

}

static int comp (const void *a, const void *b){
    return ((pp*)a)->angle - ((pp*)b)->angle;
}

// Converts 400 raw data samples (distance & angle) from: array of RPLidar library structs --> array of floats
// single_scan_data[i] = x   ------>   i = angle, x = distance
pp* reformatSamples(rplidar_response_measurement_node_t* samples, int numSamples){
    pp* single_scan_data = (pp*) malloc(sizeof(pp) * numSamples);
    for(int i = 0; i < numSamples; i++) {
        single_scan_data[i].angle = (samples[i].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
        single_scan_data[i].distance = samples[i].distance_q2/4.0f;
    }
    
    return single_scan_data;
}

// (OPTIMIZED) Converts 400 raw data samples (distance & angle) from: array of RPLidar library structs --> array of floats
// single_scan_reformed[i] = x   ------>   i = angle, x = distance
void reformatScan(){
    rplidar_response_measurement_node_t* samples = node;
    for(int i = 0; i < NUM_SAMPLES; i++) {
        single_scan_reformed[i].angle = (samples[i].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
        single_scan_reformed[i].distance = samples[i].distance_q2/4.0f;
    }
}


// Downsizes unquantized scan (400 samples) to quantized scan (360 samples)
void process(pp* input, int size, float* single_scan_data_proc){
    for (int i = 0; i < FINAL_NUM_POINTS; i++) {
        float minAngleDiff = __INT_MAX__;
        float minDist = __INT_MAX__;
        for (int j = 0; j < size; j++) {
            if (fabs(input[j].angle - i) < minAngleDiff) {
                minAngleDiff = fabs(input[j].angle - i);
                minDist = input[j].distance;
            }
        }
        single_scan_data_proc[i] = minDist;
    }
    return;
}

// #define PROCESS_SEARCH_RANGE 6
// (OPTIMIZED) Downsizes unquantized scan (400 samples) to quantized scan (360 samples)
// void process(pp* input, int size, float* single_scan_data_proc){
//     float minAngleDiff = __INT_MAX__;
//     float minDist = __INT_MAX__;
//     int min_idx = 0;

//     int i = 0;
//     int s_idx = size - PROCESS_SEARCH_RANGE;
//     while (i < PROCESS_SEARCH_RANGE * 2) {
//         int idx = (s_idx + i) % size;
//         float diff = (input[idx].angle > 180) ? (360 - input[idx].angle) : input[idx].angle;
//         if (diff < minAngleDiff) {
//             minAngleDiff = diff;
//             minDist = input[idx].distance;
//             min_idx = idx;
//         }
//         i++;
//     }
//     single_scan_data_proc[0] = minDist;

//     for (int i = 1; i < FINAL_NUM_POINTS; i++) {
//         minAngleDiff = __INT_MAX__;
//         minDist = __INT_MAX__;
        
//         int j = 0;
//         s_idx = min_idx - 1;
//         while (j < PROCESS_SEARCH_RANGE) {
//             int idx = (s_idx + j) % size;
//             int diff = fabs(input[idx].angle - i);
//             if (diff < minAngleDiff) {
//                 minAngleDiff = diff;
//                 minDist = input[idx].distance;
//                 min_idx = idx;
//             }
//             j++;
//         }
//         single_scan_data_proc[i] = minDist;
//     }
//     return;
// }

// Quantizes a scan from 400 raw data samples ----> 360 samples that are 0-degree-based
float* quantizeScan(rplidar_response_measurement_node_t* scanSamples, int numSamples){
    pp* single_scan_data = reformatSamples(scanSamples, numSamples);
    qsort(single_scan_data, numSamples, sizeof(pp), comp);
    float* single_scan_data_proc = (float*) malloc(sizeof(float) * FINAL_NUM_POINTS);
    process(single_scan_data, numSamples, single_scan_data_proc);
    // Free single_scan_data and node samples in buffer
    // for(int i = 0; i < numSamples; i++) {
    //     //free(&(single_scan_data[i]));

    //     // Don't do this, or else it will free the buffer where scans are read from
    //     //free(&(scanSamples[i]));
    // }
    return single_scan_data_proc;
}
//

float* getLiDARScan() {
    int numSamples = NUM_SAMPLES;
    startScan(false, RPLIDAR_DEFAULT_TIMEOUT*2);
    while(!(IS_OK(grabData(RPLIDAR_DEFAULT_TIMEOUT, buff))));
    stop();

    reformatScan();
    qsort(single_scan_reformed, numSamples, sizeof(pp), comp);
    float* single_scan_data_proc = (float*) malloc(sizeof(float) * FINAL_NUM_POINTS);
    process(single_scan_reformed, numSamples, single_scan_data_proc);
    
    return single_scan_data_proc;
}

void updateLiDARScan(float* prevScan) {
    int numSamples = NUM_SAMPLES;
    startScan(false, RPLIDAR_DEFAULT_TIMEOUT*2);
    while(!(IS_OK(grabData(RPLIDAR_DEFAULT_TIMEOUT, buff))));
    stop();

    reformatScan();
    qsort(single_scan_reformed, numSamples, sizeof(pp), comp);
    float* single_scan_data_proc = prevScan;
    process(single_scan_reformed, numSamples, single_scan_data_proc);
}



// Determine current location by comparing current quantized scan with the quantized initialization scans
Point getCurrLoc() {    
    // get lidar scan data (unprocessed)
    startScan(false, RPLIDAR_DEFAULT_TIMEOUT*2);
    while(!(IS_OK(grabData(RPLIDAR_DEFAULT_TIMEOUT, buff))));
    stop();
    // Quantize current scan (processing)
    float* procScanSamp = quantizeScan(node, NUM_SAMPLES);

    // // replace them with getLiDARScan();
    // float* procScanSamp = getLiDARScan();

    // Stats for initialization scan best match
    int minDiff = INT_MAX;
    int final_angle = 0;
    int minAngle = 0;

    char bestMatchID = 0;
    
    // For each square in floor plan
    for(int initScanID = 0; initScanID < SQUARES_IN_MAP; initScanID++) {
        // Skip invalid squares in floor plan
        if(fplan[initScanID / NCOLS][initScanID % NCOLS] == 1) {
            // printf("Skipping Obstacle Square (%d, %d)\n", initScanID / NCOLS, initScanID % NCOLS);
            fflush(stdout);
            continue;
        }
        
        int minDiffForCurrInitScan = INT_MAX;
        minAngle = 0;

        // Compare current quantized scan with quantized initialization scan
        // Outer for loop: Accounts for bot not facing directly north (simulates angle alignment by altering the current scan's 0-degree-index)
        //           TODO: modify the zeroDegIdx to be a window based on compass reading
        for(int zeroDegIdx = 0; zeroDegIdx < FINAL_NUM_POINTS; zeroDegIdx++) {      
            int diff = 0;
            // Inner for loop: Compare 360 samples in angle-aligned current scan with those in initialization scan
            for(int i = 0; i < FINAL_NUM_POINTS; i++) {
                diff += pow(abs(procScanSamp[(zeroDegIdx + i) % FINAL_NUM_POINTS] - lidar_data[initScanID / NCOLS][initScanID % NCOLS][i]), 1);
            }
            if(diff < minDiffForCurrInitScan) {
                minDiffForCurrInitScan = diff;
                minAngle = zeroDegIdx;
            }
        }
        if(minDiffForCurrInitScan < minDiff) {
            minDiff = minDiffForCurrInitScan;
            bestMatchID = initScanID;
            final_angle = 360 - minAngle;
            
        }
        // printf("\nDiff for (%d, %d): %d\n", initScanID / NCOLS, initScanID % NCOLS, minDiffForCurrInitScan);

    }
    // free the procScan, sicne we don't need it anymore
    free(procScanSamp);

    char angle[4];
    snprintf(angle, 4, "%d", final_angle);
    prints(angle);
    fflush(stdout);
    
    Point ret;
    ret.x = (int) bestMatchID / NROWS;
    ret.y = (int) bestMatchID % NCOLS;
    return ret;
}

float getDistHelper(float * proScanSamp, int refAngle) {
    float sum = 0;
    int dev = 0, multiplier = 1;
    float distance[FINAL_NUM_POINTS / 2] = {0};

    if (90 < refAngle && refAngle < 270) {
        multiplier = -1;
    }

    float refDist = proScanSamp[refAngle];
    refDist = (refDist == 0) ? 900 : refDist;
    int range = (atan2(FRONT_SCAN_WIDTH / 2, refDist) * 180 / M_PI) + 0.5;
    // printf("refDist = %f, range = %d\n", refDist, range);

    int j = 0, i = (FINAL_NUM_POINTS - (range/2) + refAngle) % FINAL_NUM_POINTS;
    int angle = i;
    float minDist = __FLT_MAX__, curr = 0;
    while (j < range) {
        angle = (i + j) % 360;
        if (proScanSamp[angle] > 10) {
            curr = multiplier * cos(angle / 180.0 * M_PI) * proScanSamp[angle];
            distance[j] = curr;
            if (curr < minDist) {
                minDist = curr;
            }
        } else {
            distance[j] = 0;
        }
        j++;
    }

    j = 0;
    angle = i;
    while (j < range) {
        if (fabs(distance[j] - minDist) < 15) {
            sum += distance[j];
            dev++;
        }
        j++;
    }

    // int j = 0;
    // int i = (FINAL_NUM_POINTS - (range/2) + refAngle) % FINAL_NUM_POINTS;
    // int angle = i;
    // while (j < range) {
    //     angle = (i + j) % 360;
    //     sum += cos(angle / 180.0 * M_PI) * proScanSamp[angle];
    //     dev++;
    //     j++;
    // }
    return round(sum / dev);
}

#define CONVERGENCE 15 //mm
#define NUM_CONV 2
void getFrontBackDist(float * frontFinal, float *backFinal) {
    // int convergence = NUM_CONV;
    int conv_front = NUM_CONV, conv_back = NUM_CONV;
    float frontDev = 0, backDev = 0;
    float origFront, front, newFront;
    float origBack, back, newBack;

    float * lidarScan = getLiDARScan();
    origFront = front = getDistHelper(lidarScan, 0);
    origBack = back = getDistHelper(lidarScan, 180);
    updateLiDARScan(lidarScan);
    newFront = getDistHelper(lidarScan, 0);
    newBack = getDistHelper(lidarScan, 180);

    int toggle = 1;

    // printf("OrigFront: %f ?= newFront: %f\n", origFront, newFront);
    // printf("OrigBack: %f ?= newBack: %f\n", origBack, newBack);
    while (conv_front && conv_back) {
        frontDev += toggle * (origFront - newFront);
        backDev += toggle * (origBack - newBack);
        if (frontDev < CONVERGENCE) conv_front--;
        else {
            conv_front = NUM_CONV;
            origFront = newFront;
            frontDev = 0;
        }
        if (backDev < CONVERGENCE) conv_back--;
        else {
            conv_back = NUM_CONV;
            origBack = newBack;
            backDev = 0;
        }
        front = newFront;
        back = newBack;
        updateLiDARScan(lidarScan);
        newFront = getDistHelper(lidarScan, 0);
        newBack = getDistHelper(lidarScan, 180);
        // printf("OrigFront: %f ?= newFront: %f\n", origFront, newFront);
        // printf("OrigBack: %f ?= newBack: %f\n", origBack, newBack);
        
        toggle = -1*toggle;
    }

    int one_more_chance = 2;
    if (conv_back) {
        while (one_more_chance--) {
            toggle = -1*toggle;
            backDev += toggle * (origBack - newBack);
            if (backDev < CONVERGENCE) conv_back--;
            else {
                conv_back = NUM_CONV;
                origBack = newBack;
                backDev = 0;
            }
            back = newBack;
            updateLiDARScan(lidarScan);
            newBack = getDistHelper(lidarScan, 180);
            // printf("OrigBack: %f ?= newBack: %f\n", origBack, newBack);
        }
    } else if (conv_front) {
        while (one_more_chance--) {
            toggle = -1*toggle;
            frontDev += toggle * (origFront - newFront);
            if (frontDev < CONVERGENCE) conv_front--;
            else {
                conv_front = NUM_CONV;
                origFront = newFront;
                frontDev = 0;
            }
            front = newFront;
            updateLiDARScan(lidarScan);
            newFront = getDistHelper(lidarScan, 0);
            // printf("OrigFront: %f ?= newFront: %f\n", origFront, newFront);
        }
    }

    if (conv_front <= 0) *frontFinal = origFront;
    else *frontFinal = -1;
    if (conv_back <= 0) *backFinal = origBack;
    else *backFinal = -1;

    // while (convergence) {


    //     if ((origFront - newFront) < CONVERGENCE && (origBack - newBack) < CONVERGENCE) {
    //         convergence--;
    //     } else {
    //         convergence = NUM_CONV;
    //         origFront = newFront;
    //         origBack = newBack;
    //     }


    //     front = newFront;
    //     back = newBack;
    //     updateLiDARScan(lidarScan);
    //     newFront = getDistHelper(lidarScan, 0);
    //     newBack = getDistHelper(lidarScan, 180);
    // }

    // *frontFinal = origFront;
    // *backFinal = origBack;
}

float getFrontDist(float* procScanSamp) {
    // return procScanSamp[0];

    return getDistHelper(procScanSamp, 0);
}

float getBackDist(float* procScanSamp) {
    // return procScanSamp[FINAL_NUM_POINTS/2];

    return getDistHelper(procScanSamp, 180);
}

bool doINeedToStop(float* procScanSamp, float prevFront, float prevBack, int cm, bool* obstacleFlag, bool* distanceDoneFlag) {
    // float newFront = getFrontDist(procScanSamp);
    // float newBack = getBackDist(procScanSamp);
    // char lo[100];

    // float deltaFront = fabs(prevFront - newFront);
    // float deltaBack = fabs(prevBack - newBack);

    // snprintf(lo, 100, "deltas_%f_%f", deltaFront, deltaBack);
    // prints(lo);

    // if (prevFront > prevBack) {
    //     if (fabs(prevFront - newFront) >= cm * 10) {
    //         *distanceDoneFlag = true;
    //         // snprintf(lo, 100, "1_abs_diff_%f", fabs(prevFront - newFront));
    //         // prints(lo);
    //         return;
    //     }
    // } else {
    //     if (fabs(prevBack - newBack) >= cm * 10) {
    //         *distanceDoneFlag = true;
    //         // snprintf(lo, 100, "2_abs_diff_%f", fabs(prevFront - newFront));
    //         // prints(lo);
    //         return;
    //     }
    // }

    // not optimized version
    // for(int i = 0; i < FINAL_NUM_POINTS; i++) {
    //     if ((procScanSamp[i] - object_limit[i]) <= 0) {
    //         &obstacleFlag = true;
    //         return;
    //     }
    // }

    // OPTIMIZED VERSION (using Object_limit[OBJECT_RANGE])
    int count = 0;
    int shift = FINAL_NUM_POINTS - END_ANGLE_R;
    for(int i = 0; i <= END_ANGLE_R; i++) {
        if ((procScanSamp[i] > 10) && (procScanSamp[i] - object_limit_r[i]) < 0) {
            count++;
            if (count > 10) {
                return true;
            }      
        }
    }
    for(int i = 0; i < END_ANGLE_R; i++) {
        if ((procScanSamp[i] > 10) && (procScanSamp[i + shift] - object_limit_r[i + END_ANGLE_R + 1]) < 0) {
            count++;
            if (count > 10) {
                return true;
            }
        }
    }
    
    return false;
}

bool isThereObstacle_s(float* procScanSamp, int tolerance) {
    int count = 0;
    // int shift = FINAL_NUM_POINTS - END_ANGLE;
    // for(int i = 0; i <= END_ANGLE; i++) {
    //     if ((procScanSamp[i] > 10) && (procScanSamp[i] - object_limit[i]) < tolerance) {
    //         count++;
    //         printf("At Angle = %d, distance is %f", i, procScanSamp[i]);
    //         if (count > 3) {
    //             return true;
    //         }      
    //     }
    // }
    // for(int i = 0; i < END_ANGLE; i++) {
    //     if ((procScanSamp[i + shift] > 10) && (procScanSamp[i + shift] - object_limit[i + END_ANGLE + 1]) < tolerance) {
    //         count++;
    //         printf("At Angle = %d, distance is %f", i, procScanSamp[i]);
    //         if (count > 3) {
    //             return true;
    //         }
    //     }
    // }

    int start_idx = FINAL_NUM_POINTS - END_ANGLE_S;
    int idx = start_idx;
    int j = 0;
    int prev_j = -2;
    while (j <= OBJECT_RANGE_S) {
        if ((procScanSamp[idx] > 10) && (procScanSamp[idx] - object_limit_s[j]) < tolerance) {
            if ((prev_j - j + 1) < 5) count++;
            else count = 0;
            prev_j = j;
            printf("At Angle = %d, distance is %f\n", idx, procScanSamp[idx]);
            if (count > 2) {
                return true;
            }      
        }
        j++;
        idx = (start_idx + j) % 360;
    }

    return false;
}

bool isThereObstacle_r(float* procScanSamp, int tolerance) {
    int count = 0;

    int start_idx = FINAL_NUM_POINTS - END_ANGLE_R;
    int idx = start_idx;
    int j = 0;
    int prev_j = -2;
    while (j <= OBJECT_RANGE_R) {
        if ((procScanSamp[idx] > 10) && (procScanSamp[idx] - object_limit_r[j]) < tolerance) {
            if ((prev_j - j + 1) < 5) count++;
            else count = 0;
            prev_j = j;
            printf("At Angle = %d, distance is %f\n", idx, procScanSamp[idx]);
            if (count > 2) {
                return true;
            }      
        }
        j++;
        idx = (start_idx + j) % 360;
    }

    return false;
}



int angleDiff(float* prevScan, float* newScan) {
    int minDiffForCurrInitScan = INT_MAX;
    int minAngle = 0;
    for(int zeroDegIdx = 0; zeroDegIdx < FINAL_NUM_POINTS; zeroDegIdx++) {      
        int diff = 0;
        // Inner for loop: Compare 360 samples in angle-aligned current scan with those in initialization scan
        for(int i = 0; i < FINAL_NUM_POINTS; i++) {
            diff += pow(abs(prevScan[(zeroDegIdx + i) % FINAL_NUM_POINTS] - newScan[i]), 1);
        }
        if(diff < minDiffForCurrInitScan) {
            minDiffForCurrInitScan = diff;
            minAngle = zeroDegIdx;
        }
    }
    if (180 < minAngle) {
        minAngle = minAngle - 360;
    }

    return minAngle;
}

int absoluteErrorFrom(float * procScanSamp, Point p, int * angle) {
    int minDiffForCurrInitScan = INT_MAX;
    int final_angle = 0;
    int minAngle = 0;
    for(int zeroDegIdx = 0; zeroDegIdx < FINAL_NUM_POINTS; zeroDegIdx++) {      
        int diff = 0;
        // Inner for loop: Compare 360 samples in angle-aligned current scan with those in initialization scan
        for(int i = 0; i < FINAL_NUM_POINTS; i++) {
            diff += pow(abs(procScanSamp[(zeroDegIdx + i) % FINAL_NUM_POINTS] - lidar_data[p.x][p.y][i]), 1);
        }
        if(diff < minDiffForCurrInitScan) {
            minDiffForCurrInitScan = diff;
            minAngle = zeroDegIdx;
        }
    }
    final_angle = 360 - minAngle;
    *angle = final_angle;

    return minDiffForCurrInitScan;
}


int float_comparator(const void *a, const void *b)
{
    float aa = *(float*)a, bb = *(float*)b;
   
    if (aa > bb) return -1;
    if (aa < bb) return 1;
    return 0;
}


float getVerticalOffset(float * procScanSamp, Point p, int zeroDegIdx, int front) {
    float deltaDist[FINAL_NUM_POINTS / 2] = {0};
    float sum = 0;
    int dev = 0;
    float refDist;
    if (front) refDist = procScanSamp[(0 + zeroDegIdx) % FINAL_NUM_POINTS];
    else refDist = procScanSamp[(180 + zeroDegIdx) % FINAL_NUM_POINTS];
    
    int range = (int) ((atan2(ROBOT_WIDTH / 2, refDist) * 180 / M_PI) + 0.5);

    int angle_start;
    if (front) angle_start = 360 - (range) / 2;
    else angle_start = 180 - (range) / 2;

    int j = 0;
    while (j < range) {
        float coeff = cos(((angle_start + j) % FINAL_NUM_POINTS) / 180.0 * M_PI);
        deltaDist[j] = coeff * lidar_data[p.x][p.y][(angle_start + j) % FINAL_NUM_POINTS];
        deltaDist[j] -= coeff * procScanSamp[(zeroDegIdx + angle_start + j) % FINAL_NUM_POINTS];
        j++;
    }
    qsort(deltaDist, range, sizeof(float), float_comparator);

    // filter outliers (prob by obsticles)
    int minimun = deltaDist[0];
    for (j = 0; j < range; j++) {
        if (abs(deltaDist[j] - minimun) <= DEV_THRESHOLD) {
            sum += deltaDist[j];
            dev++;
        }
    }

    return roundf(sum / dev);
}

float getHorizontalOffset(float * procScanSamp, Point p, int zeroDegIdx, int right) {
    float deltaDist[FINAL_NUM_POINTS / 2] = {0};
    float sum = 0;
    int dev = 0;
    float refDist;
    if (right) refDist = procScanSamp[(90 + zeroDegIdx) % FINAL_NUM_POINTS];
    else refDist = procScanSamp[(270 + zeroDegIdx) % FINAL_NUM_POINTS];
    
    int range = (int) ((atan2(ROBOT_LENGTH / 2, refDist) * 180 / M_PI) + 0.5);

    int angle_start;
    if (right) angle_start = 90 - (range) / 2;
    else angle_start = 270 - (range) / 2;
    
    int j = 0;
    while (j < range) {
        float coeff = sin(((angle_start + j) % FINAL_NUM_POINTS) / 180.0 * M_PI);
        deltaDist[j] = coeff * lidar_data[p.x][p.y][(angle_start + j) % FINAL_NUM_POINTS];
        deltaDist[j] -= coeff * procScanSamp[(zeroDegIdx + angle_start + j) % FINAL_NUM_POINTS];
        j++;
    }
    qsort(deltaDist, range, sizeof(float), float_comparator);

    // filter outliers (prob by obsticles)
    int minimun = deltaDist[0];
    for (j = 0; j < range; j++) {
        if (abs(deltaDist[j] - minimun) <= DEV_THRESHOLD) {
            sum += deltaDist[j];
            dev++;
        }
    }

    return roundf(sum / dev);
}

// Assume we are facing North, right (east) is +X for horizontal, top (north) is +Y for verticle
// offset is not in perspective of rover, but it is absolute offset on the plane
void getOffSetFrom(float * procScanSamp, Point p, int * offsetX, int * offsetY) {

    int zeroDegIdx = 0;
    absoluteErrorFrom(procScanSamp, p, &zeroDegIdx);
    zeroDegIdx = 360 - zeroDegIdx;

    float deltaFront = getVerticalOffset(procScanSamp, p, zeroDegIdx, 1);
    float deltaBack = getVerticalOffset(procScanSamp, p, zeroDegIdx, 0);

    float deltaRight = getHorizontalOffset(procScanSamp, p, zeroDegIdx, 1);
    float deltaLeft = getHorizontalOffset(procScanSamp, p, zeroDegIdx, 0);

    if (abs(deltaFront - deltaBack) > DEV_THRESHOLD) {
        *offsetY = (abs(deltaFront) < abs(deltaBack))? deltaFront : deltaBack;
    } else {
        float delta1 = procScanSamp[(0 + zeroDegIdx) % FINAL_NUM_POINTS];
        float delta2 = procScanSamp[(180 + zeroDegIdx) % FINAL_NUM_POINTS];
        *offsetY = (delta1 > delta2)? deltaFront : deltaBack;
    }

    if (abs(deltaRight - deltaLeft) > DEV_THRESHOLD) {
        *offsetX = (abs(deltaRight) < abs(deltaLeft))? deltaRight : deltaLeft;
    } else {
        float delta1 = procScanSamp[(90 + zeroDegIdx) % FINAL_NUM_POINTS];
        float delta2 = procScanSamp[(280 + zeroDegIdx) % FINAL_NUM_POINTS];
        *offsetX = (delta1 > delta2)? deltaRight : deltaLeft;
    }

    // float delta1 = procScanSamp[(0 + zeroDegIdx) % FINAL_NUM_POINTS];
    // float delta2 = procScanSamp[(180 + zeroDegIdx) % FINAL_NUM_POINTS];
    // *offsetY = getVerticalOffset(procScanSamp, p, zeroDegIdx, (int) delta1 > delta2);

    // delta1 = procScanSamp[(90 + zeroDegIdx) % FINAL_NUM_POINTS];
    // delta2 = procScanSamp[(270 + zeroDegIdx) % FINAL_NUM_POINTS];
    // *offsetX = getHorizontalOffset(procScanSamp, p, zeroDegIdx, (int) delta1 > delta2);
}


// Determine current location by comparing current quantized scan with the quantized initialization scans
Point get_curr_loc(int * angleReturn, Point * secondClose) {    
    float* procScanSamp = getLiDARScan();

    // Stats for initialization scan best match
    int minDiff = INT_MAX;
    int prevMinDiff = INT_MAX;
    int final_angle = 0;
    int minAngle = 0;

    char bestMatchID = 0;
    char preBestMatchID = 0;
    
    // For each square in floor plan
    for(int initScanID = 0; initScanID < SQUARES_IN_MAP; initScanID++) {
        // Skip invalid squares in floor plan
        if(fplan[initScanID / NCOLS][initScanID % NCOLS] == 1) {
            // printf("Skipping Obstacle Square (%d, %d)\n", initScanID / NCOLS, initScanID % NCOLS);
            fflush(stdout);
            continue;
        }
        
        int minDiffForCurrInitScan = INT_MAX;
        minAngle = 0;

        // Compare current quantized scan with quantized initialization scan
        // Outer for loop: Accounts for bot not facing directly north (simulates angle alignment by altering the current scan's 0-degree-index)
        //           TODO: modify the zeroDegIdx to be a window based on compass reading
        for(int zeroDegIdx = 0; zeroDegIdx < FINAL_NUM_POINTS; zeroDegIdx++) {      
            int diff = 0;
            // Inner for loop: Compare 360 samples in angle-aligned current scan with those in initialization scan
            for(int i = 0; i < FINAL_NUM_POINTS; i++) {
                diff += pow(abs(procScanSamp[(zeroDegIdx + i) % FINAL_NUM_POINTS] - lidar_data[initScanID / NCOLS][initScanID % NCOLS][i]), 1);
            }
            if(diff < minDiffForCurrInitScan) {
                minDiffForCurrInitScan = diff;
                minAngle = zeroDegIdx;
            }
        }
        if(minDiffForCurrInitScan < minDiff) {
            prevMinDiff = minDiff;
            minDiff = minDiffForCurrInitScan;
            preBestMatchID = bestMatchID;
            bestMatchID = initScanID;
            final_angle = 360 - minAngle;    
        }
        // printf("\nDiff for (%d, %d): %d\n", initScanID / NCOLS, initScanID % NCOLS, minDiffForCurrInitScan);

    }
    // free the procScan, sicne we don't need it anymore
    free(procScanSamp);

    char angle[4];
    snprintf(angle, 4, "%d", final_angle);
    prints(angle);
    fflush(stdout);
    
    Point ret;
    ret.x = (int) bestMatchID / NROWS;
    ret.y = (int) bestMatchID % NCOLS;
    *angleReturn = final_angle;

    secondClose->x = (int) preBestMatchID / NROWS;
    secondClose->y = (int) preBestMatchID % NCOLS;

    if (prevMinDiff - minDiff >= SIMILAR_THRESH) {
        secondClose->x = -1;
        secondClose->y = -1;
    }

    return ret;
}



Point get_curr_loc_input(enum compass heading, int* angleReturn, Point * secondClose) {    
    float* procScanSamp = getLiDARScan();

    // Stats for initialization scan best match
    int minDiff = INT_MAX;
    int prevMinDiff = INT_MAX;
    int final_angle = 0;
    int minAngle = 0;

    char bestMatchID = 0;
    char preBestMatchID = 0;
    
    // For each square in floor plan
    for(int initScanID = 0; initScanID < SQUARES_IN_MAP; initScanID++) {
        // Skip invalid squares in floor plan
        if(fplan[initScanID / NCOLS][initScanID % NCOLS] == 1) {
            // printf("Skipping Obstacle Square (%d, %d)\n", initScanID / NCOLS, initScanID % NCOLS);
            fflush(stdout);
            continue;
        }
        
        int minDiffForCurrInitScan = INT_MAX;
        minAngle = 0;

        // Compare current quantized scan with quantized initialization scan
        // Outer for loop: Accounts for bot not facing directly north (simulates angle alignment by altering the current scan's 0-degree-index)
        //           TODO: modify the zeroDegIdx to be a window based on compass reading
        int zeroDegIdx = 0;
        if (heading == NORTH) {
            zeroDegIdx = 360 - (SEARCH_DEGREE_RANGE / 2);
        } else if (heading == WEST) {
            zeroDegIdx = 90 - (SEARCH_DEGREE_RANGE / 2);
        } else if (heading == SOUTH) {
            zeroDegIdx = 180 - (SEARCH_DEGREE_RANGE / 2);
        } else if (heading == EAST) {
            zeroDegIdx = 270 - (SEARCH_DEGREE_RANGE / 2);
        }

        int j = 0;
        while(j < SEARCH_DEGREE_RANGE) {     
            int diff = 0;
            // Inner for loop: Compare 360 samples in angle-aligned current scan with those in initialization scan
            for(int i = 0; i < FINAL_NUM_POINTS; i++) {
                diff += pow(abs(procScanSamp[(zeroDegIdx + j + i) % FINAL_NUM_POINTS] - lidar_data[initScanID / NCOLS][initScanID % NCOLS][i]), 1);
            }
            if(diff < minDiffForCurrInitScan) {
                minDiffForCurrInitScan = diff;
                minAngle = zeroDegIdx + j;
            }
            j++;
        }
        if(minDiffForCurrInitScan < minDiff) {
            prevMinDiff = minDiff;
            minDiff = minDiffForCurrInitScan;
            preBestMatchID = bestMatchID;
            bestMatchID = initScanID;
            final_angle = 360 - minAngle;
        }
        // printf("\nDiff for (%d, %d): %d\n", initScanID / NCOLS, initScanID % NCOLS, minDiffForCurrInitScan);

    }
    // free the procScan, sicne we don't need it anymore
    free(procScanSamp);

    char angle[4];
    snprintf(angle, 4, "%d", final_angle);
    prints(angle);
    fflush(stdout);
    
    Point ret;
    ret.x = (int) bestMatchID / NROWS;
    ret.y = (int) bestMatchID % NCOLS;
    secondClose->x = (int) preBestMatchID / NROWS;
    secondClose->y = (int) preBestMatchID % NCOLS;
    *angleReturn = final_angle;

    if (prevMinDiff - minDiff >= SIMILAR_THRESH) {
        secondClose->x = -1;
        secondClose->y = -1;
    }

    return ret;
}


