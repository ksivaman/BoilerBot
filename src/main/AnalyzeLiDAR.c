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
#include "include/navigation.h"

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
    // int final_angle = 0;
    // int minAngle = 0;

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
        // minAngle = 0;

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
                // minAngle = zeroDegIdx;
            }
        }
        if(minDiffForCurrInitScan < minDiff) {
            minDiff = minDiffForCurrInitScan;
            bestMatchID = initScanID;
            // final_angle = 360 - minAngle;
            
        }
        // printf("\nDiff for (%d, %d): %d\n", initScanID / NCOLS, initScanID % NCOLS, minDiffForCurrInitScan);

    }
    // free the procScan, sicne we don't need it anymore
    free(procScanSamp);

    // char angle[4];
    // snprintf(angle, 4, "%d", final_angle);
    // prints(angle);
    // fflush(stdout);
    
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
    float maxDist = 0;
    while (j < range) {
        angle = (i + j) % 360;
        if (10 < proScanSamp[angle] && proScanSamp[angle] < 10000.0) {
            curr = multiplier * cos(angle / 180.0 * M_PI) * proScanSamp[angle];
            distance[j] = curr;
            if (fabs(curr) < fabs(minDist)) {
                minDist = curr;
            }
            if (fabs(maxDist) < fabs(curr)){
                maxDist = curr;
            }
        } else {
            distance[j] = 0;
        }
        j++;
    }

    j = 0;
    angle = i;
    while (j < range) {
        if (fabs(distance[j] - maxDist) < 20) {
            sum += distance[j];
            dev++;
        }
        j++;
    }

    return round(sum / dev);
}

#define CONVERGENCE 15 //mm
#define NUM_CONV 2 // 2
void getFrontBackDist(float * frontFinal, float *backFinal) {
    // int convergence = NUM_CONV;
    int conv_front = NUM_CONV, conv_back = NUM_CONV;
    float frontDev = 0, backDev = 0;
    float origFront, newFront;
    float origBack, newBack;

    float * lidarScan = getLiDARScan();
    origFront = getDistHelper(lidarScan, 0);
    origBack = getDistHelper(lidarScan, 180);
    

    int toggle = 1;

    // printf("OrigFront: %f ?= newFront: %f\n", origFront, newFront);
    // printf("OrigBack: %f ?= newBack: %f\n", origBack, newBack);

    do {
        updateLiDARScan(lidarScan);
        newFront = getDistHelper(lidarScan, 0);
        newBack = getDistHelper(lidarScan, 180);
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
        // printf("OrigFront: %f ?= newFront: %f\n", origFront, newFront);
        // printf("OrigBack: %f ?= newBack: %f\n", origBack, newBack);
        toggle = -1*toggle;
    } while (conv_front && conv_back);
    
    int one_more_chance = 1;
    if (conv_back) {
        do {
            updateLiDARScan(lidarScan);
            newBack = getDistHelper(lidarScan, 180);
            toggle = -1*toggle;
            backDev += toggle * (origBack - newBack);
            if (backDev < CONVERGENCE) conv_back--;
            else {
                conv_back = NUM_CONV;
                origBack = newBack;
                backDev = 0;
            }
            // printf("OrigBack: %f ?= newBack: %f\n", origBack, newBack);
        } while (--one_more_chance && conv_back); 
    } else if (conv_front) {
        do {
            updateLiDARScan(lidarScan);
            newFront = getDistHelper(lidarScan, 0);
            toggle = -1*toggle;
            frontDev += toggle * (origFront - newFront);
            if (frontDev < CONVERGENCE) conv_front--;
            else {
                conv_front = NUM_CONV;
                origFront = newFront;
                frontDev = 0;
            }
            // printf("OrigFront: %f ?= newFront: %f\n", origFront, newFront);
        } while (--one_more_chance && conv_front);
    }

    if (conv_front <= 0) *frontFinal = origFront;
    else *frontFinal = -1;
    if (conv_back <= 0) *backFinal = origBack;
    else *backFinal = -1;
}

float getFrontDist(float* procScanSamp) {
    // return procScanSamp[0];

    return getDistHelper(procScanSamp, 0);
}

float getBackDist(float* procScanSamp) {
    // return procScanSamp[FINAL_NUM_POINTS/2];

    return getDistHelper(procScanSamp, 180);
}

bool isThereObstacle_s(float* procScanSamp, int tolerance, enum dir direction) {
    int count = 0;
    int start_idx = FINAL_NUM_POINTS - END_ANGLE_R;
    int offset = (direction == FORWARD) ? 0: 180;
    int idx = (start_idx + offset) % 360;
    int j = 0;
    int prev_j = -2;
    while (j <= OBJECT_RANGE_S) {
        if ((procScanSamp[idx] > 10) && (procScanSamp[idx] - object_limit_s[j]) < tolerance) {
            if ((prev_j - j + 1) < 5) count++;
            else count = 0;
            prev_j = j;
            // printf("At Angle = %d, distance is %f\n", idx, procScanSamp[idx + offset]);
            if (count > 2) {
                return true;
            }      
        }
        j++;
        idx = (start_idx + j + offset) % 360;
    }

    return false;
}

bool isThereObstacle_r(float* procScanSamp, int tolerance, enum dir direction) {
    int count = 0;

    int start_idx = FINAL_NUM_POINTS - END_ANGLE_R;
    int offset = (direction == FORWARD) ? 0: 180;
    int idx = (start_idx + offset) % 360;
    int j = 0;
    int prev_j = -2;
    while (j <= OBJECT_RANGE_R) {
        if ((procScanSamp[idx] > 10) && (procScanSamp[idx] - object_limit_r[j]) < tolerance) {
            if ((prev_j - j + 1) < 5) count++;
            else count = 0;
            prev_j = j;
            // printf("At Angle = %d, distance is %f\n", idx, procScanSamp[idx + offset]);
            if (count > 2) {
                return true;
            }      
        }
        j++;
        idx = (start_idx + j + offset) % 360;
    }

    return false;
}

bool isThereObstacle_a(float* procScanSamp, int offset) {
    int count = 0;

    int start_idx = FINAL_NUM_POINTS - END_ANGLE_R;
    int idx = (start_idx + offset) % 360;
    int j = 0;
    int prev_j = -2;
    while (j <= OBJECT_RANGE_R) {
        if ((procScanSamp[idx] > 10) && (procScanSamp[idx] - object_limit_s[j]) < 0) {
            if ((prev_j - j + 1) < 5) count++;
            else count = 0;
            prev_j = j;
            // printf("At Angle = %d, distance is %f\n", idx, procScanSamp[idx + offset]);
            if (count > 2) {
                return true;
            }      
        }
        j++;
        idx = (start_idx + j + offset) % 360;
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

// mode 0: finding max mode, mode 1: finding min
float offsetHelper(float * lidarScan, int range, int zeroDegIdx, enum compass direction, bool mode) {
    float distance[FINAL_NUM_POINTS / 2] = {0};
    double sum = 0;
    int dev = 0;
    int angle_start = (direction - (range / 2) + FINAL_NUM_POINTS) % FINAL_NUM_POINTS;

    int j = 0;
    float minDist = __FLT_MAX__;
    float maxDist = 0;
    // float max = 0;
    float curr = 0;
    float coeff = 0;
    while (j < range) {
        float diagonal = lidarScan[(angle_start + j + zeroDegIdx) % FINAL_NUM_POINTS];
        if (10 < diagonal && diagonal < 10000.0) {
            if (direction == EAST || direction == WEST) 
                coeff = sin(((angle_start + j) % FINAL_NUM_POINTS) / 180.0 * M_PI);
            else 
                coeff = cos(((angle_start + j) % FINAL_NUM_POINTS) / 180.0 * M_PI);
            curr = coeff * diagonal;
            distance[j] = curr;
            if (fabs(curr) < fabs(minDist)) {
                minDist = curr;
            }
            if (fabs(maxDist) < fabs(curr)){
                maxDist = curr;
            }
        } else {
            distance[j] = 0;
        }
        j++;
    }

    // printf("MaxDist: %f\n", maxDist);

    j = 0;
    if (mode) {
        while (j < range) {
            if (fabs(distance[j] - minDist) < 20) {
                sum += distance[j];
                dev++;
            }
            j++;
        }
    } else {
        while (j < range) {
            if (fabs(distance[j] - maxDist) < 20) {
                // printf("adding %f\n", distance[j]);
                sum += distance[j];
                dev++;
            }
            j++;
        }
    }

    float final = (sum / dev / 10.0);
    // printf("final: %f\n", final);

    return final; // cm
}

#define NUM_CONV_OFFSET 1
void getOffsets(rover robot, int *offsetNorth, int * offsetEast) {
    Point p = robot.currLoc;
    float northRef = lidar_data[p.x][p.y][NORTH];
    float southRef = lidar_data[p.x][p.y][SOUTH];
    float eastRef = lidar_data[p.x][p.y][EAST];
    float westRef = lidar_data[p.x][p.y][WEST];
    
    int n_range = (int) ((atan2(ROBOT_LENGTH / 2, northRef) * 180 / M_PI) + 0.5);
    int s_range = (int) ((atan2(ROBOT_LENGTH / 2, southRef) * 180 / M_PI) + 0.5);
    int e_range = (int) ((atan2(ROBOT_LENGTH / 2, eastRef) * 180 / M_PI) + 0.5);
    int w_range = (int) ((atan2(ROBOT_LENGTH / 2, westRef) * 180 / M_PI) + 0.5);

    float n_orig = offsetHelper(&(lidar_data[p.x][p.y][0]), n_range, 0, NORTH, 0);
    float s_orig = offsetHelper(&(lidar_data[p.x][p.y][0]), s_range, 0, SOUTH, 0);
    float e_orig = offsetHelper(&(lidar_data[p.x][p.y][0]), e_range, 0, EAST, 0);
    float w_orig = offsetHelper(&(lidar_data[p.x][p.y][0]), w_range, 0, WEST, 0);

    // printf("ORIG North: %f, South: %f, East: %f, West: %f\n", n_orig, s_orig, e_orig, w_orig);
    int angleOffset = 0;
    angleOffset = getAngleOffset(robot);

    //////////////////////////////////////////// Measure 4 distance untill they converge
    int conv_n, conv_s, conv_e, conv_w;
    conv_n = conv_s = conv_e = conv_w = NUM_CONV_OFFSET;
    // int nDev = 0, sDev = 0, eDev = 0, wDev = 0;
    float n_0, n_1;
    float s_0, s_1;
    float e_0, e_1;
    float w_0, w_1;

    float * lidarScan = getLiDARScan();
    n_0  = offsetHelper(lidarScan, n_range, FINAL_NUM_POINTS - (robot.heading + angleOffset), NORTH, 0);
    s_0  = offsetHelper(lidarScan, s_range, FINAL_NUM_POINTS - (robot.heading + angleOffset), SOUTH, 0);
    e_0  = offsetHelper(lidarScan, e_range, FINAL_NUM_POINTS - (robot.heading + angleOffset), EAST, 0);
    w_0  = offsetHelper(lidarScan, w_range, FINAL_NUM_POINTS - (robot.heading + angleOffset), WEST, 0);
    // printf("North: %f, South: %f, East: %f, West: %f\n", n_0, s_0, e_0, w_0);
    
    // printf("North: %f, South: %f, East: %f, West: %f\n", n_1, s_1, e_1, w_1);

    int giveup = 6;
    do {
        updateLiDARScan(lidarScan);
        n_1 = offsetHelper(lidarScan, n_range, FINAL_NUM_POINTS - (robot.heading + angleOffset), NORTH, 0);
        s_1 = offsetHelper(lidarScan, s_range, FINAL_NUM_POINTS - (robot.heading + angleOffset), SOUTH, 0);
        e_1 = offsetHelper(lidarScan, e_range, FINAL_NUM_POINTS - (robot.heading + angleOffset), EAST, 0);
        w_1 = offsetHelper(lidarScan, w_range, FINAL_NUM_POINTS - (robot.heading + angleOffset), WEST, 0);
        if (conv_n) {
            if (abs(n_0 - n_1 + 0.5) < 4) conv_n--;
            else { conv_n = NUM_CONV_OFFSET; n_0 = n_1;
            }
        }
        if (conv_s) {
            if (abs(s_0 - s_1 + 0.5) < 4) conv_s--;
            else { conv_s = NUM_CONV_OFFSET; s_0 = s_1;
            }
        }
        if (conv_e) {
            if (abs(e_0 - e_1 + 0.5) < 4) conv_e--;
            else { conv_e = NUM_CONV_OFFSET; e_0 = e_1;
            }
        }
        if (conv_w) {
            if (abs(w_0 - w_1 + 0.5) < 4) conv_w--;
            else { conv_w = NUM_CONV_OFFSET; w_0 = w_1;
            }
        }
        printf("North: %f, South: %f, East: %f, West: %f\n", n_1, s_1, e_1, w_1);
    } while (((robot.heading == NORTH || robot.heading == SOUTH) ? (conv_n || conv_s) : (conv_e || conv_w)) && giveup--);

    free(lidarScan);

    
    ///////////////////////////////////////////////////////////////////////////////////

    int delat_n = (n_orig - n_1) + 0.5;
    int delat_s = (s_orig - s_1) + 0.5;
    int delat_e = (e_orig - e_1) + 0.5;
    int delat_w = (w_orig - w_1) + 0.5;

    if (giveup == 0) {
        if (conv_e == 0) {
            *offsetEast = delat_e;
        }
        if (conv_w == 0) {
            *offsetEast = delat_w;
        }
        if (conv_n == 0) {
            *offsetNorth = delat_n;
        }  
        if (conv_s == 0) {
            *offsetNorth = delat_s;
        }  
    }

    // printf("deltaNorth: %d, deltaSouth: %d, deltaEast: %d, deltaWest: %d\n", delat_n, delat_s, delat_e, delat_w);


    if (abs(delat_e - delat_w) > 3) {
        *offsetEast = (abs(delat_e) < abs(delat_w))? delat_e : delat_w;
    } else {
        *offsetEast = (abs(eastRef) < abs(westRef)) ? delat_e : delat_w;
    }

    if (abs(delat_n - delat_s) > 3) {
        *offsetNorth = (abs(delat_n) < abs(delat_s))? delat_n : delat_s;
    } else {
        *offsetNorth = (abs(northRef) < abs(southRef)) ? delat_n : delat_s;
    }
}

#define INCREMENT_LIMIT 30.0
int angleOffsetHelper(float * lidarScan, int range, enum compass direction) {
    float height[FINAL_NUM_POINTS / 2] = {0};
    float base[FINAL_NUM_POINTS / 2] = {0};
    int angle_start = (direction - (range / 2) + FINAL_NUM_POINTS) % FINAL_NUM_POINTS;

    int j = 0;
    // float minDist = __FLT_MAX__;
    // float max = 0;
    // float curr = 0;
    float coeff_sin = 0;
    float coeff_cos = 0;
    while (j < range) {
        float diagonal = lidarScan[(angle_start + j) % FINAL_NUM_POINTS];
        if (10 < diagonal && diagonal < 10000.0) {
            coeff_sin = sin(((angle_start + j) % FINAL_NUM_POINTS) / 180.0 * M_PI);
            coeff_cos = cos(((angle_start + j) % FINAL_NUM_POINTS) / 180.0 * M_PI);
            if ((direction == NORTH || direction == SOUTH)) {
                height[j] = coeff_cos * diagonal;
                base[j] = coeff_sin * diagonal;
            } else {
                height[j] = coeff_sin * diagonal;
                base[j] = coeff_cos * diagonal;
            }
        } else {
            height[j] = 0;
            base[j] = 0;
        }
        j++;
    }
    
    j = 0;
    int count = 0;
    int maxCount = 0;
    int newStart = 0;
    int maxCountStart = 0;
    float delta = 0;
    float lastDelta = 0;
    // int multiplyer = 1;
    
    while (j < range - 1) {
        delta = height[j] - height[j + 1];
        // printf("height[j] = %f, delta = %f, last delat = %f\n", height[j], delta, lastDelta);
        if (fabs(delta) < 25.0 && (fabs(delta + lastDelta) >= fabs(delta) - 10)) {
            // printf("height[j] = %f, delta = %f, last delat = %f\n", height[j], delta, lastDelta);
            count++;
            lastDelta = height[j] - height[j + 1];
            // printf("count = %d\n", count);
        } else {
            if ((count > maxCount) || ((count == maxCount) && (height[newStart] < height[maxCountStart]))) {
                maxCount = count;
                maxCountStart = newStart;  
            } 
            newStart = j + 1;
            count = 0;
        }
        j++;
    }
    if ((count > maxCount) || ((count == maxCount) && (height[newStart] < height[maxCountStart]))) {
        maxCount = count;
        maxCountStart = newStart;  
    } 

    j = 0;
    // while (j < range) {
    //     printf("Height: %f, Base: %f\n", height[j], base[j]);
    //     j++;
    // }

    // j = maxCountStart;
    // while (j < maxCountStart + maxCount + 1) {
    //     printf("MAX Height: %f, Base: %f\n", height[j], base[j]);
    //     j++;
    // }

    float delatHeight = fabs(height[maxCountStart] - height[maxCountStart + maxCount]);
    float delatBase = fabs(base[maxCountStart] - base[maxCountStart + maxCount]);
    

    int angleOffset = abs((atan2(delatHeight, delatBase) * 180 / M_PI) + 0.5);
    // if (direction == NORTH || direction == SOUTH) {
    //     angleOffset = 90 - angleOffset;
    // }
    if (abs(height[maxCountStart]) > abs(height[maxCountStart + maxCount])) {
        angleOffset = -angleOffset;
    }

    return angleOffset;
}

int getAngleOffset(rover robot) {
    Point p = robot.currLoc;
    float * preScan = &(lidar_data[p.x][p.y][0]);
    float ref_f = preScan[NORTH];
    float ref_b = preScan[SOUTH];
    float ref_r = preScan[EAST];
    float ref_l = preScan[WEST];

    int f_range = (int) ((atan2(ROBOT_LENGTH / 2, ref_f) * 180 / M_PI) + 0.5);
    int b_range = (int) ((atan2(ROBOT_LENGTH / 2, ref_b) * 180 / M_PI) + 0.5);
    int r_range = (int) ((atan2(ROBOT_LENGTH / 2, ref_r) * 180 / M_PI) + 0.5);
    int l_range = (int) ((atan2(ROBOT_LENGTH / 2, ref_l) * 180 / M_PI) + 0.5);

    float f_min = fabs(offsetHelper(preScan, f_range, 0, NORTH, 1));
    float b_min = fabs(offsetHelper(preScan, b_range, 0, SOUTH, 1));
    float r_min = fabs(offsetHelper(preScan, r_range, 0, EAST, 1));
    float l_min = fabs(offsetHelper(preScan, l_range, 0, WEST, 1));

    // printf("fmin: %f, bmin: %f, rmin: %f, lmin: %f\n", f_min, b_min, r_min, l_min);

    // float ref = ref_f;
    float minimum = fabs(f_min);
    enum compass direction = NORTH;
    if (b_min < minimum) {
        // ref = ref_b;
        minimum = b_min;
        direction = SOUTH;
    }
    if (r_min < minimum) {
        // ref = ref_r;
        minimum = r_min;
        direction = EAST;
    }
    if (l_min < minimum) {
        // ref = ref_l;
        minimum = l_min;
        direction = WEST;
    }

    // get minimum of f b r l
    direction = (direction + 360 - robot.heading) % 360;
    printf("Looking at Direction: %d\n", direction);
    // int range = (int) ((atan2(ROBOT_LENGTH / 2, ref) * 180 / M_PI) + 0.5);
    float * lidarScan = getLiDARScan();
    int angleOffset = angleOffsetHelper(lidarScan, 45, direction);
    free(lidarScan);
    return angleOffset;
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
        

    }
    printf("\nDiff for (%d, %d): %d\n", bestMatchID / NCOLS, bestMatchID % NCOLS, minDiff);
    // printf("Diff for (%d, %d): %d\n", preBestMatchID / NCOLS, preBestMatchID % NCOLS, prevMinDiff);
    // printf("Diff == %d\n\n", prevMinDiff - minDiff);
    // free the procScan, sicne we don't need it anymore
    free(procScanSamp);
    
    Point ret;
    ret.x = (int) bestMatchID / NROWS;
    ret.y = (int) bestMatchID % NCOLS;
    secondClose->x = (int) preBestMatchID / NROWS;
    secondClose->y = (int) preBestMatchID % NCOLS;
    *angleReturn = final_angle;

    if (abs(prevMinDiff - minDiff) >= SIMILAR_THRESH) {
        secondClose->x = -1;
        secondClose->y = -1;
    }

    return ret;
}



Point get_curr_loc_input_2(enum compass heading, int* angleReturn, Point * secondClose) {    
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
            double diff = 0;
            float last_nonZero = 300;
            // Inner for loop: Compare 360 samples in angle-aligned current scan with those in initialization scan
            for(int i = 0; i < FINAL_NUM_POINTS; i++) {
                double temp = pow((procScanSamp[(zeroDegIdx + j + i) % FINAL_NUM_POINTS] - lidar_data[initScanID / NCOLS][initScanID % NCOLS][i]), 2);
                float div = lidar_data[initScanID / NCOLS][initScanID % NCOLS][i];
                if (div == 0) {
                    div = last_nonZero; 
                } else {
                    last_nonZero = div;
                }
                diff += abs(temp / last_nonZero);
                // diff += pow(abs(procScanSamp[(zeroDegIdx + j + i) % FINAL_NUM_POINTS] - lidar_data[initScanID / NCOLS][initScanID % NCOLS][i]), 1);
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
        

    }
    printf("\nDiff for (%d, %d): %d\n", bestMatchID / NCOLS, bestMatchID % NCOLS, minDiff);
    // printf("Diff for (%d, %d): %d\n", preBestMatchID / NCOLS, preBestMatchID % NCOLS, prevMinDiff);
    // printf("Diff == %d\n\n", prevMinDiff - minDiff);
    // free the procScan, sicne we don't need it anymore
    free(procScanSamp);
    
    Point ret;
    ret.x = (int) bestMatchID / NROWS;
    ret.y = (int) bestMatchID % NCOLS;
    secondClose->x = (int) preBestMatchID / NROWS;
    secondClose->y = (int) preBestMatchID % NCOLS;
    *angleReturn = final_angle;

    if (abs(prevMinDiff - minDiff) >= SIMILAR_THRESH) {
        secondClose->x = -1;
        secondClose->y = -1;
    }

    return ret;
}

