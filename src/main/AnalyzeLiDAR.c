#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <math.h>
#include "include/rplidar_cmd.h"
#include "RPLidar_c.c"
#include "include/AnalyzeLiDAR.h"

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
    ledc_fade_func_install(0);
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
    vTaskDelay(9000 / portTICK_PERIOD_MS); 

}

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
Point getCurrLoc() {    
    //get lidar scan data (unprocessed)
    startScan(false, RPLIDAR_DEFAULT_TIMEOUT*2);
    while(!(IS_OK(grabData(RPLIDAR_DEFAULT_TIMEOUT, buff))));
    stop();

    // Quantize current scan (processing)
    float* procScanSamp = quantizeScan(node, NUM_SAMPLES);

    // Stats for initialization scan best match
    int minDiff = INT_MAX;
    char bestMatchID = 0;
    
    // For each square in floor plan
    for(int initScanID = 0; initScanID < SQUARES_IN_MAP; initScanID++) {
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
                diff += pow(abs(procScanSamp[(zeroDegIdx + i) % FINAL_NUM_POINTS] - lidar_data[initScanID / NCOLS][initScanID % NCOLS][i]), 1);
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
    
    Point ret;
    ret.x = (int) bestMatchID / NROWS;
    ret.y = (int) bestMatchID % NCOLS;
    return ret;
}