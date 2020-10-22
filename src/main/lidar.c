#include <string.h>
#include <stdio.h>
#include <stdlib.h>
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

#define FINAL_NUM_POINTS 360
#define ANGLE 0
#define DIST 1

typedef struct _pp {
    float angle;
    float distance;
} pp;

pp * single_scan_data;
float single_scan_data_proc[FINAL_NUM_POINTS];
int total_points = 0;

static int comp (const void *a, const void *b){
    return ((pp *)a)->angle - ((pp *)b)->angle;
}

void processJinText(){
    FILE * fp = fopen("t1.txt", "r");
    float d, a, diff;
    int i = 0;

    while (fscanf(fp, "Distance: %f, Angle: %f --- diff is %f\n", &d, &a, &diff) != EOF){
        total_points++;         
    }

    fseek(fp, 0, SEEK_SET);
    single_scan_data = (pp *)malloc(sizeof(pp) * total_points);
    
    while (fscanf(fp, "Distance: %f, Angle: %f --- diff is %f\n", &d, &a, &diff) != EOF){
        single_scan_data[i].angle = a; 
        single_scan_data[i].distance = d;    
        i++;         
    }

    // total_points = 0;
}

void process(pp * input, int size){
    
    for (int i = 0; i < FINAL_NUM_POINTS; i++) {
        float minAngleDiff = __INT_MAX__;
        float minDist = __INT_MAX__;
        for (int j = 0; j < total_points; j++) {
            if (abs(input[j].angle - i) < minAngleDiff) {
                minAngleDiff = abs(input[j].angle - i);
                minDist = input[j].distance;
            }
        }
        single_scan_data_proc[i] = minDist;
    }
}

int main(){
    processJinText();
    qsort(single_scan_data, total_points, sizeof(pp), comp);
    process(single_scan_data, total_points);
    for (int i = 0; i < FINAL_NUM_POINTS; i++){
        printf("%d: %f\n", i, single_scan_data_proc[i]);
    }
    // for (int i = 0; i < total_points; i++){
    //     printf("%f: %f\n", single_scan_data[i].angle, single_scan_data[i].distance);
    // }
}