/* UART asynchronous example, that uses separate RX and TX tasks
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "RPLIdar_c.h"
#include "driver/ledc.h"
#include "AnalyzeLiDAR.h"
#include "astar.h"
#include "lidar_data.h"

#ifdef CONFIG_IDF_TARGET_ESP32
#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO       (21)
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#endif

static const int RX_BUF_SIZE = 1024;
static const short NUM_SAMPLES = 400;

#define TXD_PIN (GPIO_NUM_10)
#define RXD_PIN (GPIO_NUM_9)

void init(void) {
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


}

void app_main(void)
{   
    printf("initializing\n");
    init();

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

    // printf("Init done, Turning on the Motor\n");
    
    vTaskDelay(4000 / portTICK_PERIOD_MS);
    printf("Init done, Turning on the Motor\n");

    uint8_t* buff = (uint8_t*) malloc(sizeof(rplidar_response_measurement_node_t) * NUM_SAMPLES);
    rplidar_response_measurement_node_t * node = (rplidar_response_measurement_node_t*)(buff);

    // struct _rplidar_response_measurement_node_t node[100];
    RPLidarMeasurement _currentMeasurement;
    int count = 0;
    int i;

    ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, 0);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
    
    // try to detect RPLIDAR...
    vTaskDelay(1000 / portTICK_PERIOD_MS); 

    stop();
    vTaskDelay(1000 / portTICK_PERIOD_MS); 
    startScan(false, RPLIDAR_DEFAULT_TIMEOUT*2);
    
    // start motor rotating at max allowed speed
    int duty = 1270;
    ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, duty);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
    vTaskDelay(3000 / portTICK_PERIOD_MS); 
    vTaskDelay(3000 / portTICK_PERIOD_MS); 
    vTaskDelay(3000 / portTICK_PERIOD_MS); 


    printf("Determining location...\n");
    fflush(stdout);

    // Get raw data of current scan
    startScan(false, RPLIDAR_DEFAULT_TIMEOUT*2);
    while(!(IS_OK(grabData(RPLIDAR_DEFAULT_TIMEOUT, buff))));
    stop();

    printf("\n\nCurrent Location Scan:\n");
    fflush(stdout);

    // Get ID of the square the bot is in, convert it to (x, y)
    char currScanID = getCurrLoc(node, NUM_SAMPLES, fplan, lidar_data, SQUARES_IN_MAP);
    int x = (int) currScanID / NROWS;
    int y = (int) currScanID % NCOLS;

    printf("Bot is in square (%d, %d)\n", x, y);
    fflush(stdout);
    
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    return;
}