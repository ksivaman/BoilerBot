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

#ifdef CONFIG_IDF_TARGET_ESP32
#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO       (21)
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#endif

static const int RX_BUF_SIZE = 1024;

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

int sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, (const char *) data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

// static void tx_task(void *arg)
// {
//     static const char *TX_TASK_TAG = "TX_TASK";
//     esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
//     while (1) {
//         sendData(TX_TASK_TAG, "Hello world");
//         vTaskDelay(2000 / portTICK_PERIOD_MS);
//     }
// }

// static void rx_task(void *arg)
// {
//     static const char *RX_TASK_TAG = "RX_TASK";
//     esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
//     uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
//     while (1) {
//         const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
//         if (rxBytes > 0) {
//             data[rxBytes] = 0;
//             ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
//             ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
//         }
//     }
//     free(data);
// }

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

    uint8_t* buff = (uint8_t*) malloc(sizeof(rplidar_response_measurement_node_t) * 400);
    rplidar_response_measurement_node_t * node = (rplidar_response_measurement_node_t*)(buff);

    // struct _rplidar_response_measurement_node_t node[100];
    RPLidarMeasurement _currentMeasurement;
    int count = 0;
    int i;
    printf("hello??");

    ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, 0);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
    
    // try to detect RPLIDAR...
    vTaskDelay(1000 / portTICK_PERIOD_MS); 

    startScan(false, RPLIDAR_DEFAULT_TIMEOUT*2);
    
    // start motor rotating at max allowed speed
    int duty = 1000;
    ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, duty);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
    vTaskDelay(3000 / portTICK_PERIOD_MS); 
    vTaskDelay(3000 / portTICK_PERIOD_MS); 
    vTaskDelay(3000 / portTICK_PERIOD_MS); 

    float angle = 0;
    float prevAngle = angle;

    // if (IS_OK(grabData(RPLIDAR_DEFAULT_TIMEOUT, buff))) {
    // // if (IS_OK(waitPoint(RPLIDAR_DEFAULT_TIMEOUT, & _currentMeasurement))) {
    //     // printf("Distance: %f, Angle: %f\n", _currentMeasurement.distance, _currentMeasurement.angle);
        
        
    //     for (i = 0; i < 400; i++) {
    //         prevAngle = angle;
    //         angle = (node[i].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
    //         printf("Distance: %f, Angle: %f", node[i].distance_q2/4.0f, (node[i].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
    //         printf(" --- diff is %f\n", angle - prevAngle);
    //     }
    // }

    // printf("\n\nTry waiting\n\n");
    // vTaskDelay(3000 / portTICK_PERIOD_MS); 
    printf("\nnewData\n");

    // if (IS_OK(grabData(RPLIDAR_DEFAULT_TIMEOUT, buff))) {
    // // if (IS_OK(waitPoint(RPLIDAR_DEFAULT_TIMEOUT, & _currentMeasurement))) {
    //     // printf("Distance: %f, Angle: %f\n", _currentMeasurement.distance, _currentMeasurement.angle);
        
        
    //     for (i = 0; i < 400; i++) {
    //         prevAngle = angle;
    //         angle = (node[i].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
    //         printf("Distance: %f, Angle: %f", node[i].distance_q2/4.0f, (node[i].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
    //         printf(" --- diff is %f\n", angle - prevAngle);
    //     }
    // }

    while(duty < 4000) {
        // vTaskDelay(1000 / portTICK_PERIOD_MS); 
        float sum = 0;
        if (IS_OK(grabData(RPLIDAR_DEFAULT_TIMEOUT, buff))) {
        // if (IS_OK(waitPoint(RPLIDAR_DEFAULT_TIMEOUT, & _currentMeasurement))) {
            // printf("Distance: %f, Angle: %f\n", _currentMeasurement.distance, _currentMeasurement.angle);
            float angle = 0;
            float prevAngle = angle;
            float diff = 0;
            int div = 0;
            for (i = 0; i < 400; i++) {
                prevAngle = angle;
                angle = (node[i].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
                diff = angle - prevAngle;
                // printf("Distance: %f, Angle: %f", node[i].distance_q2/4.0f, (node[i].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
                // printf(" --- diff is %f\n", diff);
                if (diff > -20 && diff < 20) {
                    sum += diff;
                    div++;
                }
                
            }
            printf("\nstop");
            printf("Duty: %0.2f%%, Avg Angle diff: %f\n", duty / 4096.0 * 100, sum / div);
            vTaskDelay(20 / portTICK_PERIOD_MS); 
            stop();
            duty += 10;
            ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, duty);
            ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
            vTaskDelay(3000 / portTICK_PERIOD_MS); 
        }
            startScan(false, RPLIDAR_DEFAULT_TIMEOUT*2);
            vTaskDelay(20 / portTICK_PERIOD_MS); 
            // ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, 0);
            // ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
            
            // // try to detect RPLIDAR...
            // vTaskDelay(1000 / portTICK_PERIOD_MS); 

            // startScan(false, RPLIDAR_DEFAULT_TIMEOUT*2);
            
            // // start motor rotating at max allowed speed
            // ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, 1024);
            // ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
            // vTaskDelay(1000 / portTICK_PERIOD_MS); 

        /* Original testing code
        uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);

        // Send START SCAN
        data[0] = 0xA5;
        data[1] = 0x20;
        data[2] = '\0';
        static const char *TX_TASK_TAG = "Task";
        printf("Sending START_SCAN: ");
        for(int i = 0; i < 2; i++) {
            printf("%02x", data[i]);
        }
        esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
        sendData(TX_TASK_TAG, (const char *) data);
        // Read Response Descriptor Packet
        int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
        data[rxBytes] = '\0';
        printf("Received: ");
        for(int i = 0; i < rxBytes; i++) {
            printf("%02x", data[i]);
        }
        printf("\n");
        
        printf("waiting\n");
        vTaskDelay(4000 / portTICK_PERIOD_MS);
        printf("waiting Done\n");

        // Send STOP
        data[0] = 0xA5;
        data[1] = 0x25;
        data[2] = '\0';
        printf("Sending STOP: ");
        for(int i = 0; i < 2; i++) {
            printf("%02x", data[i]);
        }
        esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
        sendData(TX_TASK_TAG, (const char *) data);
        rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
        data[rxBytes] = '\0';
        vTaskDelay(4000 / portTICK_PERIOD_MS);
        */
    }
    
    // xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
    // xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);

    // Send start scan command


}
