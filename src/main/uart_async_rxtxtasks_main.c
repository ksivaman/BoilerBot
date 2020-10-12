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
    init();
    while(1) {
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
    }
    
    // xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
    // xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);

    // Send start scan command


}
