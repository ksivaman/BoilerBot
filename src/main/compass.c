/* i2c - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "math.h"

// static const char *TAG = "i2c-example";

// #define _I2C_NUMBER(num) I2C_NUM_##num
// #define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 8                  /*!< Data buffer length of test buffer */
// #define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

// #define I2C_SLAVE_NUM I2C_NUMBER(CONFIG_I2C_SLAVE_PORT_NUM) /*!< I2C port number for slave dev */
// #define I2C_SLAVE_TX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave tx buffer size */
// #define I2C_SLAVE_RX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave rx buffer size */

#define I2C_MASTER_SCL_IO 22               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 21              /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM 0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 50000        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

// #define BH1750_SENSOR_ADDR 0b0011001            /*!< slave address for BH1750 sensor */
// #define BH1750_CMD_START CONFIG_BH1750_OPMODE   /*!< Operation mode */
// #define ESP_SLAVE_ADDR CONFIG_I2C_SLAVE_ADDRESS /*!< ESP32 slave address, you can set any 7bit value */
#define I2C_SLAVE_ADDR 0b0011110
// #define AGR_READ 1
// #define AGR_WRITE 0

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */

#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

#define BLINK_GPIO 4

SemaphoreHandle_t print_mux = NULL;

// /**
//  * @brief test code to read esp-i2c-slave
//  *        We need to fill the buffer of esp slave device, then master can read them out.
//  *
//  * _______________________________________________________________________________________
//  * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
//  * --------|--------------------------|----------------------|--------------------|------|
//  *
//  */
// static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *data_rd, size_t size)
// {
//     if (size == 0) {
//         return ESP_OK;
//     }
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
//     if (size > 1) {
//         i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
//     }
//     i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
//     i2c_master_stop(cmd);
//     esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
//     i2c_cmd_link_delete(cmd);
//     return ret;
// }

// /**
//  * @brief Test code to write esp-i2c-slave
//  *        Master device write data to slave(both esp32),
//  *        the data will be stored in slave buffer.
//  *        We can read them out from slave buffer.
//  *
//  * ___________________________________________________________________
//  * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
//  * --------|---------------------------|----------------------|------|
//  *
//  */
// static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t *data_wr, size_t size)
// {
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
//     i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
//     i2c_master_stop(cmd);
//     esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
//     i2c_cmd_link_delete(cmd);
//     return ret;
// }

/**
 * @brief test code to operate on BH1750 sensor
 *
 * 1. set operation mode(e.g One time L-resolution mode)
 * _________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write 1 byte + ack  | stop |
 * --------|---------------------------|---------------------|------|
 * 2. wait more than 24 ms
 * 3. read data
 * ______________________________________________________________________________________
 * | start | slave_addr + rd_bit + ack | read 1 byte + ack  | read 1 byte + nack | stop |
 * --------|---------------------------|--------------------|--------------------|------|
 */
static esp_err_t t19_compass_read(uint8_t addr, uint8_t *data_rd, size_t size);
static esp_err_t t19_compass_write(uint8_t data, uint8_t addr){
    i2c_port_t i2c_num = I2C_MASTER_NUM;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, I2C_SLAVE_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, addr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);



    // uint8_t data_rd = 0;
    // if ((ret = t19_compass_read(addr, &data_rd, 1)) != ESP_OK) {
    //     printf("ERROR: compass_read111\n");
    //     return ret;
    // }



    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t t19_compass_read(uint8_t addr, uint8_t *data_rd, size_t size)
{
    i2c_port_t i2c_num = I2C_MASTER_NUM;
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, addr, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);

    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }

    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t t19_compass_read_xyz(short* x, short* y, short* z){
    uint8_t tempL, tempH;
    esp_err_t ret;

    if ((ret = t19_compass_read(0x68, &tempL, 1)) != ESP_OK) {
        printf("ERROR: compass_read_xL");
        return ret;
    }
    if ((ret = t19_compass_read(0x69, &tempH, 1)) != ESP_OK) {
        printf("ERROR: compass_read_xH");
        return ret;
    }
    *x = (tempH << 8) | tempL;

    if ((ret = t19_compass_read(0x6A, &tempL, 1)) != ESP_OK) {
        printf("ERROR: compass_read_yL");
        return ret;
    }
    if ((ret = t19_compass_read(0x6B, &tempH, 1)) != ESP_OK) {
        printf("ERROR: compass_read_yH");
        return ret;
    }
    *y = (tempH << 8) | tempL;

    if ((ret = t19_compass_read(0x6C, &tempL, 1)) != ESP_OK) {
        printf("ERROR: compass_read_zL");
        return ret;
    }
    if ((ret = t19_compass_read(0x6D, &tempH, 1)) != ESP_OK) {
        printf("ERROR: compass_read_zH");
        return ret;
    }
    *z = (tempH << 8) | tempL;

    return ESP_OK;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t i2c_master_sensor_test()
{
    // uint8_t buf;
    int ret;
    uint8_t data_rd = 0;

    short _x=0, _y=0, _z=0;
    short x=0, y=0, z=0;

    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BLINK_GPIO, 0);

    printf("Starting... ... ...\n");

    ESP_ERROR_CHECK(i2c_master_init());
    printf("I2C master init done\n");

    
    if ((ret = t19_compass_write(0x8c, 0x60)) != ESP_OK) {
        printf("ERROR: compass_write1\n");
        return ret;
    }

    if ((ret = t19_compass_write(0x02, 0x61)) != ESP_OK) {
        printf("ERROR: compass_write2\n");
        return ret;
    }

    if ((ret = t19_compass_write(0x10, 0x62)) != ESP_OK) {
        printf("ERROR: compass_write3\n");
        return ret;
    }

    printf("Compass init done\n");

    vTaskDelay(20 / portTICK_RATE_MS);

    if ((ret = t19_compass_read(0x67, &data_rd, 1)) != ESP_OK) {
        printf("ERROR: compass_read\n");
        return ret;
    }
    
    while((data_rd & 0b1000) == 0) {
        // printf("waiting for zyxda %x..\n", data_rd);
        vTaskDelay(5 / portTICK_RATE_MS);
        if ((ret = t19_compass_read(0x67, &data_rd, 1)) != ESP_OK) {
            printf("ERROR: compass_read\n");
            return ret;
        }
    }

    t19_compass_read_xyz(&_x, &_y, &_z);
    
    // printf("data %d %d %d\n", _x, _y, _z);
    int num_avg = 50;
    int heading;

    while (1) {
        for (int i = 0; i < num_avg; i++){
            t19_compass_read(0x67, &data_rd, 1);
            while((data_rd & 0b1000) == 0) {
                // printf("waiting for zyxda..\n");
                // vTaskDelay(5 / portTICK_RATE_MS);
                t19_compass_read(0x67, &data_rd, 1);
            }

            t19_compass_read_xyz(&_x, &_y, &_z);
            x += (short) _x;
            y += (short) _y;
            z += (short) _z;
        }

        x /= num_avg;
        y /= num_avg;
        z /= num_avg;

        heading = atan2(y, x) * 180 / M_PI;
        printf("%d %d %d %d\n", x, y, z, heading);
        if (heading > 85 && heading < 105) {
            gpio_set_level(BLINK_GPIO, 1);
        } else {
            gpio_set_level(BLINK_GPIO, 0);
        }

        // vTaskDelay(100 / portTICK_RATE_MS);
    }
   
    return ret;
}



// void app_main(void)
// {
//     print_mux = xSemaphoreCreateMutex();
//     ESP_ERROR_CHECK(i2c_slave_init());
//     ESP_ERROR_CHECK(i2c_master_init());
//     xTaskCreate(i2c_test_task, "i2c_test_task_0", 1024 * 2, (void *)0, 10, NULL);
//     xTaskCreate(i2c_test_task, "i2c_test_task_1", 1024 * 2, (void *)1, 10, NULL);
// }
