/* LEDC (LED Controller) fade example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"

/*
 * About this example
 *
 * 1. Start with initializing LEDC module:
 *    a. Set the timer of LEDC first, this determines the frequency
 *       and resolution of PWM.
 *    b. Then set the LEDC channel you want to use,
 *       and bind with one of the timers.
 *
 * 2. You need first to install a default fade function,
 *    then you can use fade APIs.
 *
 * 3. You can also set a target duty directly without fading.
 *
 * 4. This example uses GPIO18/19/4/5 as LEDC output,
 *    and it will change the duty repeatedly.
 *
 * 5. GPIO18/19 are from high speed channel group.
 *    GPIO4/5 are from low speed channel group.
 *
 */
#ifdef CONFIG_IDF_TARGET_ESP32
#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO       (21)
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#endif

#define LEDC_TEST_DUTY         (2500)
#define LEDC_TEST_FADE_TIME    (1000)
#define MOTOR_ACCELERATE       (1000)

#define GPIO_OUTPUT_IN_1    (22)
#define GPIO_OUTPUT_IN_2    (19)
#define GPIO_OUTPUT_IN_3    (23)
#define GPIO_OUTPUT_IN_4    (18)
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IN_1) | (1ULL<<GPIO_OUTPUT_IN_2) | (1ULL<<GPIO_OUTPUT_IN_3) | (1ULL<<GPIO_OUTPUT_IN_4))

enum dir{FORWARD, BACKWARD, RIGHT, LEFT, STOP};

typedef struct {
    ledc_channel_config_t pwm;
    gpio_num_t motor_1;
    gpio_num_t motor_2;
    gpio_num_t motor_3;
    gpio_num_t motor_4;
} rover;

void motor_forward(rover robot) {
    gpio_set_level(robot.motor_1, 1);
    gpio_set_level(robot.motor_2, 0);
    gpio_set_level(robot.motor_3, 1);
    gpio_set_level(robot.motor_4, 0);
}

void motor_stop(rover robot) {
    gpio_set_level(robot.motor_1, 0);
    gpio_set_level(robot.motor_2, 0);
    gpio_set_level(robot.motor_3, 0);
    gpio_set_level(robot.motor_4, 0);
}

void motor_backward(rover robot) {
    gpio_set_level(robot.motor_1, 0);
    gpio_set_level(robot.motor_2, 1);
    gpio_set_level(robot.motor_3, 0);
    gpio_set_level(robot.motor_4, 1);
}

void motor_right(rover robot) {
    gpio_set_level(robot.motor_1, 1);
    gpio_set_level(robot.motor_2, 0);
    gpio_set_level(robot.motor_3, 0);
    gpio_set_level(robot.motor_4, 1);
}

void motor_left(rover robot) {
    gpio_set_level(robot.motor_1, 0);
    gpio_set_level(robot.motor_2, 1);
    gpio_set_level(robot.motor_3, 1);
    gpio_set_level(robot.motor_4, 0);
}

void rover_run(rover robot, int speed, enum dir direction, bool acc) {
    ledc_channel_config_t channel = robot.pwm;

    if (direction == FORWARD) {
        motor_forward(robot);
    } else if (direction == BACKWARD) {
        motor_backward(robot);
    } else if (direction == RIGHT) {
        motor_right(robot);
    } else if (direction == LEFT) {
        motor_left(robot);
    } else {
        motor_stop(robot);
    }

    if (acc) {
        ledc_set_fade_with_time(channel.speed_mode, channel.channel, speed, MOTOR_ACCELERATE);
        ledc_fade_start(channel.speed_mode, channel.channel, LEDC_FADE_NO_WAIT);
    } else {
        ledc_set_duty(channel.speed_mode, channel.channel, speed);
        ledc_update_duty(channel.speed_mode, channel.channel);
    }

}


void app_main(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE; //disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT; //set as output mode
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL; //bit mask of the pins that you want to set,e.g.GPIO 18-19, 22-23
    io_conf.pull_down_en = 1; //enable pull-down mode
    io_conf.pull_up_en = 0; //disable pull-up mode
    gpio_config(&io_conf); //configure GPIO with the given settings
    

    /*
     * Prepare and set configuration of timers
     * that will be used by LED Controller
     */
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_12_BIT, // resolution of PWM duty
        .freq_hz = 10000,                     // frequency of PWM signal
        .speed_mode = LEDC_HS_MODE,           // timer mode
        .timer_num = LEDC_HS_TIMER,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
    };
    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);

    /*
     * Prepare individual configuration
     * for each channel of LED Controller
     * by selecting:
     * - controller's channel number
     * - output duty cycle, set initially to 0
     * - GPIO number where LED is connected to
     * - speed mode, either high or low
     * - timer servicing selected channel
     *   Note: if different channels use one timer,
     *         then frequency and bit_num of these channels
     *         will be the same
     */
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

    rover robot1;
    robot1.pwm = ledc_channel;
    robot1.motor_1 = GPIO_OUTPUT_IN_1;
    robot1.motor_2 = GPIO_OUTPUT_IN_2;
    robot1.motor_3 = GPIO_OUTPUT_IN_3;
    robot1.motor_4 = GPIO_OUTPUT_IN_4;

    while (1) {
        
        printf("1. Forward accelerate up to duty = %d\n", LEDC_TEST_DUTY);
        rover_run(robot1, LEDC_TEST_DUTY, FORWARD, true); 
        vTaskDelay(2 * LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);

        printf("2. Forward decclerate down to duty = 0\n");
        rover_run(robot1, 0, FORWARD, true); 
        vTaskDelay(2 * LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);
        
        printf("3. Backword accelerate up to duty = %d\n", LEDC_TEST_DUTY);
        rover_run(robot1, LEDC_TEST_DUTY, BACKWARD, true); 
        vTaskDelay(2 * LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);

        printf("4. Stop immediately\n");
        rover_run(robot1, 0, STOP, false);
        vTaskDelay(2 * LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);

        printf("5. turn right\n");
        rover_run(robot1, LEDC_TEST_DUTY, RIGHT, false);
        vTaskDelay(3 * LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);

        printf("6. Stop immediately\n");
        rover_run(robot1, 0, STOP, false);
        vTaskDelay(2 * LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);

        printf("7. turn left\n");
        rover_run(robot1, LEDC_TEST_DUTY, LEFT, false);
        vTaskDelay(3 * LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);
        
        // stops
        printf("8. Done!\n");
        rover_run(robot1, 0, STOP, false);
        vTaskDelay(3 * LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);

    }
}
