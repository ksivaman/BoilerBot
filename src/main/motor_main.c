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
#include "include/AnalyzeLiDAR.h"
#include "include/astar.h"
#include "include/navigation.h"

#include "esp_timer.h"

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
// #define LEDC_HS_CH0_GPIO       (21)
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

#define MM_PER_SEC (768)
#define DEGREE_PER_SEC_FAST (3072) //(4096)
#define DEGREE_PER_SEC_SLOW (2560)
#define DEGREE_PER_SEC_2 (2048) //(3584)
#define HOPES 850

void motor_forward(rover robot);

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

// void rover_run(rover robot, int speed, enum dir direction, bool acc) {
//     ledc_channel_config_t channel = robot.pwm;

//     if (direction == FORWARD) {
//         motor_forward(robot);
//     } else if (direction == BACKWARD) {
//         motor_backward(robot);
//     } else if (direction == RIGHT) {
//         motor_right(robot);
//     } else if (direction == LEFT) {
//         motor_left(robot);
//     } else {
//         motor_stop(robot);
//     }

//     if (acc) {
//         ledc_set_fade_with_time(channel.speed_mode, channel.channel, speed, MOTOR_ACCELERATE);
//         ledc_fade_start(channel.speed_mode, channel.channel, LEDC_FADE_NO_WAIT);
//     } else {
//         ledc_set_duty(channel.speed_mode, channel.channel, speed);
//         ledc_update_duty(channel.speed_mode, channel.channel);
//     }

// }
#define OBJECT_TOLERANCE 0 //mm
#define OBSTACLE_WAIT_DURATION 9000000


void toggleMotor(ledc_channel_config_t channel, int speed) {
    ledc_set_duty(channel.speed_mode, channel.channel, speed);
    ledc_update_duty(channel.speed_mode, channel.channel);
}

// returns -1 when moved distance cannot be calculated
float burst_rover(rover robot, int cm, enum compass heading, bool * earlyStop) {
    enum dir direction;
    if (robot.heading == heading) {
        direction = FORWARD;
    } else if (abs(robot.heading - heading) == 180) {
        direction = BACKWARD;
    } else {
        motor_stop(robot);
        return 0;
    }
    ledc_channel_config_t channel = robot.pwm;

    float distanceMoved = 0; // total distance moved
    int requestedDist = cm; // total distance requested to be moved

    bool obstacleFlag = false;
    // bool distanceFlag = false;

    float* lidarScan = getLiDARScan();
    float prevFront = -1, prevBack = -1;
    // printf("HERE1\n");
    getFrontBackDist(&prevFront, &prevBack);
    if (prevBack == -1) // prevBack is more desired, so re try once more.
        getFrontBackDist(&prevFront, &prevBack);

    printf("Front = %f mm, Back = %f mm\n", prevFront, prevBack);
    // printf("HERE2\n");
    int64_t time_start = 0;
    float time_run;

    // int move; // next moving amount
    // char lo[100];
    int64_t waitTime = esp_timer_get_time();
    int64_t newTime = esp_timer_get_time(); 
    obstacleFlag = isThereObstacle_s(lidarScan, 0);
    while(obstacleFlag && ((float)(newTime - waitTime) < (float) OBSTACLE_WAIT_DURATION)) {
        updateLiDARScan(lidarScan);
        obstacleFlag = isThereObstacle_s(lidarScan, 0);
        newTime = esp_timer_get_time();   
    }
    if ((float)(newTime - waitTime) >= (float) OBSTACLE_WAIT_DURATION) {
        *earlyStop = true;
        free(lidarScan);
        return distanceMoved;
    }
    bool first = true;

    while ((distanceMoved < requestedDist) && (fabs(distanceMoved - requestedDist) > 2.4)) {

        if (direction == FORWARD) { motor_forward(robot);  
        } else if (direction == BACKWARD) { motor_backward(robot);
        } else { motor_stop(robot); break;
        }

        if (cm > 25) {
            // move = cm - 3;
            // time_run = 27.765 * move + 390;
            // time_run = 30*cm - 250;
            time_run = 31.176 * cm - 350;
        } else if (cm > 8) {
            time_run = 486; // 8cm
        } else {
            time_run = 300; // 4cm
        }

        time_run *= 1000;
        // Wait untill obstacle is gone
        // if (cm > 10) 
        if (/*obstacleFlag*/ !first && (cm > 10)) {
            updateLiDARScan(lidarScan);
            waitTime = esp_timer_get_time();
            newTime = waitTime;
            obstacleFlag = (cm > 10) ? isThereObstacle_r(lidarScan, 0) : isThereObstacle_s(lidarScan, 0);
            while(obstacleFlag && ((float)(newTime - waitTime) < (float) OBSTACLE_WAIT_DURATION)) {
                updateLiDARScan(lidarScan);
                obstacleFlag = (cm > 10) ? isThereObstacle_r(lidarScan, 0) : isThereObstacle_s(lidarScan, 0);
                newTime = esp_timer_get_time();   
            }
        } else {
             vTaskDelay(200 / portTICK_PERIOD_MS);
        }
        first = false;

        // printf("%lld\n", newTime - waitTime);
        // printf("%d\n", OBSTACLE_WAIT_DURATION);
        // printf("(float)(newTime - waitTime) < OBSTACLE_WAIT_DURATION) = %d\n", (float)(newTime - waitTime) < (float) OBSTACLE_WAIT_DURATION);
        if ((float)(newTime - waitTime) >= (float) OBSTACLE_WAIT_DURATION) {
            *earlyStop = true;
            free(lidarScan);
            return distanceMoved;
        }
        printf("HERE5\n");
       
        // Turn on Motor
        toggleMotor(channel, MM_PER_SEC);
        time_start = esp_timer_get_time();

        while (((float)(esp_timer_get_time() - time_start) < time_run ) && !obstacleFlag) {
            if(cm > 25) {
                updateLiDARScan(lidarScan);
                obstacleFlag = isThereObstacle_r(lidarScan, OBJECT_TOLERANCE);
            }
        }

        //////////////////////////////break
        motor_stop(robot);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        if (direction == FORWARD) {  motor_backward(robot);
        } else if (direction == BACKWARD) { motor_forward(robot);
        } else { motor_stop(robot); break;
        }

        toggleMotor(channel, 512);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        motor_stop(robot);
        toggleMotor(channel, 0);
        /////////////////////////////


        // Calculate Distance
        
        // updateLiDARScan(lidarScan);
        // float newFront = getFrontDist(lidarScan);
        // float newBack = getBackDist(lidarScan);
        float delta, deltaBack, deltaFront;
        if (cm > 25) {
            float newFront = -1, newBack = -1;
            getFrontBackDist(&newFront, &newBack);
            if (obstacleFlag) newFront = -1; // if we stopped due to obstacle, we disregard front measurement

            // Case when distance cannot be calculated
            if (prevFront == -1 && newBack == -1) { // try one more scan
                getFrontBackDist(&newFront, &newBack);
                if (newBack == -1) {
                    free(lidarScan);
                    *earlyStop = true;
                    return -1;
                    printf("here 1");
                }
            }
            if (prevBack == -1 && newFront == -1) { // try one more scan
                getFrontBackDist(&newFront, &newBack);
                if (obstacleFlag) newFront = -1;
                if (newFront == -1) {
                    free(lidarScan);
                    *earlyStop = true;
                    return -1;
                    printf("here 2");
                }
            }
            printf("newFront = %f mm, newBack = %f mm\n", newFront, newBack);
            // Now at least one of them are ok

            
            deltaBack = newBack - prevBack;
            deltaFront = newFront - prevFront;
            
            printf("deltaBack = %f mm, deltaFront = %f mm\n", deltaBack, deltaFront);

            if (!obstacleFlag) {
                if(fabs(deltaBack/10 - cm) > 30) {
                    prevBack = -1;
                    printf("here7_1");
                }
                if(fabs(deltaFront/10 - cm) > 30) {
                    prevFront = -1;
                    printf("here7_2");
                }
            }

            if (direction == BACKWARD) {
                float temp = deltaBack;
                deltaBack = deltaFront;
                deltaFront = temp;
            }
            // if both are avaliable
            if (prevBack != -1 && prevFront != -1) {
                if (deltaBack >= 0 && deltaFront <= 0) { delta = (prevBack < prevFront) ? deltaBack : deltaFront;
                } else if (deltaBack >= 0) { delta = deltaBack;
                } else if (deltaFront <= 0) { delta = deltaFront;
                } else { // both data useless
                    free(lidarScan);
                    *earlyStop = true;
                    printf("here 3");
                    return -1;
                }
            } else if (prevBack != -1) {
                if (deltaBack >= 0) {
                    delta = deltaBack;
                } else {
                    free(lidarScan);
                    *earlyStop = true;
                    printf("here 4");
                    return -1;
                }
            } else { // (prevFront != -1)
                if (deltaFront <= 0) {
                    delta = deltaFront;
                } else {
                    free(lidarScan);
                    *earlyStop = true;
                    printf("here 5");
                    return -1;
                }
            }
            delta = fabs(delta) / 10;

            prevBack = newBack;
            prevFront = newFront;
        } else if (cm > 8) {
            delta = 8;
        } else {
            delta = 4;
        }
        
        printf("delat: %f\n", delta);
        // snprintf(lo, 100, "deltas_%f", delta);
        // prints(lo);
        distanceMoved += (delta);
        
        cm = cm - distanceMoved;
    }

    // snprintf(lo, 100, "out_%d_%d", (distanceMoved < requestedDist), (fabs(distanceMoved - requestedDist) > 1.5));
    // prints(lo);

    free(lidarScan);
    return distanceMoved;
    
}

// void burst_rover(rover robot, int cm, enum dir direction) {
//     ledc_channel_config_t channel = robot.pwm;

//     if (direction == FORWARD) {
//         motor_forward(robot);
//     } else if (direction == BACKWARD) {
//         motor_backward(robot);
//     } else {
//         motor_stop(robot);
//     }

//     // float time = 30* cm + 280;
//     float time = 27.765 * cm + 390;

//     ledc_set_duty(channel.speed_mode, channel.channel, MM_PER_SEC);
//     ledc_update_duty(channel.speed_mode, channel.channel);

//     // while (time)

//     vTaskDelay(time / portTICK_PERIOD_MS);
    
//     //////////////////////////////break
//     motor_stop(robot);
//     vTaskDelay(100 / portTICK_PERIOD_MS);
    
//     if (direction == FORWARD) {  
//         motor_backward(robot);
//     } else if (direction == BACKWARD) {
//         motor_forward(robot);
//     } else {
//         motor_stop(robot);
//     }

//     ledc_set_duty(channel.speed_mode, channel.channel, 512);
//     ledc_update_duty(channel.speed_mode, channel.channel);
//     vTaskDelay(100 / portTICK_PERIOD_MS);
//     motor_stop(robot);
//     ledc_set_duty(channel.speed_mode, channel.channel, 0);
//     ledc_update_duty(channel.speed_mode, channel.channel);
//     /////////////////////////////
    
// }

int turn_rover(rover robot, int degree, enum dir direction) {
    ledc_channel_config_t channel = robot.pwm;
    
    if (direction == RIGHT) {
        degree = degree;
    } else if (direction == LEFT) {
        degree = -degree;
    } else {
        motor_stop(robot);
        return -1;
    }

    float angleTurned = 0;
    float requestedDegree = degree;

    float* prevScan = getLiDARScan();
    float* newScan = getLiDARScan();
    // int64_t time_start = 0;
    // int64_t time_end = 0;
    float time_turn;

    int turn;
    short speed = (robot.currLoc.y < 3) ? DEGREE_PER_SEC_FAST : DEGREE_PER_SEC_SLOW;

    while (abs(angleTurned - requestedDegree) > 1) {
        if (degree > 0) {
            motor_right(robot);
        } else {
            motor_left(robot);
        }

        if (abs(degree) > 8) {
            turn = abs(degree) - 5;
            time_turn = 10.889 * abs(degree) + 60;
        } else {
            turn = 2;
            time_turn = 10.889 * turn + 60;
        }


        toggleMotor(channel, speed);
        vTaskDelay(time_turn / portTICK_PERIOD_MS);
        motor_stop(robot);
        toggleMotor(channel, 0);
        updateLiDARScan(newScan);

        angleTurned = angleDiff(prevScan, newScan);
        degree = requestedDegree - angleTurned;
        printf("___ angle turned: %f\n", angleTurned);
    }

    free(prevScan);
    free(newScan);

    return angleTurned;
}


// bool getOutOfBorder(rover robot, Point curr, Point secondClose, enum compass moveToward) {
//     int angleFacing;
//     bool obstacleFlag;
//     float * curr_scan = getLiDARScan();
//     int curr_error = absoluteErrorFrom(curr_scan, curr, &angleFacing);
//     int second_error = absoluteErrorFrom(curr_scan, secondClose, &angleFacing);
//     while (abs(curr_error - second_error) > SIMILAR_THRESH && !obstacleFlag) {
//         burst_rover(robot, 100, moveToward, &obstacleFlag);
//         updateLiDARScan(curr_scan);
//         curr_error = absoluteErrorFrom(curr_scan, curr, &angleFacing);
//         second_error = absoluteErrorFrom(curr_scan, secondClose, &angleFacing);
//     }
//     free(curr_scan);
//     return obstacleFlag;
// }

// int fitInSquare(rover robot) {
//     int angleFacing = 0;
//     Point secondClose = {0, 0};
//     Point curr = get_curr_loc_input(robot.heading, &angleFacing, &secondClose);
//     bool obstacleFlag = false;

//     // Check if robot is within the border
//     if (secondClose.x != -1) {
//         // Assume robot is alwasy be vertically offset
//         short delta_h = curr.x - secondClose.x;
//         short delta_v = curr.y - secondClose.y;

//         // check if two points are adjacent, and difference are alligned with robot's heading
//         if ((robot.heading == NORTH || robot.heading == SOUTH) && abs(delta_v) == 1 && abs(delta_h) == 0) {
//             enum compass moveToward = (delta_v < 0) ? SOUTH : NORTH;
//             obstacleFlag = getOutOfBorder(robot, curr, secondClose, moveToward);
//         }
//         if ((robot.heading == EAST || robot.heading == WEST) && abs(delta_v) == 0 && abs(delta_h) == 1) {
//             enum compass moveToward = (delta_h < 0) ? EAST : WEST;
//             obstacleFlag = getOutOfBorder(robot, curr, secondClose, moveToward);
//         }
//     }

//     if (obstacleFlag) {
//         return -1;
//     }

//     float * curr_scan = getLiDARScan();
//     // turn if needed
//     absoluteErrorFrom(curr_scan, curr, &angleFacing);
//     int degree = 0;
//     degree = (robot.heading - angleFacing) % FINAL_NUM_POINTS;
//     turn_rover(robot, degree, RIGHT);

//     updateLiDARScan(curr_scan);
//     int offsetX, offsetY;
//     enum compass moveToward = NORTH;
//     int cm = 0;
//     getOffSetFrom(curr_scan, curr, &offsetX, &offsetY);
//     free(curr_scan);
    
//     if ((robot.heading == NORTH || robot.heading == SOUTH)) {
//         moveToward = (offsetY < 0) ? NORTH : SOUTH;
//         cm = abs(offsetY);
//     }
//     if ((robot.heading == EAST || robot.heading == WEST)) {
//         moveToward = (offsetX < 0) ? EAST : WEST;
//         cm = abs(offsetX);
//     }
//     burst_rover(robot, cm, moveToward, &obstacleFlag);


//     if (obstacleFlag) {
//         return -1;
//     }

//     return 0;
// }


// void app_main(void)
// {
//     gpio_config_t io_conf;
//     io_conf.intr_type = GPIO_PIN_INTR_DISABLE; //disable interrupt
//     io_conf.mode = GPIO_MODE_OUTPUT; //set as output mode
//     io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL; //bit mask of the pins that you want to set,e.g.GPIO 18-19, 22-23
//     io_conf.pull_down_en = 1; //enable pull-down mode
//     io_conf.pull_up_en = 0; //disable pull-up mode
//     gpio_config(&io_conf); //configure GPIO with the given settings
    

//     /*
//      * Prepare and set configuration of timers
//      * that will be used by LED Controller
//      */
//     ledc_timer_config_t ledc_timer = {
//         .duty_resolution = LEDC_TIMER_12_BIT, // resolution of PWM duty
//         .freq_hz = 10000,                     // frequency of PWM signal
//         .speed_mode = LEDC_HS_MODE,           // timer mode
//         .timer_num = LEDC_HS_TIMER,            // timer index
//         .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
//     };
//     // Set configuration of timer0 for high speed channels
//     ledc_timer_config(&ledc_timer);

//     /*
//      * Prepare individual configuration
//      * for each channel of LED Controller
//      * by selecting:
//      * - controller's channel number
//      * - output duty cycle, set initially to 0
//      * - GPIO number where LED is connected to
//      * - speed mode, either high or low
//      * - timer servicing selected channel
//      *   Note: if different channels use one timer,
//      *         then frequency and bit_num of these channels
//      *         will be the same
//      */
//     ledc_channel_config_t ledc_channel = 
//         {
//             .channel    = LEDC_HS_CH0_CHANNEL,
//             .duty       = 0,
//             .gpio_num   = LEDC_HS_CH0_GPIO,
//             .speed_mode = LEDC_HS_MODE,
//             .hpoint     = 0,
//             .timer_sel  = LEDC_HS_TIMER
//         };
    
//     // // Set LED Controller with previously prepared configuration
//     ledc_channel_config(&ledc_channel);

//     // Initialize fade service.
//     ledc_fade_func_install(0);

//     rover robot1 = {
//         .pwm = ledc_channel,
//         .motor_1 = GPIO_OUTPUT_IN_1,
//         .motor_2 = GPIO_OUTPUT_IN_2,
//         .motor_3 = GPIO_OUTPUT_IN_3,
//         .motor_4 = GPIO_OUTPUT_IN_4,
//         .heading = NORTH,
//         {
//             .x = 0,
//             .y = 0
//         },
//     };


//     // robot1.pwm = ledc_channel;
//     // robot1.motor_1 = GPIO_OUTPUT_IN_1;
//     // robot1.motor_2 = GPIO_OUTPUT_IN_2;
//     // robot1.motor_3 = GPIO_OUTPUT_IN_3;
//     // robot1.motor_4 = GPIO_OUTPUT_IN_4;


//     // printf("1. Forward burst = %d cm\n", 100);
//     // burst_rover(robot1, 100, NORTH);

//     // vTaskDelay(2000 / portTICK_PERIOD_MS);

//     // printf("1. Backward burst = %d cm\n", 75);
//     // burst_rover(robot1, 75, BACKWARD);

//     // vTaskDelay(2000 / portTICK_PERIOD_MS);

//     // printf("1. Forward burst = %d cm\n", 50);
//     // burst_rover(robot1, 50, FORWARD);

//     // vTaskDelay(2000 / portTICK_PERIOD_MS);

//     // printf("1. Backward burst = %d cm\n", 25);
//     // burst_rover(robot1, 25, BACKWARD);

//     // vTaskDelay(2000 / portTICK_PERIOD_MS);

//     // printf("1. Backward burst = %d cm\n", 50);
//     // burst_rover(robot1, 50, BACKWARD);

//     printf("1. Forward burst = %d cm\n", 100);
//     bool obstacle = false;
//     float movedDist = burst_rover(robot1, 85, FORWARD, &obstacle);

//     while (1) {
        
//         vTaskDelay(10 * LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);
        
//         // printf("1. Forward burst = %d cm\n", 100);
//         // burst_rover(robot1, 100,FORWARD);
//         // rover_run(robot1, LEDC_TEST_DUTY, FORWARD, true); 
        

//         // printf("2. Forward decclerate down to duty = 0\n");
//         // rover_run(robot1, 0, FORWARD, true); 
//         // vTaskDelay(2 * LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);
        
//         // printf("3. Backword accelerate up to duty = %d\n", LEDC_TEST_DUTY);
//         // rover_run(robot1, LEDC_TEST_DUTY, BACKWARD, true); 
//         // vTaskDelay(2 * LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);

//         // printf("4. Stop icmediately\n");
//         // rover_run(robot1, 0, STOP, false);
//         // vTaskDelay(2 * LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);

//         // printf("5. turn right\n");
//         // rover_run(robot1, LEDC_TEST_DUTY, RIGHT, false);
//         // vTaskDelay(3 * LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);

//         // printf("6. Stop icmediately\n");
//         // rover_run(robot1, 0, STOP, false);
//         // vTaskDelay(2 * LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);

//         // printf("7. turn left\n");
//         // rover_run(robot1, LEDC_TEST_DUTY, LEFT, false);
//         // vTaskDelay(3 * LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);
        
//         // // stops
//         // printf("8. Done!\n");
//         // rover_run(robot1, 0, STOP, false);
//         // vTaskDelay(3 * LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);

//     }
// }
