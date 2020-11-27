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
#define DEGREE_PER_SEC_FAST (2830) //(3072)
#define DEGREE_PER_SEC_SLOW (2300) //(2560)
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

void toggleMotor(ledc_channel_config_t channel, int speed) {
    ledc_set_duty(channel.speed_mode, channel.channel, speed);
    ledc_update_duty(channel.speed_mode, channel.channel);
}


void backOff(rover robot, int time_run) {
    ledc_channel_config_t channel = robot.pwm;
    // int time_run = 586; // 4.25cm MOVE_SECOND_BOUND

    motor_backward(robot);
    toggleMotor(channel, MM_PER_SEC);
    vTaskDelay(time_run / portTICK_PERIOD_MS);

    //////////////////////////////break
    motor_stop(robot);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    motor_forward(robot);
    toggleMotor(channel, 512);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    motor_stop(robot);
    toggleMotor(channel, 0);
    /////////////////////////////
}

int getBurstTime(int cm, int *remain) {
    int runTime = 0;
    if (cm <= 48) { runTime = 500; cm -= 19;
    } else if (cm <= 79) { runTime = 1000; cm -= 48;
    } else if (cm <= 110) { runTime = 2000; cm -= 79;
    } else if (cm <= 143) { runTime = 2900; cm -= 110;
    } else if (cm <= 176) { runTime = 3800; cm -= 143;
    } else if (cm <= 205) { runTime = 4600; cm -= 176;
    } else if (cm <= 237) { runTime = 5500; cm -= 205;
    } else if (cm <= 268) { runTime = 6400; cm -= 237;
    } else { runTime = 7000; cm -= 268;
    }
    *remain = cm;
    return runTime;
}

#define OBSTACLE_FREE_BOUND 22
#define MOVE_FIRST_BOUND 8.5
#define MOVE_SECOND_BOUND 4.25
#define OBJECT_TOLERANCE 0 //mm
#define OBSTACLE_WAIT_DURATION 8000000 //us

float burst_rover(rover robot, int cm, enum compass heading) {
    // Determine wheather to move front or back
    ledc_channel_config_t channel = robot.pwm;

    if (cm > SQUARE_WIDTH * 4) {
        cm = SQUARE_WIDTH * 4;
    }

    enum dir direction;
    if (robot.heading == heading) {
        direction = FORWARD;
    } else if (abs(robot.heading - heading) == 180) {
        direction = BACKWARD;
    } else {
        motor_stop(robot);
        return 0;
    }
    if (cm > OBSTACLE_FREE_BOUND + 7) {
        cm -= 7;
    }
    float distanceMoved = 0; // total distance moved
    int requestedDist = cm; // total distance requested to be moved
    float time_run;
    
    bool obstacleFlag = false;
    bool obstacleFlag_s = false;

    // Get initial distance measurements
    float prevFront = -1, prevBack = -1;
    if (requestedDist > OBSTACLE_FREE_BOUND){
        getFrontBackDist(&prevFront, &prevBack);
        printf("burst_rover(): Front = %f mm, Back = %f mm\n", prevFront, prevBack);
    }
    
    // Initial Check for Obstacles
    float* lidarScan = getLiDARScan();
    obstacleFlag_s = isThereObstacle_r(lidarScan, 0, direction);
    // obstacleFlag = (cm > OBSTACLE_FREE_BOUND) ? isThereObstacle_r(lidarScan, 0, direction) : isThereObstacle_s(lidarScan, 0, direction);
    if (obstacleFlag_s && (cm > OBSTACLE_FREE_BOUND)) {
        free(lidarScan);
        return -1;
    } else {
        if (isThereObstacle_s(lidarScan, 0, direction)) {
            free(lidarScan);
            return -1;
        }
    }
    printf("burst_rover(): No Obstacle, starting to move\n");


    // Internal loop to move requested distance
    while ((distanceMoved < requestedDist) && (fabs(distanceMoved - requestedDist) > 2.3)) {
        int remain = 0;
        if (cm > OBSTACLE_FREE_BOUND) {
            time_run = getBurstTime(cm, &remain);
        } else if (cm > MOVE_FIRST_BOUND) {
            time_run = 486; // 8.5cm MOVE_FIRST_BOUND
        } else {
            time_run = 300; // 4.25cm MOVE_SECOND_BOUND
        }
        printf("burst_rover(): time_run1 = %fms, remain = %dcm\n", time_run, remain);

        time_run *= 1000;

        bool earlyStop = false;
        int64_t time_end = 0;
        int64_t time_start = 0;

        bool run = true;
        while(run) {
            printf("run_motor(): Turnning on motor\n");
            /////////////////////////////// run_motor()
            if (direction == FORWARD) { motor_forward(robot);  
            } else if (direction == BACKWARD) { motor_backward(robot);
            } else { motor_stop(robot); return 1;
            }
            toggleMotor(channel, MM_PER_SEC);
            time_start = esp_timer_get_time();
            while (((float)(time_end - time_start) < time_run ) && !obstacleFlag) {
                if(cm > OBSTACLE_FREE_BOUND) {
                    updateLiDARScan(lidarScan);
                    obstacleFlag = isThereObstacle_r(lidarScan, OBJECT_TOLERANCE, direction);
                }
                time_end = esp_timer_get_time();
            }
            ////////////////////////////// break
            motor_stop(robot);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            if (direction == FORWARD) {  motor_backward(robot);
            } else if (direction == BACKWARD) { motor_forward(robot);
            } else { motor_stop(robot); return 1;
            }
            toggleMotor(channel, 512);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            motor_stop(robot);
            toggleMotor(channel, 0);
            earlyStop = ((float)(time_end - time_start) < time_run);
            /////////////////////////////
            printf("run_motor(): Stopped\n");

            if (remain > OBSTACLE_FREE_BOUND) {
                // lidarScan = getLiDARScan();
                // obstacleFlag = isThereObstacle_r(lidarScan, 0, direction);
                if (!obstacleFlag) {
                    printf("burst_rover(): Remain big Enough\n");
                    time_run = getBurstTime(remain, &remain);
                    printf("burst_rover(): time_run2 = %f\n", time_run);
                    time_run *= 1000;
                } else {
                    run = false;
                }
                remain = 0;
            } else {
                run = false;
            }
        }

        if (earlyStop) { // Obstacles is here, just exit.
            printf("Stopped due to obstacle");
            free(lidarScan);
            return -1;
        }

        // no obstacle, continue running.
        // Calculate Distance
        float delta, deltaBack, deltaFront;
        if (cm > OBSTACLE_FREE_BOUND) {
            float newFront = -1, newBack = -1;
            getFrontBackDist(&newFront, &newBack);

            // Case when distance cannot be calculated
            if (prevFront == -1 && newBack == -1) { // try one more scan
                free(lidarScan);
                printf("here 1\n");
                return -1;
            }
            if (prevBack == -1 && newFront == -1) { // try one more scan
                free(lidarScan);
                printf("here 2\n");
                return -1;         
            }
            // Now at least one of them are ok
            
            deltaBack = newBack - prevBack;
            deltaFront = newFront - prevFront;

            printf("newFront = %f mm, newBack = %f mm\n", newFront, newBack);
            printf("deltaBack = %f mm, deltaFront = %f mm\n", deltaBack, deltaFront);

            // Consider the case when measurements being corrupted due to moving objects
            if(fabs(deltaBack/10 - cm) > 33) {
                prevBack = -1;
                newBack = -1;
                printf("here7_1\n");
            }
            if(fabs(deltaFront/10 - cm) > 33) {
                prevFront = -1;
                newFront = -1;
                printf("here7_2\n");
            }

            if (direction == BACKWARD) {
                float temp = deltaBack;
                deltaBack = deltaFront;
                deltaFront = temp;
            }
            
            if ((prevBack != -1 && newBack != -1) && (prevFront != -1 && newFront != -1)) { // if both are avaliable
                if (deltaBack >= 0 && deltaFront <= 0) { delta = (prevBack < prevFront) ? deltaBack : deltaFront;
                } else if (deltaBack >= 0) { delta = deltaBack;
                } else if (deltaFront <= 0) { delta = deltaFront;
                } else { // both data useless
                    free(lidarScan);
                    printf("here 3");
                    return -1;
                }
            } else if (prevBack != -1) { // back is avaliable
                if (deltaBack >= 0) {
                    delta = deltaBack;
                } else {
                    free(lidarScan);
                    printf("here 4");
                    return -1;
                }
            } else { // (prevFront != -1) // front is avaliable
                if (deltaFront <= 0) {
                    delta = deltaFront;
                } else {
                    free(lidarScan);
                    printf("here 5");
                    return -1;
                }
            }
            delta = fabs(delta) / 10;

            prevBack = newBack;
            prevFront = newFront;

        } else if (cm > MOVE_FIRST_BOUND) {
            delta = MOVE_FIRST_BOUND;
        } else {
            delta = MOVE_SECOND_BOUND;
        }
        
        printf("delat: %f\n", delta);;
        distanceMoved += (delta);
        
        bool movedLot = cm > OBSTACLE_FREE_BOUND;
        // cm = cm - distanceMoved;
        cm = cm - delta;
        
        if (cm > OBSTACLE_FREE_BOUND || obstacleFlag_s || movedLot) {
            updateLiDARScan(lidarScan);
            obstacleFlag_s = isThereObstacle_r(lidarScan, 0, direction);
            obstacleFlag = (cm > OBSTACLE_FREE_BOUND) ? obstacleFlag_s : isThereObstacle_s(lidarScan, 0, direction);
            if (obstacleFlag) {
                free(lidarScan);
                return -1;
            }
        }
        // Continue unless there is obstacle in short-range;
    }

    free(lidarScan);
    return distanceMoved;
    
}

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
    if (robot.currLoc.y < 3) {
        printf("Turning with Fast Speed\n");
    } else {
        printf("Turning with Slow Speed\n");
    }

    while (abs(angleTurned - requestedDegree) > 2) {
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
        if (abs(degree) >= 180) {
            if (degree > 0) {
                degree = degree - 360;
            } else {
                degree = degree + 360;
            }
        }
        printf("need to turn %d degree more\n", degree);
        printf("___ angle turned: %f\n\n", angleTurned);
    }

    free(prevScan);
    free(newScan);

    return angleTurned;
}