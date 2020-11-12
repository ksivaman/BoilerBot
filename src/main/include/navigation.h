#ifndef __NAVIGATION_H__
#define __NAVIGATION_H__


#include "astar.h"
#include "AnalyzeLiDAR.h"

enum dir{FORWARD, BACKWARD, RIGHT, LEFT, STOP};

typedef struct {
    ledc_channel_config_t pwm;
    gpio_num_t motor_1;
    gpio_num_t motor_2;
    gpio_num_t motor_3;
    gpio_num_t motor_4;
    enum compass heading;
    Point currLoc;
} rover;


void navigate(Path* pathStart, enum compass* currHeading, rover * robot1);
int getTurnAngle(Path* path, enum compass* currHeading);


#endif