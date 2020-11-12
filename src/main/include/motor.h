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
