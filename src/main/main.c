#include <string.h>
#include <stdio.h>
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_http_client.h"
#include "include/wifi_login.h"
#include "astar.c"
#include "include/constants.h"
#include "include/lidar_data.h"
#include "AnalyzeLiDAR.c"
#include "esp_timer.h"
#include "motor_main.c"
#include <stdlib.h>
#include "navigation.c"

/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_MAXIMUM_RETRY  5

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "wifi station";

static int s_retry_num = 0;

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
    vEventGroupDelete(s_wifi_event_group);
}

void getRequest(char* buf, Point * start, Point * end) {
    start->x = (buf[0] - 48);
    start->y = (buf[2] - 48);
    end->x = (buf[4] - 48);
    end->y = (buf[6] - 48);
}

int post_url_content(const char url[], char buf[], int length) {
    esp_err_t err;
    // Add correct website domain later
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_PUT,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if ((err = esp_http_client_open(client, 1024)) != ESP_OK){
        printf("Error opening client connection\n");
    }
    // esp_http_client_set_header(client, "Content-Type", "application/json");

    int write_bytes;
    write_bytes = esp_http_client_write(client, buf, length);

    //cleanup
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    return write_bytes;
}

int get_url_content(const char url[], char buf[], int length) {
    esp_err_t err;
    // Add correct website domain later
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if ((err = esp_http_client_open(client, 0)) != ESP_OK){
        printf("Error opening client connection\n");
    }
    
    // Get content-length
    int content_length;
    if ((content_length = esp_http_client_fetch_headers(client)) == ESP_FAIL){
        printf("Error fetching headers\n");
    }

    int read_bytes;
    read_bytes = esp_http_client_read(client, buf, length);
    buf[read_bytes] = '\0';

    //cleanup
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    return read_bytes;
}

void prints(char * p) {
    char buf[1024];
    char empty[10];
    snprintf(buf, 1024, "http://boilerbot-289518.uc.r.appspot.com/admin/print?text=%s", p);
    get_url_content(buf, empty, 0);
}

void unlock(){
    printf("__________ Unlocking...\n");
    gpio_set_level(LOCK_GPIO, 1);
    vTaskDelay(5000/ portTICK_PERIOD_MS);

    printf("__________ Locking...\n");
    gpio_set_level(LOCK_GPIO, 0);
    vTaskDelete( NULL );
}

void create_unlock_task(void) {

    /* Create the task, storing the handle. */
    xTaskCreate(
                    unlock,                 /* Function that implements the task. */
                    "unlock the lock",      /* Text name for the task. */
                    1000,                   /* Stack size in words, not bytes. */
                    NULL,                   /* Parameter passed into the task. */
                    1,                      /* Priority at which the task is created. */
                    NULL );                 /* Used to pass out the created task's handle. */
}



void app_main(void) {
    int read_bytes;
    Point start, end, curr; 
    start.x = 0; start.y = 0; end.x = 0; end.y = 0;
    curr.x = 2; curr.y = 1;
    char buf[100];
    printf("__________ Wassup. Im here. Back at work. Lets go...\n");

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    printf("__________ Initialized flash...\n");

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");

    // Setup block
    // Setup lock gpio
    gpio_pad_select_gpio(LOCK_GPIO);
    gpio_set_direction(LOCK_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LOCK_GPIO, 0);

    // Check if not connected, and keep trying again and again
    wifi_init_sta();
    printf("__________ Initialized wifi!\n");



    // ROVER INITIALIZE
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
        .timer_num = LEDC_TIMER_1,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
    };
    // // Set configuration of timer0 for high speed channels
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
        .channel    = LEDC_CHANNEL_1,
        .duty       = 0,
        .gpio_num   = 21,
        .speed_mode = LEDC_HS_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_1
    };
    
    // // Set LED Controller with previously prepared configuration
    ledc_channel_config(&ledc_channel);

    // Initialize fade service.
    // ledc_fade_func_install(0);

    rover robot1 = {
        .pwm = ledc_channel,
        .motor_1 = GPIO_OUTPUT_IN_1,
        .motor_2 = GPIO_OUTPUT_IN_2,
        .motor_3 = GPIO_OUTPUT_IN_3,
        .motor_4 = GPIO_OUTPUT_IN_4,
        .heading = NORTH,
        {
            .x = 0,
            .y = 6
        },
    };

    // vTaskDelay(4000 / portTICK_PERIOD_MS);
    bool obstacle = false;
    
    //Initialize LiDAR stuff
    printf("turningOn Lidar\n");
    init_lidar();
    vTaskDelay(4000 / portTICK_PERIOD_MS);

    // float moved = burst_rover(robot1, SQUARE_WIDTH, NORTH, &obstacle);
    // printf("BustEnded, moved: %f\n", moved);

    // Point test_curr = {3,5};
    // // printf("Curr = (%d, %d)\n", curr.x, curr.y);
    // int offsetNorth = 0;
    // int offsetEast = 0;
    // float * newScan = getLiDARScan();
    // robot1.currLoc = test_curr;
    // robot1.heading = WEST;
    // float frontDist, backDist;
    // // Point secondClose = {-1, -1};
    // // int angle;
    // // printf("%lf\n", sqrt(5));
    // while(1) {
    //     updateLiDARScan(newScan);
    //     // robot1.currLoc = getCurrLoc();
    //     // printf("CurrLoc = (%d, %d)\n", robot1.currLoc.x, robot1.currLoc.y);
    //     // int angleOffset = getAngleOffset(robot1);
    //     // angleOffsetHelper(newScan, 45, WEST);
    //     // printf("AngleOffset = %d\n", angleOffset);

    //     obstacle = isThereObstacle_r(newScan, 0, FORWARD);
    //     printf("Obstacle_r: %d\n", (int) obstacle);
    //     obstacle = isThereObstacle_s(newScan, 0, FORWARD);
    //     printf("Obstacle_s: %d\n", (int) obstacle);

    //     // printf("Orig Version\n");
    //     // test_curr = getCurrLoc();
    //     // int error = absoluteErrorFrom(newScan, test_curr, &angle);
    //     // printf("New Version\n");
    //     // test_curr = get_curr_loc_input(NORTH, &angle, &secondClose);

    //     // printf("New Version\n");
    //     // getOffsets(robot1, &offsetNorth, &offsetEast);
    //     // printf("North %d, East: %d\n", offsetNorth, offsetEast);
    //     // printf("\n\n");
    //     // printf("Orig Version\n");
    //     // getOffSetFrom(newScan, test_curr, &offsetNorth, &offsetEast);
    //     // printf("North %d, East: %d\n", offsetNorth, offsetEast);
    //     // printf("\n\n");

    //     vTaskDelay(1000 / portTICK_PERIOD_MS);

    //     // getFrontBackDist(&frontDist, &backDist);
    //     // printf("Front = %f mm, Back = %f mm\n", frontDist, backDist);   
    // }

    bool earlyStop = false;
    robot1.heading = NORTH;
    // robot1.currLoc = getCurrLoc();

    reposition(&robot1);
    // float * newScan = getLiDARScan();
    // int offset_a = getAngleOffset(robot1);
    // float cm_right = fabs(offsetHelper(newScan, 30, 0, EAST, 1));
    // float cm_left = fabs(offsetHelper(newScan, 30, 0, WEST, 1));
    // printf("cm_right = %f, cm_left = %f\n", cm_right, cm_left);
    // getOut(robot1, offset_a, (cm_right < cm_left) ? RIGHT : LEFT);
    // while (1) {
    //     robot1.currLoc = getCurrLoc();
    //     printf("CurrLoc = (%d, %d)\n", robot1.currLoc.x, robot1.currLoc.y);
    //     int offset_n, offset_e;
        //     getOffsets(robot1, &offset_n, &offset_e);
    //     printf("North %d, East: %d\n", offset_n, offset_e);

    //     int move = 0;
    //     if ((robot1.heading == EAST || robot1.heading == WEST)) {
    //         while (abs(offset_e) > 2) {
    //             if (abs(offset_e) > OBSTACLE_FREE_BOUND) {
    //                 do {
    //                     move = OBSTACLE_FREE_BOUND;
    //                     printf("move: %d\n", move);
    //                     burst_rover(robot1, move, (offset_e > 0) ? WEST : EAST);
    //                     offset_e = (offset_e > 0) ? (offset_e - OBSTACLE_FREE_BOUND) : (offset_e + OBSTACLE_FREE_BOUND);
    //                 } while (abs(offset_e) > OBSTACLE_FREE_BOUND);
    //             }
    //             burst_rover(robot1, abs(offset_e), (offset_e > 0) ? WEST : EAST);
    //             getOffsets(robot1, &offset_n, &offset_e);
    //         }
    //     } else {
    //         while (abs(offset_n) > 2) {
    //             if (abs(offset_n) > OBSTACLE_FREE_BOUND) {
    //                 do {
    //                     move = OBSTACLE_FREE_BOUND;
    //                     printf("move: %d\n", move);
    //                     burst_rover(robot1, move, (offset_n > 0) ? SOUTH : NORTH);
    //                     offset_n = (offset_n > 0) ? (offset_n - OBSTACLE_FREE_BOUND) : (offset_n + OBSTACLE_FREE_BOUND);
    //                 } while (abs(offset_n) > OBSTACLE_FREE_BOUND);
    //             }
    //             burst_rover(robot1, abs(offset_n), (offset_n > 0) ? SOUTH : NORTH);
    //             getOffsets(robot1, &offset_n, &offset_e);
    //         }
    //     }
    // }
    
    while(1);

    burst_rover(robot1, 85, NORTH);


    robot1.currLoc.x = 1;
    robot1.currLoc.y = 2;
    robot1.heading = NORTH;
    burst_rover(robot1, 2 * 85, NORTH);
    robot1.currLoc.x = 3;
    robot1.currLoc.y = 2;

    turn_rover(robot1, -90, RIGHT);
    robot1.heading = WEST;

    burst_rover(robot1, 3 * 85, WEST);
    robot1.currLoc.x = 3;
    robot1.currLoc.y = 5;

    turn_rover(robot1, -90, RIGHT);
    robot1.heading = SOUTH;

    burst_rover(robot1, 3 * 85, SOUTH);
    robot1.currLoc.x = 0;
    robot1.currLoc.y = 5;

    turn_rover(robot1, 90, RIGHT);
    robot1.heading = WEST;

    burst_rover(robot1, 85, WEST);
    robot1.currLoc.x = 0;
    robot1.currLoc.y = 6;

    while(1);


    // float frontDist = -1;
    // float backDist = -1;
    // printf("GetCurrLoc\n");
    // curr = getCurrLoc();
    // printf("Curr = (%d, %d)\n", curr.x, curr.y);
    // robot1.currLoc.x = curr.x;
    // robot1.currLoc.y = curr.y;
    // printf("AdjustHeading\n");
    // adjustHeading(&robot1);

    // start.x = 0; start.y = 4;
    // end.x = 3; end.y = 4;

    // Path * testPath = getPathAStar(NROWS, NCOLS, fplan, start, end);
    // printf("CallingNavigation\n");
    // navigate(testPath, &robot1);
    // Testing obstacle Detection
    
    

    // printf("StartingBurst\n");
    // float moved = burst_rover(robot1, 3* 85, NORTH, &obstacle);
    // printf("BustEnded, moved: %f\n", moved);

    // printf("StartingTurn\n");
    // float turned = turn_rover(robot1, 90, RIGHT);
    // printf("TurnEnded, turned: %f\n", turned);
    // // burst_rover(robot1, 85, FORWARD);
    // snprintf(lo, 114, "moved_%f___obstacle_%d", moved, obstacle);
    // prints(lo);

    // //Initialize LiDAR stuff
    // init_lidar();
    // // Point secondClose = {-1, -1};
    // // int angle;

    // bool obstacle;
    // float moved = burst_rover(robot1, 114, NORTH, &obstacle);
    // // burst_rover(robot1, 85, FORWARD);
    // snprintf(lo, 114, "moved_%f___obstacle_%d", moved, obstacle);
    // prints(lo);

    // curr = getCurrLoc();


    // if (1) {return;}
    get_url_content("http://boilerbot-289518.uc.r.appspot.com/admin/restart_vars", buf, 3);
    // while(1) {
    //     float * newScan = getLiDARScan();
    //     int angle;
    //     int currHead = -90;
    //     Point p = {3, 2};
    //     absoluteErrorFrom(newScan, p, &angle);
    //     free(newScan);
        
    //     int subangle = 0;

    //     if (angle > 180){
    //         angle -= 360;
    //     }
    //     subangle = currHead;
    //     if (currHead == 270){
    //         subangle = -90;
    //     }
    //     printf("Angle: %d, currHeading = %d, angleDiff %d \n", angle, currHead, angle - subangle);
    //     vTaskDelay(2000 / portTICK_PERIOD_MS);
    // }

    
    

    // if (abs(subangle - angle) > 5 )
    //     turn_rover(*robot1, subangle-angle , RIGHT);

    // turn_rover(robot1, 90, RIGHT);
    // turn_rover(robot1, 90, RIGHT);
    // turn_rover(robot1, 90, RIGHT);
    // turn_rover(robot1, 90, RIGHT);
    // if (1) return;



    // Wait till there is a request
    printf("__________ Waiting for new delivery request...\n");
    while ((read_bytes = get_url_content("http://boilerbot-289518.uc.r.appspot.com/admin/get_from_queue", buf, 12)) <= 2){
        // Poll website every second
        vTaskDelay(1000/ portTICK_PERIOD_MS);
    }

    // Request found
    printf("__________ Found new delivery request!\n");

    getRequest(buf, &start, &end);

    printf("GetCurrLoc\n");
    curr = getCurrLoc();
    printf("Curr = (%d, %d)\n", curr.x, curr.y);
    robot1.currLoc.x = curr.x;
    robot1.currLoc.y = curr.y;
    printf("AdjustHeading\n");
    adjustHeading(robot1);
 
    printf("Start: (%d, %d); End: (%d, %d)\n", start.x, start.y, end.x, end.y);

    // Get path from curr to start, and navigate
    enum compass heading = NORTH;
    Path* path = getPathAStar(NROWS, NCOLS, fplan, curr, start);
    printPath(path);

    // Go from curr to start
    printf("__________ Navigating from curr to start...\n");
    // while (!isPointEqual(robot1.currLoc, start)) {
    //     navigate(path, &heading, &robot1);
    // }
    // navigate(path, &heading, &robot1);
    int angle;
    Point secondClose;

    // int navigateFlag = navigate(path, &heading, &robot1);
    // while ((navigateFlag == -1) && !isPointEqual(robot1.currLoc, start)) {
    //     path = getPathAStar(NROWS, NCOLS, fplan, robot1.currLoc, start);
    //     navigateFlag = navigate(path, &heading, &robot1);
    //     curr = get_curr_loc_input(robot1.heading, &angle, &secondClose);
    //     printf("Curr_1 = (%d, %d)\n", curr.x, curr.y);
    //     robot1.currLoc.x = curr.x;
    //     robot1.currLoc.y = curr.y;
    // } 

    printf("__________ Waiting for sender to start delivery...\n");
    while ((read_bytes = get_url_content("http://boilerbot-289518.uc.r.appspot.com/admin/has_delivery_started", buf, 3)) <= 2){
        // Poll website every second
        read_bytes = get_url_content("http://boilerbot-289518.uc.r.appspot.com/admin/check_unlock", buf, 1);
        if (buf[0] == 'y') {
            create_unlock_task();
        } 
        vTaskDelay(1000/ portTICK_PERIOD_MS);
    }

    // Go from start to end
    path = getPathAStar(NROWS, NCOLS, fplan, start, end);
    printPath(path);

    // TODO: navigate(path);
    printf("__________ Navigating from start to end...\n");
    // navigate(path, &robot1);
    // navigate(path, &heading, &robot1);
    
    // navigateFlag = navigate(path, &heading, &robot1);
    //     while ((navigateFlag == -1) && !isPointEqual(robot1.currLoc, end)) {
    //     path = getPathAStar(NROWS, NCOLS, fplan, robot1.currLoc, end);
    //     navigateFlag = navigate(path, &heading, &robot1);
    //     curr = get_curr_loc_input(robot1.heading, &angle, &secondClose);
    //     printf("Curr_2 = (%d, %d)\n", curr.x, curr.y);
    //     robot1.currLoc.x = curr.x;
    //     robot1.currLoc.y = curr.y;
    // } 

    // vTaskDelay(30000/ portTICK_PERIOD_MS); // simulating the navigate func

    printf("__________ Reached destination!\n");
    printf("__________ Bot's current orientation is %d degs\n", heading);
    get_url_content("http://boilerbot-289518.uc.r.appspot.com/admin/set_reached_destination", buf, 3);

    printf("__________ Waiting for receiver to end delivery...\n");
    while ((read_bytes = get_url_content("http://boilerbot-289518.uc.r.appspot.com/admin/has_delivery_ended", buf, 3)) <= 2){
        // Poll website every second
        read_bytes = get_url_content("http://boilerbot-289518.uc.r.appspot.com/admin/check_unlock", buf, 1);
        if (buf[0] == 'y') {
            create_unlock_task();
        } 
        vTaskDelay(1000/ portTICK_PERIOD_MS);
    }

    printf("__________ Delivery completed. Going idle...\n");

    // i2c_master_sensor_test();
    printf("__________ Done. Successfully I hope...\n");

}

// getCurPath, 