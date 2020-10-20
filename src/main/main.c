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

#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_http_client.h"
#include "include/wifi_login.h"
#include "astar.c"
#include "compass.c"

#define LOCK_GPIO 4

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
    
    start->x = (buf[0] - 48) * 10 + (buf[1] - 48);
    start->y = (buf[3] - 48) * 10 + (buf[4] - 48);
    end->x = (buf[6] - 48) * 10 + (buf[7] - 48);
    end->y = (buf[9] - 48) * 10 + (buf[10] - 48);
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

void unlock(){
    printf("__________ Unlocking...\n");
    gpio_set_level(LOCK_GPIO, 1);
    vTaskDelay(5000/ portTICK_PERIOD_MS);

    printf("__________ Locking...\n");
    gpio_set_level(LOCK_GPIO, 0);
    vTaskDelete( NULL );
}

void create_unlock_task(void) {
    BaseType_t xReturned;

    /* Create the task, storing the handle. */
    xReturned = xTaskCreate(
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
    char buf[1024];

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

    // Wait till there is a request
    printf("__________ Waiting for new delivery request...\n");
    while ((read_bytes = get_url_content("http://boilerbot-289518.uc.r.appspot.com/admin/get_from_queue", buf, 12)) <= 2){
        // Poll website every second
        vTaskDelay(1000/ portTICK_PERIOD_MS);
    }

    // Request found
    printf("__________ Found new delivery request!\n");

    getRequest(buf, &start, &end);
    printf("Start: (%d, %d); End: (%d, %d)\n", start.x, start.y, end.x, end.y);

    // Get path from curr to start, and navigate
    Path* path = getPathAStar(NROWS, NCOLS, fplan, curr, start);
    printPath(path);

    // Go from curr to start
    printf("__________ Navigating from curr to start...\n");
    // TODO: navigate(path);

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
    vTaskDelay(30000/ portTICK_PERIOD_MS); // simulating the navigate func
    printf("__________ Reached destination!\n");
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
