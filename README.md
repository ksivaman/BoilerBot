# BoilerBot
Indoor delivery bot

## To use:
1. Confire and cd into the ESP-IDF folder.
    ```
    git clone https://github.com/espressif/esp-idf
    cd esp-idf
    ```
2. Execute the following commands
    ```
    git clone https://github.com/ksivaman/BoilerBot
    cd BoilerBot/src/main/include
    mkdir include
    cd include
    touch wifi_login.h
    ```
3. In the wifi_login.h, add the following:
    ```
    #define EXAMPLE_ESP_WIFI_SSID      "<your wifi ssid>"
    #define EXAMPLE_ESP_WIFI_PASS      "<your wifi password>"
    ```
4. build, and flash
    ```
    idf.py build
    idf.py -p PORT flash monitor
    ```
    
## Demo

[![Working example](https://img.youtube.com/vi/mLd8BgoYLkw/0.jpg)](https://www.youtube.com/watch?v=mLd8BgoYLkw)
