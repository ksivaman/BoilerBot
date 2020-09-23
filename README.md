# BoilerBot
Indoor delivery bot

## To use:
1. Open the ESP_IDF folder (parent folder containing all exaxmples, includes etc.)
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