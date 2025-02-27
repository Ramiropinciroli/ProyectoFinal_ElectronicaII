#include "persist.h"

Preferences persist;

extern Config config;

void save_config(uint8_t name){
    switch(name){
        case DEVICE_CONFIG:
            persist.begin("config");    //--Open storage area
            persist.putString("ssid",config.ssid);
            persist.putString("ssid_pass",config.ssid_pass);
            persist.end();
            break;
        case IRR_CONFIG:
            persist.begin("config");    //--Open storage area
            persist.end();  //--Close storage area
    }
    //--Verify results
    Serial.println("Config saved:");
    Serial.println(config.ssid);
    Serial.println(config.ssid_pass);
}

void load_config(void){
    //--Open storage area
    persist.begin("config");
    config.ssid=persist.getString("ssid",DEFAULT_WIFI_SSID);
    config.ssid_pass=persist.getString("ssid_pass",DEFAULT_WIFI_PASSWORD);
    //--Close storage area
    persist.end();

    //--Verify results
    Serial.println("Config readed:");
    Serial.print("ssid: ");
    Serial.println(config.ssid);
    Serial.print("ssid_pass: ");
    Serial.println(config.ssid_pass);
}