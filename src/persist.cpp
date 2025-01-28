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
            persist.putInt("irr_hour",config.irr_hour);
            persist.putInt("irr_minute",config.irr_minute);
            persist.putInt("irr_time",config.irr_time);
            persist.end();  //--Close storage area
    }
    //--Verify results
    Serial.println("Config saved:");
    Serial.println(config.ssid);
    Serial.println(config.ssid_pass);
    Serial.println(config.irr_hour);
    Serial.println(config.irr_minute);
    Serial.println(config.irr_time);
}

void load_config(void){
    //--Open storage area
    persist.begin("config");
    config.ssid=persist.getString("ssid",WIFI_SSID);
    config.ssid_pass=persist.getString("ssid_pass",WIFI_PASSWORD);
    config.irr_hour=persist.getInt("irr_hour",DEFAULT_IRR_HOUR);
    config.irr_minute=persist.getInt("irr_minute",DEFAULT_IRR_MINUTE);
    config.irr_time=persist.getInt("irr_time",DEFAULT_IRR_TIME);
    //--Close storage area
    persist.end();

    //--Verify results
    Serial.println("Config readed:");
    Serial.print("ssid: ");
    Serial.println(config.ssid);
    Serial.print("ssid_pass: ");
    Serial.println(config.ssid_pass);
    Serial.print("irr_hour: ");
    Serial.println(config.irr_hour);
    Serial.print("irr_minute: ");
    Serial.println(config.irr_minute);
    Serial.print("irr_time: ");
    Serial.println(config.irr_time);
}