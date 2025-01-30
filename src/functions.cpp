#include "functions.h"

extern Config config;

void conf_read(void){
    ArduinoJson::StaticJsonDocument<1024> parse_comm;
    const char *ssid;
    const char *ssid_pass;
    uint8_t irr_hour;
    uint8_t irr_minute;
    uint8_t irr_time;
    uint32_t st;
    Serial.println(F("Put json config file..."));
    st=millis();
    
    while(millis()-st<TIMEOUT_SERIAL_CONFIG){
        if (Serial.available()>0){
            String comm = Serial.readStringUntil('\n');
            Serial.println(comm);
            DeserializationError error = deserializeJson(parse_comm, comm);
            if (error) {
                Serial.print(F("Faill to parse Json file. "));
                Serial.println(error.f_str());
                break;
            }
            if(parse_comm.containsKey("device")){
                ssid=parse_comm["device"]["ssid"];
                ssid_pass=parse_comm["device"]["ssid_pass"];
                config.ssid=ssid;
                config.ssid_pass=ssid_pass;
                save_config(DEVICE_CONFIG);
            }
            if(parse_comm.containsKey("irr")){
                irr_hour=parse_comm["irr"]["irr_hour"];
                irr_minute=parse_comm["irr"]["irr_minute"];
                irr_time=parse_comm["irr"]["irr_time"];
                config.irr_hour=irr_hour;
                config.irr_minute=irr_minute;
                config.irr_time=irr_time;
                save_config(IRR_CONFIG);
            }
            Serial.println(F("Configuration changed!"));
            return;
        }

 
        delay(100);
    }
    Serial.println(F("No json received. No changes."));
}