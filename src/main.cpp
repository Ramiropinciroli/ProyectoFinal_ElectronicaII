#include <Arduino.h>
#include "sensor_handler.h"
#include "config.h"
#include "wifi_manager.h"
#include "display_manager.h"
#include "FSM_sensor_manager.h"
#include "wifi_manager.h"

void setup() {
    Serial.begin(SERIAL_BAUDRATE);

    //--Initialize ultrasonic sensors
    initSensors();

    //--Initialize OLED Display
    initDisplay();

    //--Set the relay pin as an output and turn it off at startup
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW); //--Relay initially off

    //--Initialize WiFi
    initWiFi();
}

unsigned long lastStateChange = 0;

void loop() {

    //--Ensure WiFi stays connected
    handleWiFiConnection();
    
    //--Run the FSM
    FSM_sensor_run();
}
