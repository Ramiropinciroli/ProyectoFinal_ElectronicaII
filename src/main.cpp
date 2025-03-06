#include <Arduino.h>
#include "sensor_handler.h"
#include "config.h"
#include "wifi_manager.h"
#include "display_manager.h"
#include "FSM_sensor_manager.h"
#include "relay_manager.h"
#include "mqtt.h"
#include "publish.h"

Config config;
void setup() {
    Serial.begin(SERIAL_BAUDRATE);

    load_config();

    //--Initialize ultrasonic sensors
    initSensors();

    //--Initialize OLED Display
    initDisplay();

    //--initialize relay
    initrelay();

    //--Initialize WiFi
    initWiFi();

    //--Initialize mqtt protocol
    setupMQTT();

}

unsigned long lastStateChange = 0;

void loop() {

    //--Ensure WiFi stays connected
    handleWiFiConnection();

    //--Ensure MQTT stays connected
    handleMQTTConnection();
    
    //--Run the FSM
    FSM_sensor_run();
}
