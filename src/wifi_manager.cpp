#include "wifi_manager.h"

#define WIFI_TIMEOUT 20000 //--Maximum time to try to connect (in ms)

extern Config config;

void initWiFi() {
    WiFi.begin(config.ssid.c_str(), config.ssid_pass.c_str());


    unsigned long startAttemptTime = millis();

    //--Try to connect to WiFi
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("WiFi connected.");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("Failed to connect to WiFi.");
    }
}

bool isWiFiConnected() {
    return WiFi.status() == WL_CONNECTED;
}

void handleWiFiConnection() {
    if (!isWiFiConnected()) {
        Serial.println("WiFi disconnected. Attempting to reconnect...");
        initWiFi();
    }
}
