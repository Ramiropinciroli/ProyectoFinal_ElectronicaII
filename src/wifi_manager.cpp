#include "wifi_manager.h"

#define WIFI_TIMEOUT 20000 // Maximum time to attempt connection (in milliseconds)

extern Config config;

// Function to initialize WiFi connection
void initWiFi() {
    WiFi.begin(config.ssid.c_str(), config.ssid_pass.c_str()); // Start WiFi connection with stored credentials

    unsigned long startAttemptTime = millis(); // Record the start time of the connection attempt

    // Try to connect within the defined timeout
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT) {
        delay(1000);
        Serial.println("Connecting to WiFi..."); // Print connection attempt message
    }

    // Check if the connection was successful
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("WiFi connected."); // Confirm connection
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP()); // Display the assigned IP address
    } else {
        Serial.println("Failed to connect to WiFi."); // Notify if connection attempt failed
    }
}

// Function to check if WiFi is connected
bool isWiFiConnected() {
    return WiFi.status() == WL_CONNECTED;
}

// Function to handle WiFi reconnection if disconnected
void handleWiFiConnection() {
    if (!isWiFiConnected()) { // If WiFi is not connected
        Serial.println("WiFi disconnected. Attempting to reconnect...");
        initWiFi(); // Attempt to reconnect
    }
}

