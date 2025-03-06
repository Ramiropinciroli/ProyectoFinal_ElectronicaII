#include "persist.h"

// Create an instance of Preferences to handle persistent storage
Preferences persist;

// External configuration structure
extern Config config;

// Function to save configuration data into persistent storage
void save_config(uint8_t name) {
    switch(name) {
        case DEVICE_CONFIG:
            persist.begin("config"); // Open the storage area
            persist.putString("ssid", config.ssid); // Store WiFi SSID
            persist.putString("ssid_pass", config.ssid_pass); // Store WiFi password
            persist.putInt("max_distance", config.max_distance); // Store max distance setting
            persist.end(); // Close the storage area
            break;
        case IRR_CONFIG:
            persist.begin("config"); // Open the storage area
            persist.end(); // Close the storage area without changes
    }

    // Print stored configuration values for verification
    Serial.println("Config saved:");
    Serial.println(config.ssid);
    Serial.println(config.ssid_pass);
    Serial.println(config.max_distance);
}

// Function to load configuration data from persistent storage
void load_config(void) {
    // Open the storage area
    persist.begin("config");

    // Retrieve stored values, using defaults if they are not found
    config.ssid = persist.getString("ssid", DEFAULT_WIFI_SSID);
    config.ssid_pass = persist.getString("ssid_pass", DEFAULT_WIFI_PASSWORD);
    config.max_distance = persist.getInt("max_distance", DEFAULT_MAX_DISTANCE);

    // Close the storage area
    persist.end();

    // Print loaded configuration values for verification
    Serial.println("Config read:");
    Serial.print("SSID: ");
    Serial.println(config.ssid);
    Serial.print("SSID Password: ");
    Serial.println(config.ssid_pass);
    Serial.print("Max Distance: ");
    Serial.println(config.max_distance);
}
