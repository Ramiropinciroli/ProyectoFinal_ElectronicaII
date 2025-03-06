#include "functions.h"

extern Config config;

// Function to read configuration data from a JSON file via Serial communication
void conf_read(void) {
    // Create a JSON document to store the parsed data
    ArduinoJson::StaticJsonDocument<1024> parse_comm;
    const char *ssid;
    const char *ssid_pass;
    uint32_t st;

    Serial.println(F("Put json config file...")); // Prompt message to indicate waiting for JSON input
    st = millis(); // Store the current time for timeout handling

    // Wait for incoming JSON data over Serial within a defined timeout
    while (millis() - st < TIMEOUT_SERIAL_CONFIG) {
        if (Serial.available() > 0) { // Check if there is data available on Serial
            String comm = Serial.readStringUntil('\n'); // Read the incoming JSON string until a newline character
            Serial.println(comm); // Print the received JSON string

            // Attempt to parse the JSON string
            DeserializationError error = deserializeJson(parse_comm, comm);
            if (error) {
                Serial.print(F("Fail to parse Json file. ")); // Print error message if parsing fails
                Serial.println(error.f_str()); // Print the specific error
                break; // Exit loop if JSON parsing fails
            }

            // Check if the JSON contains the expected "device" key
            if (parse_comm.containsKey("device")) {
                ssid = parse_comm["device"]["ssid"]; // Extract SSID
                ssid_pass = parse_comm["device"]["ssid_pass"]; // Extract SSID password

                // Store the extracted values in the global configuration object
                config.ssid = ssid;
                config.ssid_pass = ssid_pass;

                // Save the updated configuration to a file or persistent storage
                save_config(DEVICE_CONFIG);
            }

            Serial.println(F("Configuration changed!")); // Confirm configuration update
            return; // Exit the function after processing the JSON data
        }

        delay(100); // Small delay to prevent excessive CPU usage
    }

    Serial.println(F("No json received. No changes.")); // Print message if no valid JSON is received within timeout
}
