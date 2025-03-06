#include "publish.h"

// Function to publish the vehicle count to the MQTT broker
void publishvehicle_count(int vehicle_count) {
    // Convert the integer vehicle count to a string
    String payload = String(vehicle_count);

    // Publish the payload to the specified MQTT topic
    if (mqttClient.publish("proyectoelectronicaII/vehicleCount", payload.c_str())) {
        Serial.println("Publish successful"); // Confirm successful publishing
    } else {
        Serial.println("Publish not successful"); // Notify if publishing fails
    }
}
