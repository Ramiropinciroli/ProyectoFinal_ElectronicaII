#include "mqtt.h"

// Initial configuration for MQTT connection

#define MQTT_SERVER "192.168.0.193"     // MQTT broker IP address
#define MQTT_PORT 1883                  // MQTT broker port
#define MQTT_USER "proyectoelectronicaII" // MQTT username
#define MQTT_PASSWORD "proyectoelectronicaII" // MQTT password

// Create a WiFi client instance for MQTT communication
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Variables to store received MQTT topics and messages
String topic_rpc_req;
String msg_rpc_req;

// Function to set up the MQTT client with server and callback function
void setupMQTT() {
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT); // Set MQTT broker address and port
    mqttClient.setCallback(mqttCallback); // Set the callback function to handle received messages
}

// Function to reconnect to the MQTT broker if the connection is lost
void reconnectMQTT() {
    while (!mqttClient.connected()) { // Check if the client is not connected
        Serial.println("Try to connect to MQTT server"); // Print connection attempt message
        
        // Attempt to connect to the MQTT broker using the provided credentials
        if (mqttClient.connect("proyectoelectronicaII", MQTT_USER, MQTT_PASSWORD)) {
            Serial.println("Connected to MQTT server!"); // Print success message
            
            // Subscribe to the topic to receive commands from the broker
            mqttClient.subscribe("proyectoelectronicaII/commands"); 
        } else {
            delay(5000); // Wait before retrying the connection
        }
    }
}

// Callback function triggered when an MQTT message is received
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    payload[length] = '\0'; // Ensure the payload is null-terminated

    // Store the received topic and message as strings
    topic_rpc_req = String((char*)topic);
    msg_rpc_req = String((char*)payload);

    // Debug messages to print received data
    Serial.print("[DEBUG RPC] Topic received: "); Serial.println(topic_rpc_req);
    Serial.print("[DEBUG RPC] Message received: "); Serial.println(msg_rpc_req);
}

// Function to maintain the MQTT connection and process incoming messages
void handleMQTTConnection() {
    if (!mqttClient.connected()) { // Check if MQTT connection is active
        reconnectMQTT(); // Attempt to reconnect if disconnected
    }
    mqttClient.loop(); // Process incoming messages and maintain connection
}
