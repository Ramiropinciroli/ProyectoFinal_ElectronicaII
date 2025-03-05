#include "mqtt.h"
#include <WiFi.h>

// set of the initial configuration for mqtt

#define MQTT_SERVER "192.168.0.193"     
#define MQTT_PORT 1883
#define MQTT_USER "proyectoelectronicaII"
#define MQTT_PASSWORD "proyectoelectronicaII"


WiFiClient espClient;
PubSubClient mqttClient(espClient);

String topic_rpc_req;
String msg_rpc_req;

void setupMQTT() {
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    mqttClient.setCallback(mqttCallback);
}


void reconnectMQTT() {
    
    while (!mqttClient.connected()) {
        Serial.println("Try to connect to MQTT server");
        if (mqttClient.connect("proyectoelectronicaII", MQTT_USER, MQTT_PASSWORD)) {
            Serial.println("Connected to MQTT server!");
            mqttClient.subscribe("proyectoelectronicaII/commands");
        } else {
            delay(5000);
        }
    }
}


void mqttCallback(char* topic, byte* payload, unsigned int length) {
    payload[length] = '\0';
    topic_rpc_req = String((char*)topic);
    msg_rpc_req = String((char*)payload);
    //--Debug message
    Serial.print("[DEBUG RPC] Topico de pregunta:");Serial.println(topic_rpc_req);
    Serial.print("[DEBUG RPC] Mensaje de pregunta:");Serial.println(msg_rpc_req);
}

void handleMQTTConnection() {
    if (!mqttClient.connected()) {
        reconnectMQTT();
    }
    mqttClient.loop(); 
}