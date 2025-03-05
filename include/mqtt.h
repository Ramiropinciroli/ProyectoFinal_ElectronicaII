#ifndef MQTT_H
#define MQTT_H


#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>


extern WiFiClient espClient;
extern PubSubClient mqttClient;


void setupMQTT();
void reconnectMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);


#endif