#include "sensor_handler.h"
#include <Arduino.h>
#include <pin_config.h>

// Definimos los tiempos de umbral para detectar vehículos
#define DETECTION_THRESHOLD 10 // Umbral en cm para considerar un objeto
#define MAX_DISTANCE 400       // Distancia máxima que el sensor puede detectar
#define NO_OBJECT -1           // Valor cuando no hay objeto

// Variables para almacenar el estado actual y anterior de los sensores
bool sensor1Active = false; // Estado del sensor de ingreso
bool sensor2Active = false; // Estado del sensor de egreso

// Inicialización de los pines de los sensores ultrasónicos
void initSensors() {
    pinMode(TRIG_PIN1, OUTPUT);
    pinMode(ECHO_PIN1, INPUT);
    pinMode(TRIG_PIN2, OUTPUT);
    pinMode(ECHO_PIN2, INPUT);
}

// Medir la distancia con los sensores ultrasónicos
long measureDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 30000); // Timeout de 30 ms
    if (duration == 0) {
        return NO_OBJECT; // Sin respuesta del sensor
    }

    long distance = (duration / 2) * 0.0343; // Calcula distancia en cm
    if (distance > MAX_DISTANCE) {
        return NO_OBJECT; // Distancia fuera del rango del sensor
    }

    return distance;
}


// Verificar si un vehículo ingresó (sensor 1)
bool checkVehicleEntry() {
    long distance = measureDistance(TRIG_PIN1, ECHO_PIN1);

    if (distance < DETECTION_THRESHOLD && distance != NO_OBJECT) {
        if (!sensor1Active) { // Detecta sólo si antes no estaba activo
            sensor1Active = true; // Marca el sensor como activo
            return true;          // Vehículo ingresó
        }
    } else {
        sensor1Active = false; // Resetea cuando el objeto se aleja
    }

    return false;
}

// Verificar si un vehículo salió (sensor 2)
bool checkVehicleExit() {
    long distance = measureDistance(TRIG_PIN2, ECHO_PIN2);

    if (distance < DETECTION_THRESHOLD && distance != NO_OBJECT) {
        if (!sensor2Active) { // Detecta sólo si antes no estaba activo
            sensor2Active = true; // Marca el sensor como activo
            return true;          // Vehículo salió
        }
    } else {
        sensor2Active = false; // Resetea cuando el objeto se aleja
    }

    return false;
}
