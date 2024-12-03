#ifndef SENSOR_HANDLER_H
#define SENSOR_HANDLER_H

// Funciones para la inicialización y control de los sensores ultrasónicos
void initSensors();
bool checkVehicleEntry();
bool checkVehicleExit();
long measureDistance(int trigPin, int echoPin);

#endif // SENSOR_HANDLER_H
