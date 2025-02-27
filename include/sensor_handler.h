#ifndef _sensor_handler_h_
#define _sensor_handler_h_

//--Includes
#include <Arduino.h>
#include "config.h"
#include "display_manager.h"
#include "functions.h"
#include "persist.h"

#define VEHICLE_ENTRY 0
#define VEHICLE_EXIT  1

//--Functions for the initialization and control of the ultrasonic sensors
void initSensors();

//--Generalized function to check vehicle detection
bool checkVehicle(int trigPin, int echoPin, int sensorIndex);
long measureDistance(int trigPin, int echoPin);

#endif 
