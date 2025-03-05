#include "publish.h"

void publishvehicle_count(int vehicle_count) {
    String payload = String(vehicle_count);
    if (mqttClient.publish("proyectoelectronicaII/vehicleCount", payload.c_str())){
        Serial.println("Publish successfull");
    }else{
        Serial.println("Publish not successfull");
    }
}