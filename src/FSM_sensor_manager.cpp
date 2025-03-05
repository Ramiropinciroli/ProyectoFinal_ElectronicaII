#include "FSM_sensor_manager.h"
#include "publish.h"

//--Global variable for current state
SystemState currentState = IDLE;

//--System related variables
int vehicle_count = 0;
bool relay_active = false;
unsigned long relay_timer_start = 0;
const long interval = 500;        //--Interval between sensor readings (in ms)
#define RELAY_DURATION 120000    //--Relay duration in milliseconds 

//--Initialize FSM
void initFSM_sensor() {
    currentState = IDLE;
}

//--Run FSM
void FSM_sensor_run() {
    unsigned long currentMillis = millis();
    static unsigned long lastStateChange = 0;

    switch (currentState) {
        case IDLE:
            if (currentMillis - lastStateChange >= interval) {
                currentState = CHECK_ENTRY;
                lastStateChange = currentMillis;
            }
            break;

        case CHECK_ENTRY:
            if (checkVehicle(TRIG_PIN1, ECHO_PIN1, VEHICLE_ENTRY)) {
                processEntry();
            }
            currentState = CHECK_EXIT;
            break;

        case CHECK_EXIT:
            if (checkVehicle(TRIG_PIN2, ECHO_PIN2, VEHICLE_EXIT)) {
                processExit();
            }
            currentState = UPDATE_DISPLAY;
            break;

        case UPDATE_DISPLAY:
            updateDisplay(vehicle_count);
            currentState = RELAY_CONTROL;
            break;

        case RELAY_CONTROL:
            if (relay_active && (millis() - relay_timer_start >= RELAY_DURATION)) {
                digitalWrite(RELAY_PIN, LOW);
                relay_active = false;
                Serial.println("Relay turned off after 2 minutes");
            }
            currentState = IDLE;
            break;
    }
}

//--Process vehicle entry
void processEntry() {
    vehicle_count++;
    Serial.println("Vehicle entered");
    digitalWrite(RELAY_PIN, HIGH);
    relay_active = true;
    relay_timer_start = millis();

    //--Publish updated vehicle count
    publishvehicle_count(vehicle_count);
}

//--Process vehicle exit
void processExit() {
    vehicle_count = max(0, vehicle_count - 1);
    Serial.println("Vehicle exited");

    //--Publish updated vehicle count
    publishvehicle_count(vehicle_count);
}
