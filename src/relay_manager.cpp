#include "relay_manager.h"

//--Initialize the pins for the relay
void initrelay() {
    pinMode(RELAY_PIN, OUTPUT);       // Set the relay pin as an output
    digitalWrite(RELAY_PIN, LOW);     // Turn off the relay at startup
}