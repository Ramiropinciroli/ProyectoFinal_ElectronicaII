#include <Arduino.h>
#include "sensor_handler.h"
#include "display_manager.h"
#include "pin_config.h"

// Variable global para el contador de autos
int vehicle_count = 0;
bool relay_active = false;
unsigned long relay_timer_start = 0;
const unsigned long relay_duration = 120000; // 2 minutos (120,000 ms)

void setup() {
    Serial.begin(115200);

    // Inicializa los sensores ultrasónicos
    initSensors();

    // Inicializa el display OLED
    initDisplay();

    // Configurar el pin del relé como salida y apagarlo al inicio
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW); // Relé apagado inicialmente
}

void loop() {
    // Sensado de ingreso y egreso
    if (checkVehicleEntry()) {
        vehicle_count++;
        Serial.println("Vehículo ingresó");
        // Activar el relé por 2 minutos
        digitalWrite(RELAY_PIN, HIGH);
        relay_active = true;
        relay_timer_start = millis(); // Registrar el momento de activación
    }
    if (checkVehicleExit()) {
        vehicle_count = max(0, vehicle_count - 1);
        Serial.println("Vehículo salió");
    }

    // Actualiza el display con el contador actual
    updateDisplay(vehicle_count);

    // Comprobar si se deben apagar el relé después de 2 minutos
    if (relay_active && (millis() - relay_timer_start >= relay_duration)) {
        digitalWrite(RELAY_PIN, LOW);
        relay_active = false; // Desactivar el estado del relé
        Serial.println("Relé apagado después de 2 minutos");

    }

    // Pausa para evitar lecturas erráticas
    delay(500);
}
