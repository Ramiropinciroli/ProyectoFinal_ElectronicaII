#include <U8g2lib.h>
#include <Wire.h>
#include "pin_config.h"

// Inicializa el objeto U8g2 para pantallas SSD1306 en modo I2C
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

void initDisplay() {
    u8g2.begin(); // Inicializa el display
    u8g2.clearBuffer(); // Limpia el buffer
    u8g2.setFont(u8g2_font_t0_11b_tf); // Configura la fuente
    u8g2.drawStr(22, 20, "Sistema iniciado"); // Mensaje inicial
    u8g2.sendBuffer(); // Envía el buffer al display
    delay(2000); // Pausa para visualizar
    u8g2.clearBuffer();
}

void updateDisplay(int vehicleCount) {
    u8g2.clearBuffer(); // Limpia el contenido previo
    u8g2.setFont(u8g2_font_t0_11b_tf); // Configura la fuente

    // Escribe el texto en la pantalla
    u8g2.drawStr(18, 15, "Autos en cochera:");
    u8g2.setCursor(60, 40);
    u8g2.print(vehicleCount); // Muestra la cantidad de autos

    u8g2.sendBuffer(); // Actualiza la pantalla
}

