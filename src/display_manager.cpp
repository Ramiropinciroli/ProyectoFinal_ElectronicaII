#include "display_manager.h"

//--Initialize the U8g2 object for SSD1306 displays in I2C mode
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

void initDisplay() {
    u8g2.begin(); //--Initialize the display
    u8g2.clearBuffer(); //--Clear the buffer
    u8g2.setFont(u8g2_font_t0_11b_tf); //--Set the font
    u8g2.drawStr(22, 20, "Sistema iniciado"); //--Initial message
    u8g2.sendBuffer(); //--Send the buffer to the display
    delay(2000); //--Pause to visualize
    u8g2.clearBuffer();
}

void updateDisplay(int vehicleCount) {
    u8g2.clearBuffer(); //--Clear the previous content
    u8g2.setFont(u8g2_font_t0_11b_tf); //--Set the font

    //--Write text in the screen
    u8g2.drawStr(18, 15, "Autos en cochera:");
    u8g2.setCursor(60, 40);
    u8g2.print(vehicleCount); //--Display number of cars

    u8g2.sendBuffer(); //--Update screen
}
