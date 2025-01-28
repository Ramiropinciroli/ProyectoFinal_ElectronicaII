#ifndef _config_h_
#define _config_h_

//--Includes
#include <Arduino.h>

//--Ultrasonic sensors
#define TRIG_PIN2 26  //--Exit sensor (TRIG)
#define ECHO_PIN2 25 //--Exit sensor (ECHO)
#define TRIG_PIN1 19 //--Entry sensor (TRIG)
#define ECHO_PIN1 18 //--Entry sensor (ECHO)

//--Relay (light control)
#define RELAY_PIN 17

//--OLED Display (I2C)
#define SDA_PIN 21 //--Pin SDA
#define SCL_PIN 22 //--Pin SCL

//--Hardware PINS definitions
#define ONBOARD_LED_PIN             2

//--Software constants
#define SERIAL_BAUDRATE             115200

//--Structures
struct Config {
  String ssid;
  String ssid_pass;
  uint8_t irr_hour;
  uint8_t irr_minute;
  uint8_t irr_time;
};

#endif 