#ifndef _functions_h_
#define _functions_h_

//--Includes
#include <Arduino.h>
#include <ArduinoJson.h>
#include "config.h"
#include "persist.h"

#define TIMEOUT_SERIAL_CONFIG   10000
#define DEVICE_CONFIG           1
#define IRR_CONFIG              2

void conf_read(void);

#endif