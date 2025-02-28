#ifndef _persist_h_
#define _persist_h_

#include <Preferences.h>
#include "config.h"
#include "functions.h"

//--Variables default values
#define DEFAULT_WIFI_SSID       "Wi-Fi 8 A"
#define DEFAULT_WIFI_PASSWORD   "ramiguille2024"
#define DEFAULT_MAX_DISTANCE 20

void save_config(uint8_t name);
void load_config(void);



#endif