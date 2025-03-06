#ifndef _persist_h_
#define _persist_h_

//--Includes
#include <Preferences.h>
#include "config.h"
#include "functions.h"

//--Variables default values
#define DEFAULT_WIFI_SSID       "Fibertel598 2.4GHz"
#define DEFAULT_WIFI_PASSWORD   "00421413349"
#define DEFAULT_MAX_DISTANCE 20

void save_config(uint8_t name);
void load_config(void);



#endif