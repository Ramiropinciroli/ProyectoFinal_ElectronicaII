#ifndef _persist_h_
#define _persist_h_

#include <Preferences.h>
#include "config.h"
#include "functions.h"

//--Variables default values
#define DEFAULT_IRR_HOUR    22
#define DEFAULT_IRR_MINUTE  00
#define DEFAULT_IRR_TIME    10
#define WIFI_SSID        "Fibertel598 2.4GHz"
#define WIFI_PASSWORD   "00421413349"

void save_config(uint8_t name);
void load_config(void);



#endif