#ifndef _wifi_manager_h_
#define _wifi_manager_h_

//--Includes
#include "persist.h"
#include "functions.h"
#include <WiFi.h>

void initWiFi();
bool isWiFiConnected();
void handleWiFiConnection();

#endif 