#ifndef _display_manager_h_
#define _display_manager_h_

//--Includes
#include <U8g2lib.h>
#include <Wire.h>
#include "config.h"

void initDisplay();
void updateDisplay(int vehicleCount);

#endif 
