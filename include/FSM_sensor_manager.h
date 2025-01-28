#ifndef _FSM_sensor_manager_h_
#define _FSM_sensor_manager_h_

//--Includes
#include "sensor_handler.h"
#include "display_manager.h"
#include "config.h"
#include <Arduino.h>

//--Define system states
enum SystemState { IDLE, CHECK_ENTRY, CHECK_EXIT, UPDATE_DISPLAY, RELAY_CONTROL };

//--Declare the current state variable
extern SystemState currentState;

//--Function to initialize FSM
void initFSM_sensor();

//--Function to run FSM
void FSM_sensor_run();

//--Declaration of auxiliary functions
void processEntry();
void processExit();

#endif 
