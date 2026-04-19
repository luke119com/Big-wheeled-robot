#ifndef PS2_H
#define PS2_H

#include "Arduino.h"
#include <PS2X_lib.h>  
#include "robot.h"
#include "WS2812.h"

#define PS2_DAT        5  //MISO  19
#define PS2_CMD        37  //MOSI  23
#define PS2_SEL        38  //SS     5
#define PS2_CLK        39  //SLK   18

#define pressures   false
#define rumble      false


void mapPs2ToRobotControl();
void ps2Init();
void PS2_switch();
bool ps2IsConnected();


#endif
