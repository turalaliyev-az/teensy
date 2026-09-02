#ifndef RF_COMMAND_H
#define RF_COMMAND_H

#include <Arduino.h>

// RFD900X (Serial2) uzrinden komanda: '1' = ARM, '0' = DISARM
void rf_command_init();
void rf_command_update();   // loop-da cagirilir, Serial2 RX oxuyur
bool rf_armed();            // son alinan veziyyet

#endif
