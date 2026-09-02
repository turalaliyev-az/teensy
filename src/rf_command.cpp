#include "rf_command.h"

static bool _armed = false;

void rf_command_init() {
    _armed = false;
}

void rf_command_update() {
    while (Serial2.available()) {
        char c = (char)Serial2.read();
        if (c == '1')      _armed = true;
        else if (c == '0') _armed = false;
        // basqa baytlar nezer alinmir
    }
}

bool rf_armed() {
    return _armed;
}
