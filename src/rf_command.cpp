#include "rf_command.h"

static bool _armed = false;
static bool _last_ack = false;   // son tesdiq olunan veziyyet

void rf_command_init() {
    _armed = false;
    _last_ack = false;
}

void rf_command_update() {
    while (Serial2.available()) {
        char c = (char)Serial2.read();
        if (c == '1')      _armed = true;
        else if (c == '0') _armed = false;
        else continue;
        // Veziyyet deyisende RF ile geriye tesdiq gonder: '1' (ARM) / '0' (DISARM)
        if (_armed != _last_ack) {
            _last_ack = _armed;
            Serial2.print(_armed ? '1' : '0');
            Serial2.print('\n');
        }
    }
}

bool rf_armed() {
    return _armed;
}
