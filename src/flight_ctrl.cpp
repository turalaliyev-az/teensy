#include "flight_ctrl.h"
#include "esc.h"

#define ALT_TRIGGER_M   500.0f
#define RAMP_MS         500UL
#define RAMP_STEP_US    ((float)(ESC_US_RUN - ESC_US_OFF) / (float)RAMP_MS)  // us/ms

void FlightCtrl::init() {
    state = FS_DISARMED;
    throttle_us = (float)ESC_US_OFF;
    _last_ms = 0;
}

void FlightCtrl::update(bool armed, bool level_ok, bool descending, float rel_alt, uint32_t now_ms) {
    uint32_t dt_ms = 0;
    if (_last_ms != 0) {
        dt_ms = now_ms - _last_ms;
        if (dt_ms > 100) dt_ms = 100;   // clamp
    }
    _last_ms = now_ms;

    // DISARM her zaman ustundur: derhal 1000 us
    if (!armed) {
        state = FS_DISARMED;
        throttle_us = (float)ESC_US_OFF;
        esc_write_us((uint16_t)throttle_us);
        return;
    }

    switch (state) {
    case FS_DISARMED:
        // ARM alindi -> gozleme veziyyetine kec (motor hele 1000 us)
        throttle_us = (float)ESC_US_OFF;
        state = FS_ARMED;
        break;

    case FS_ARMED:
        // Aktivlesme: level && enir && alt_rel <= 500 m
        if (level_ok && descending && rel_alt <= ALT_TRIGGER_M) {
            state = FS_MOTORS_ON;
        }
        break;

    case FS_MOTORS_ON:
        // tilt > +-5 -> qaz donur (hold); level qayidanda ramp davam edir
        if (!level_ok) break;
        if (throttle_us < (float)ESC_US_RUN) {
            throttle_us += RAMP_STEP_US * (float)dt_ms;
            if (throttle_us > (float)ESC_US_RUN) throttle_us = (float)ESC_US_RUN;
        }
        break;
    }

    esc_write_us((uint16_t)throttle_us);
}
