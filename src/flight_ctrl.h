#ifndef FLIGHT_CTRL_H
#define FLIGHT_CTRL_H

#include <Arduino.h>

enum FlightState : uint8_t {
    FS_DISARMED = 0,
    FS_ARMED = 1,
    FS_MOTORS_ON = 2
};

struct FlightCtrl {
    FlightState state;
    float throttle_us;   // cari ESC impulsu (us)

    void init();
    // armed: RF-den. level_ok: |roll|<=5 && |pitch|<=5. descending: vel<0.
    void update(bool armed, bool level_ok, bool descending, float rel_alt, uint32_t now_ms);
    uint16_t throttle() const { return (uint16_t)throttle_us; }
    uint8_t state_code() const { return (uint8_t)state; }

private:
    uint32_t _last_ms;
};

#endif
