#include "alt_vel.h"
#include <math.h>

#define GRAVITY      9.80665f
#define CALIB_MS     2000UL
#define CALIB_MIN_N  10
#define P_ALPHA      0.30f   // tezyiq suzgeci
#define DPDT_ALPHA   0.25f   // dP/dt suzgeci
#define A_ALPHA      0.25f   // saquli suretlendirme suzgeci
#define G_ALPHA      0.15f   // G-quvvesi suzgeci
#define Q_ALT        0.05f   // proqnoz sesi (hundurluk)
#define Q_VEL        0.6f    // proqnoz sesi (suret)
#define R_ALT        1.0f    // olcme sesi (baro)

void AltVel::init() {
    rel_alt = 0.0f;
    vel = 0.0f;
    g_force = 0.0f;
    dpdt = 0.0f;
    _p0 = 1013.25f;
    _p_smooth = 0.0f;
    _prev_p = 0.0f;
    _dpdt_smooth = 0.0f;
    _a_smooth = 0.0f;
    _g_smooth = 0.0f;
    _last_us = 0;
    _have_prev = false;
    _P00 = 1.0f;
    _P01 = 0.0f;
    _P11 = 1.0f;
    _calib_start_ms = 0;
    _calib_sum = 0.0f;
    _calib_count = 0;
    _calibrated = false;
}

void AltVel::update(float pressure_hpa, float az, float ax, float ay) {
    uint32_t now_us = micros();
    float dt = 0.0f;
    if (_last_us != 0) {
        dt = (float)(now_us - _last_us) * 1.0e-6f;
        if (dt < 0.0f) dt = 0.0f;
        if (dt > 0.5f) dt = 0.5f;
    }
    _last_us = now_us;

    // G-quvvesi (EMA ile hamarlanir)
    float g_raw = sqrtf(ax*ax + ay*ay + az*az) / GRAVITY;
    if (_g_smooth <= 0.0f) _g_smooth = g_raw;
    else _g_smooth += G_ALPHA * (g_raw - _g_smooth);
    g_force = _g_smooth;

    // Tezyiqin alcaq-tezlikli suzgeci
    if (!_have_prev) {
        _p_smooth = pressure_hpa;
        _prev_p = pressure_hpa;
        _have_prev = true;
    } else {
        _p_smooth += P_ALPHA * (pressure_hpa - _p_smooth);
    }

    // dP/dt (hPa/s)
    if (dt > 1.0e-4f) {
        float raw = (_p_smooth - _prev_p) / dt;
        _dpdt_smooth += DPDT_ALPHA * (raw - _dpdt_smooth);
    }
    _prev_p = _p_smooth;
    dpdt = _dpdt_smooth;

    // Kalibrasiya: yerdeki tezyiq ortalamasi (P0) => nisbi hundurluk 0 m
    if (!_calibrated) {
        if (_calib_start_ms == 0) _calib_start_ms = millis();
        _calib_sum += pressure_hpa;
        _calib_count++;
        rel_alt = 0.0f;
        vel = 0.0f;
        if (millis() - _calib_start_ms >= CALIB_MS && _calib_count >= CALIB_MIN_N) {
            _p0 = _calib_sum / (float)_calib_count;
            _p_smooth = _p0;
            _prev_p = _p0;
            _calibrated = true;
        }
        return;
    }

    // Barometrik nisbi hundurluk (olcme)
    float z = 0.0f;
    if (_p0 > 1.0f) {
        z = 44330.0f * (1.0f - powf(_p_smooth / _p0, 0.1903f));
    }

    // IMU-dan saquli suretlendirme (level olduqda deqiq; baro olcmesi ustundur)
    float a_vert = az - GRAVITY;
    _a_smooth += A_ALPHA * (a_vert - _a_smooth);

    // 2-dovletli Kalman proqnozu
    float alt_p = rel_alt + vel*dt + 0.5f*_a_smooth*dt*dt;
    float vel_p = vel + _a_smooth*dt;
    float P00_p = _P00 + 2.0f*dt*_P01 + dt*dt*_P11 + Q_ALT;
    float P01_p = _P01 + dt*_P11;
    float P11_p = _P11 + Q_VEL;

    // Yenileme (baro olcmesi ile)
    float S = P00_p + R_ALT;
    float K0 = P00_p / S;
    float K1 = P01_p / S;
    float innov = z - alt_p;

    rel_alt = alt_p + K0*innov;
    vel = vel_p + K1*innov;

    _P00 = (1.0f - K0) * P00_p;
    _P01 = (1.0f - K0) * P01_p;
    _P11 = P11_p - K1 * P01_p;
}
