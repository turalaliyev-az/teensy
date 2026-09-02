#ifndef ALT_VEL_H
#define ALT_VEL_H

#include <Arduino.h>

// ======================== HUNDURLUK + SURET (IMU + BARO fusion) ========================
// Nisbi hundurluk: boot-da yer tezyiqi (P0) sifir qebul edilir.
// Barometrik tezyiq + IMU suretlendirmesi 2-dovletli Kalman ile birlesdirilir.
struct AltVel {
    float rel_alt;    // nisbi hundurluk (m, + yuxari)
    float vel;        // saquli suret (m/s, + yuxari)
    float g_force;    // G-quvvesi (g)
    float dpdt;       // tezyiq deyisme sureti (hPa/s)

    void init();
    // BME280 tezliyinde cagirilir:
    //   pressure_hpa - tezyiq (hPa), az/ax/ay - BNO055 xam suretlendirme (m/s2)
    void update(float pressure_hpa, float az_mps2, float ax_mps2, float ay_mps2);
    bool calibrated() const { return _calibrated; }

private:
    float _p0;              // yer istinad tezyiqi (hPa)
    float _p_smooth;        // alcaq-tezlikli suzulmus tezyiq
    float _prev_p;
    float _dpdt_smooth;
    float _a_smooth;        // suzulmus saquli suretlendirme
    float _g_smooth;        // suzulmus G-quvvesi
    uint32_t _last_us;
    bool _have_prev;

    float _P00, _P01, _P11; // Kalman kovaryansi

    uint32_t _calib_start_ms;
    float _calib_sum;
    uint32_t _calib_count;
    bool _calibrated;
};

#endif
