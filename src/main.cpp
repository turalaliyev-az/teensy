/**
 * @file main.cpp — Drone (CC)
 *
 * Teensy 4.1 (Cortex-M7, 600MHz, Hardware FPU) — 0 XARICI KITABXANA
 *
 * Sensorlar:   BNO055(0x28) + BME280(0x76) + AHT20(0x38) + GPS(Serial7)
 * RF:          Serial2 (TX=7,RX=8) @ 115200 baud, 15 Hz CSV + komanda RX
 *
 * Filtrler:
 *   1D Kalman     — BME280 temperatur + yukseklik
 *   Attitude EKF  — 7-dovletli quaternion (gyro+accel, yaw-suz)
 *   BME280 IIR x16 — daxili hardware filtr
 *
 * Paket formati (15 Hz):
 *   CC,ts,AX,AY,AZ,GX,GY,GZ,MX,MY,MZ,T,P,H,Alt,aT,aH,lat,lon,gps_alt,spd,crs,roll,pitch,yaw,rel_alt,vel,g,dpdt,arm,state,pwm\n
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>




// ======================== ESC (50 Hz PWM — DShot DEYIL) ========================
#define ESC1_PIN      15      // QuadTimer3_3
#define ESC2_PIN      23      // FlexPWM4_1_A
#define ESC_PWM_FREQ  50.0f
#define ESC_US_MIN    1000
#define ESC_US_MAX    2000
#define ESC_US_OFF    1000    // disarm / sifir qaz
#define ESC_US_RUN    1480    // is (≈48% qaz)

// ESC-leri 1000us ile ise salir ve PWM pinlerini hazirlayir.
void esc_init();

// Her iki ESC-ye eyni deyeri yollar (us, 1000..2000).
void esc_write_us(uint16_t us);

// ESC-lere ayri-ayri deyer yollar.
void esc_write_us(uint16_t us1, uint16_t us2);



static uint32_t us_to_duty(uint16_t us) {
    if (us < ESC_US_MIN) us = ESC_US_MIN;
    if (us > ESC_US_MAX) us = ESC_US_MAX;
    // 0..65535 (16-bit) => 0..20000 us
    return (uint32_t)us * 65536UL / 20000UL;
}

void esc_init() {
    pinMode(ESC1_PIN, OUTPUT);
    pinMode(ESC2_PIN, OUTPUT);
    analogWriteFrequency(ESC1_PIN, ESC_PWM_FREQ);
    analogWriteFrequency(ESC2_PIN, ESC_PWM_FREQ);
    analogWriteResolution(16);
    esc_write_us(ESC_US_OFF);   // tehlukesiz: boot-da 1000 us
}

void esc_write_us(uint16_t us1, uint16_t us2) {
    analogWrite(ESC1_PIN, us_to_duty(us1));
    analogWrite(ESC2_PIN, us_to_duty(us2));
}

void esc_write_us(uint16_t us) {
    esc_write_us(us, us);
}


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



// ======================== 7-DOVLETLI QUATERNION EKF ========================
// Veziyyet: quaternion q[4] (w,x,y,z) + gyro bias b[3] (rad/s) = 7 dovlet.
// Proqnoz: gyro (bias korreksiyasi ile).
// Olcme: akseleorometr qravitasiya vektoru (yalniz roll/pitch).
// Yaw mushahide olunmur (maqnitometr yox) -> drift edir, amma istifade olunmur.
// Vibrasiyaya qarsi adaptiv R: |a| 1g-den kenarlasdıqca olcme sesi artır.
struct AttitudeEKF {
    float q[4];     // quaternion (q0=w, q1=x, q2=y, q3=z)
    float b[3];     // gyro bias (rad/s)
    float P[7][7];  // kovarians matrisi

    void init(float ax, float ay, float az);
    void predict(float gx, float gy, float gz, float dt);
    void update(float ax, float ay, float az);
    void getEulerDeg(float &roll, float &pitch, float &yaw) const;
};



#define RAD2DEG 57.29577951308232f

// ---- EKF tuning ----
#define GYRO_NOISE  0.02f    // gyro sesi std (rad/s)
#define BIAS_NOISE  0.0005f  // gyro bias random walk (rad/s^2 * sqrt(s))
#define R_BASE      0.003f   // esas akseleorometr olcme sesi (normallasdirilmis)
#define R_ADAPT     2.0f     // vibrasiya adaptasiya emsali
#define ACC_MIN     3.0f     // |a| < 3 m/s2 -> free-fall, olcme kecilir
#define ACC_MAX     25.0f    // |a| > 25 m/s2 -> anormal, olcme kecilir

static void quat_norm(float q[4]) {
    float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n < 1e-8f) { q[0]=1.0f; q[1]=q[2]=q[3]=0.0f; return; }
    n = 1.0f / n;
    q[0]*=n; q[1]*=n; q[2]*=n; q[3]*=n;
}

void AttitudeEKF::init(float ax, float ay, float az) {
    // Qisa qovs quaternion: akseleorometr (body "up") -> dunya "up" [0,0,1]
    float n = sqrtf(ax*ax + ay*ay + az*az);
    if (n < 1e-4f) n = 1.0f;
    ax /= n; ay /= n; az /= n;

    float axis_x = ay, axis_y = -ax, axis_z = 0.0f; // cross(a, [0,0,1])
    float d = az;                                    // dot(a, [0,0,1])
    if (d > 0.999999f) {
        q[0]=1.0f; q[1]=q[2]=q[3]=0.0f;
    } else if (d < -0.999999f) {
        q[0]=0.0f; q[1]=1.0f; q[2]=q[3]=0.0f;        // 180°, ixtiyari ox
    } else {
        float ang = acosf(d);
        float s = sinf(ang * 0.5f);
        float an = sqrtf(axis_x*axis_x + axis_y*axis_y);
        if (an < 1e-8f) an = 1.0f;
        q[0] = cosf(ang * 0.5f);
        q[1] = s * axis_x / an;
        q[2] = s * axis_y / an;
        q[3] = s * axis_z / an;
    }
    b[0] = b[1] = b[2] = 0.0f;

    for (int i=0;i<7;i++) for (int j=0;j<7;j++) P[i][j] = 0.0f;
    P[0][0]=P[1][1]=P[2][2]=P[3][3]=0.1f;   // quaternion qeyri-mueyyenliyi
    P[4][4]=P[5][5]=P[6][6]=0.1f;           // bias (rad/s)^2
}

void AttitudeEKF::predict(float gx, float gy, float gz, float dt) {
    float wx = gx - b[0], wy = gy - b[1], wz = gz - b[2];

    // ---- veziyyet proqnozu: dq/dt = 0.5 * Omega(w) * q ----
    float qd[4];
    qd[0] = 0.5f * (-wx*q[1] - wy*q[2] - wz*q[3]);
    qd[1] = 0.5f * ( wx*q[0] + wz*q[2] - wy*q[3]);
    qd[2] = 0.5f * ( wy*q[0] - wz*q[1] + wx*q[3]);
    qd[3] = 0.5f * ( wz*q[0] + wy*q[1] - wx*q[2]);
    q[0] += qd[0]*dt; q[1] += qd[1]*dt; q[2] += qd[2]*dt; q[3] += qd[3]*dt;
    quat_norm(q);

    // ---- F = [ Fqq  Fqb ; 0  I3 ] ----
    float F[7][7];
    for (int i=0;i<7;i++) for (int j=0;j<7;j++) F[i][j] = 0.0f;
    // Fqq = I + 0.5*Omega(w)*dt
    F[0][0]=1.0f;             F[0][1]=-0.5f*wx*dt; F[0][2]=-0.5f*wy*dt; F[0][3]=-0.5f*wz*dt;
    F[1][0]= 0.5f*wx*dt;      F[1][1]=1.0f;        F[1][2]= 0.5f*wz*dt; F[1][3]=-0.5f*wy*dt;
    F[2][0]= 0.5f*wy*dt;      F[2][1]=-0.5f*wz*dt; F[2][2]=1.0f;        F[2][3]= 0.5f*wx*dt;
    F[3][0]= 0.5f*wz*dt;      F[3][1]= 0.5f*wy*dt; F[3][2]=-0.5f*wx*dt; F[3][3]=1.0f;
    // Fqb = -0.5 * Xi(q) * dt
    F[0][4]= 0.5f*q[1]*dt; F[0][5]= 0.5f*q[2]*dt; F[0][6]= 0.5f*q[3]*dt;
    F[1][4]=-0.5f*q[0]*dt; F[1][5]= 0.5f*q[3]*dt; F[1][6]=-0.5f*q[2]*dt;
    F[2][4]=-0.5f*q[3]*dt; F[2][5]=-0.5f*q[0]*dt; F[2][6]= 0.5f*q[1]*dt;
    F[3][4]= 0.5f*q[2]*dt; F[3][5]=-0.5f*q[1]*dt; F[3][6]=-0.5f*q[0]*dt;
    F[4][4]=F[5][5]=F[6][6]=1.0f;

    // ---- Q = [ Qqq 0 ; 0 Qbb ] ----
    float Q[7][7];
    for (int i=0;i<7;i++) for (int j=0;j<7;j++) Q[i][j] = 0.0f;
    // Xi(q) 4x3
    float Xi[4][3] = {
        {-q[1], -q[2], -q[3]},
        { q[0], -q[3],  q[2]},
        { q[3],  q[0], -q[1]},
        {-q[2],  q[1],  q[0]}
    };
    float qg = GYRO_NOISE * GYRO_NOISE;
    float s = 0.25f * qg * dt * dt;
    for (int i=0;i<4;i++) {
        for (int j=0;j<4;j++) {
            float sum = 0.0f;
            for (int k=0;k<3;k++) sum += Xi[i][k] * Xi[j][k];
            Q[i][j] = s * sum;
        }
    }
    Q[0][0]+=1e-9f; Q[1][1]+=1e-9f; Q[2][2]+=1e-9f; Q[3][3]+=1e-9f;
    float qb = BIAS_NOISE * BIAS_NOISE * dt;
    Q[4][4]=Q[5][5]=Q[6][6]=qb;

    // ---- P = F P F^T + Q ----
    float FP[7][7];
    for (int i=0;i<7;i++)
        for (int j=0;j<7;j++) {
            float sum=0.0f;
            for (int k=0;k<7;k++) sum += F[i][k] * P[k][j];
            FP[i][j] = sum;
        }
    for (int i=0;i<7;i++)
        for (int j=0;j<7;j++) {
            float sum=0.0f;
            for (int k=0;k<7;k++) sum += FP[i][k] * F[j][k]; // F^T
            P[i][j] = sum + Q[i][j];
        }
}

void AttitudeEKF::update(float ax, float ay, float az) {
    float amag = sqrtf(ax*ax + ay*ay + az*az);
    // Free-fall / anormal tecilde olcme etibarsizdir -> kec
    if (amag < ACC_MIN || amag > ACC_MAX) return;
    float n = 1.0f / amag;
    ax *= n; ay *= n; az *= n;

    // ---- proqnoz qravitasiya (body) ----
    float h0 = 2.0f * (q[1]*q[3] - q[0]*q[2]);
    float h1 = 2.0f * (q[2]*q[3] + q[0]*q[1]);
    float h2 = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

    float y0 = ax - h0;
    float y1 = ay - h1;
    float y2 = az - h2;

    // ---- H (3x7) ----
    float H[3][7];
    for (int j=0;j<7;j++) H[0][j]=H[1][j]=H[2][j]=0.0f;
    H[0][0]=-2.0f*q[2]; H[0][1]= 2.0f*q[3]; H[0][2]=-2.0f*q[0]; H[0][3]= 2.0f*q[1];
    H[1][0]= 2.0f*q[1]; H[1][1]= 2.0f*q[0]; H[1][2]= 2.0f*q[3]; H[1][3]= 2.0f*q[2];
    H[2][0]= 2.0f*q[0]; H[2][1]=-2.0f*q[1]; H[2][2]=-2.0f*q[2]; H[2][3]= 2.0f*q[3];

    // ---- adaptiv R: |a| 1g-den kenarlasdıqca artır ----
    float dev = fabsf(amag - 9.80665f) / 9.80665f;
    float r = R_BASE + R_ADAPT * dev * dev;

    // ---- S = H P H^T + R*I ----
    float PHt[7][3];
    for (int i=0;i<7;i++)
        for (int j=0;j<3;j++) {
            float sum=0.0f;
            for (int k=0;k<7;k++) sum += P[i][k] * H[j][k];
            PHt[i][j] = sum;
        }
    float S[3][3];
    for (int i=0;i<3;i++)
        for (int j=0;j<3;j++) {
            float sum=0.0f;
            for (int k=0;k<7;k++) sum += H[i][k] * PHt[k][j];
            S[i][j] = sum;
        }
    S[0][0]+=r; S[1][1]+=r; S[2][2]+=r;

    // ---- S^-1 (3x3) ----
    float det = S[0][0]*(S[1][1]*S[2][2]-S[1][2]*S[2][1])
              - S[0][1]*(S[1][0]*S[2][2]-S[1][2]*S[2][0])
              + S[0][2]*(S[1][0]*S[2][1]-S[1][1]*S[2][0]);
    if (fabsf(det) < 1e-12f) return;
    float id = 1.0f / det;
    float Si[3][3];
    Si[0][0]=(S[1][1]*S[2][2]-S[1][2]*S[2][1])*id;
    Si[0][1]=(S[0][2]*S[2][1]-S[0][1]*S[2][2])*id;
    Si[0][2]=(S[0][1]*S[1][2]-S[0][2]*S[1][1])*id;
    Si[1][0]=(S[1][2]*S[2][0]-S[1][0]*S[2][2])*id;
    Si[1][1]=(S[0][0]*S[2][2]-S[0][2]*S[2][0])*id;
    Si[1][2]=(S[0][2]*S[1][0]-S[0][0]*S[1][2])*id;
    Si[2][0]=(S[1][0]*S[2][1]-S[1][1]*S[2][0])*id;
    Si[2][1]=(S[0][1]*S[2][0]-S[0][0]*S[2][1])*id;
    Si[2][2]=(S[0][0]*S[1][1]-S[0][1]*S[1][0])*id;

    // ---- K = P H^T S^-1 (7x3) ----
    float K[7][3];
    for (int i=0;i<7;i++)
        for (int j=0;j<3;j++) {
            float sum=0.0f;
            for (int k=0;k<3;k++) sum += PHt[i][k] * Si[k][j];
            K[i][j] = sum;
        }

    // ---- veziyyet yenileme: dx = K y ----
    q[0] += K[0][0]*y0 + K[0][1]*y1 + K[0][2]*y2;
    q[1] += K[1][0]*y0 + K[1][1]*y1 + K[1][2]*y2;
    q[2] += K[2][0]*y0 + K[2][1]*y1 + K[2][2]*y2;
    q[3] += K[3][0]*y0 + K[3][1]*y1 + K[3][2]*y2;
    quat_norm(q);
    b[0] += K[4][0]*y0 + K[4][1]*y1 + K[4][2]*y2;
    b[1] += K[5][0]*y0 + K[5][1]*y1 + K[5][2]*y2;
    b[2] += K[6][0]*y0 + K[6][1]*y1 + K[6][2]*y2;

    // ---- P = (I - K H) P ----
    float KH[7][7];
    for (int i=0;i<7;i++)
        for (int j=0;j<7;j++) {
            float sum=0.0f;
            for (int k=0;k<3;k++) sum += K[i][k] * H[k][j];
            KH[i][j] = sum;
        }
    float Pnew[7][7];
    for (int i=0;i<7;i++)
        for (int j=0;j<7;j++) {
            float sum=0.0f;
            for (int k=0;k<7;k++) sum += ((i==k?1.0f:0.0f) - KH[i][k]) * P[k][j];
            Pnew[i][j] = sum;
        }
    for (int i=0;i<7;i++) for (int j=0;j<7;j++) P[i][j] = Pnew[i][j];
}

void AttitudeEKF::getEulerDeg(float &roll, float &pitch, float &yaw) const {
    roll  = atan2f(2.0f*(q[0]*q[1] + q[2]*q[3]), 1.0f - 2.0f*(q[1]*q[1] + q[2]*q[2])) * RAD2DEG;
    pitch = asinf(2.0f*(q[0]*q[2] - q[3]*q[1])) * RAD2DEG;
    yaw   = atan2f(2.0f*(q[0]*q[3] + q[1]*q[2]), 1.0f - 2.0f*(q[2]*q[2] + q[3]*q[3])) * RAD2DEG;
}


// ======================== CIHAZ BASLIGI VE ADI ========================
#define DEVICE_HEADER F("CC")   // Drone
#define DEVICE_NAME   "DRONE (CC)"

// ======================== PIN ========================
#define LED_PIN         13
#define RF_SERIAL       Serial2       // RF TX=7, RX=8
#define RF_BAUD         115200
#define GPS_SERIAL      Serial7       // GPS RX=9, TX=10
#define GPS_BAUD        9600
#define I2C_FREQ        400000UL

// ======================== SENSOR UNVANLARI ========================
#define BNO055_ADDR     0x28
#define BME280_ADDR     0x76
#define AHT20_ADDR      0x38

// ======================== KADENSLER ========================
#define BNO055_PERIOD   10    // 100 Hz
#define BME280_PERIOD   40    // 25 Hz
#define AHT20_PERIOD    1000  // 1 Hz
#define GPS_PERIOD      200   // 5 Hz
#define PRINT_PERIOD    200   // USB 5 Hz
#define RF_PERIOD       66    // RF 15 Hz
#define FLIGHT_PERIOD   10    // 100 Hz ucush nezareti (yalniz CC)
#define I2C_DIAG_PERIOD 5000  // 5 s I2C diaqnostika

// ======================== BNO055 REGISTERLERI ========================
#define BNO055_CHIP_ID      0x00
#define BNO055_OPR_MODE     0x3D
#define BNO055_PWR_MODE     0x3E
#define BNO055_SYS_TRIG     0x3F
#define BNO055_UNIT_SEL     0x3B
#define BNO055_CALIB_STAT   0x35
#define BNO055_PAGE_ID      0x07
#define BNO055_ACC_START    0x08
#define BNO055_MAG_START    0x0E
#define BNO055_GYRO_START   0x14

// ======================== BME280 REGISTER ========================
#define BME280_CHIP_ID      0xD0
#define BME280_CTRL_HUM     0xF2
#define BME280_CTRL_MEAS    0xF4
#define BME280_CONFIG       0xF5
#define BME280_DATA_START   0xF7
#define BME280_CALIB_START  0x88

// ======================== AHT20 ========================
#define AHT20_CMD_INIT      0xBE
#define AHT20_CMD_TRIG      0xAC
#define AHT20_STAT_CAL      0x08
#define AHT20_STAT_BUSY     0x80

// ======================== SABIT CEVRIME EMSALLARI ========================
#define BNO055_ACC_SCALE        0.01f
#define BNO055_GYRO_SCALE       0.00106526354f
#define BNO055_MAG_SCALE        0.0625f
#define BME280_TEMP_SCALE       0.01f
#define BME280_PRESS_SCALE      0.0000390625f
#define BME280_HUM_SCALE        0.0009765625f
#define SEA_LEVEL_HPA           1013.25f

// ======================== NMEA -> DECIMAL DERECE ========================
static float nmea_to_decimal(float ddmm) {
    int deg = (int)(ddmm / 100.0f);
    float min = ddmm - (float)(deg * 100);
    return (float)deg + min / 60.0f;
}

// ======================== GPS DATA ========================
struct GPSData {
    float lat, lon, altitude, speed, course;
    uint8_t fix, satellites;
    bool updated;
    GPSData() : lat(0),lon(0),altitude(0),speed(0),course(0),fix(0),satellites(0),updated(false) {}
};
static GPSData gps;
static char gps_buf[128];
static uint8_t gps_idx = 0;

// ======================== ATTITUDE EKF (7-DOVLETLI) ========================
// attitude_ekf.h/.cpp modulundadir — Madgwick evezine tam Kalman (yaw-suz)

// ======================== KALMAN ========================
struct Kalman1D {
    float Q,R,P,K,X;
    void init(float q,float r,float x0){Q=q;R=r;P=1;K=0;X=x0;}
    float update(float z){P+=Q;K=P/(P+R);X+=K*(z-X);P=(1-K)*P;return X;}
};

// ======================== BME280 KALIBRASIYA ========================
static uint16_t dig_T1,dig_P1;
static int16_t dig_T2,dig_T3,dig_P2,dig_P3,dig_P4,dig_P5,dig_P6,dig_P7,dig_P8,dig_P9;
static uint8_t dig_H1,dig_H3; static int16_t dig_H2,dig_H4,dig_H5; static int8_t dig_H6;
static int32_t t_fine;

// ======================== GLOBAL STATUS ========================
static struct{uint8_t bno055:1,bme280:1,aht20:1,gps_fix:1;} ok;
static AttitudeEKF ekf;
static Kalman1D kalmanTemp,kalmanAlt;
static bool kalman_ready=false,aht_triggered=false;
static uint32_t aht_trigger_ms=0,lastBno,lastBme,lastAht,lastGps,lastPrn,lastRf,lastI2cDiag;
static uint32_t lastFlight;
static AltVel altvel;
static FlightCtrl flight;

// Sensor data buferi
static float ax,ay,az,gx,gy,gz,mx,my,mz;
static float bme_t,bme_p,bme_h,bme_a,bme_tk,bme_ak;
static float aht_t,aht_h;
static float mad_roll,mad_pitch,mad_yaw;

// ======================== I2C INLINE ========================
static inline void w8(uint8_t a,uint8_t r,uint8_t v){Wire.beginTransmission(a);Wire.write(r);Wire.write(v);Wire.endTransmission();}
static inline uint8_t r8(uint8_t a,uint8_t r){Wire.beginTransmission(a);Wire.write(r);Wire.endTransmission(false);Wire.requestFrom(a,(uint8_t)1);return Wire.read();}
static inline void rBuf(uint8_t a,uint8_t r,uint8_t*b,uint8_t n){Wire.beginTransmission(a);Wire.write(r);Wire.endTransmission(false);Wire.requestFrom(a,n);for(uint8_t i=0;i<n;i++)b[i]=Wire.read();}

// ======================== GPS NMEA PARSER (GPGGA + GPRMC) ========================
static void gps_parse_gpgga(char*s){
    char*p=s;
    for(int i=0;i<1;i++){p=strchr(p,',');if(!p)return;p++;}
    float lat_raw=strtof(p,&p);if(!p||*p!=',')return;p++;
    if(*p=='S')lat_raw=-lat_raw;
    p=strchr(p,',');if(!p)return;p++;
    float lon_raw=strtof(p,&p);if(!p||*p!=',')return;p++;
    if(*p=='W')lon_raw=-lon_raw;
    p=strchr(p,',');if(!p)return;p++;
    int fix=(int)strtol(p,&p,10);if(!p||*p!=',')return;p++;
    int sats=(int)strtol(p,&p,10);if(!p)return;
    for(int i=0;i<2;i++){p=strchr(p,',');if(!p)return;p++;}
    float alt=strtof(p,&p);
    gps.lat=nmea_to_decimal(fabsf(lat_raw))*(lat_raw<0?-1:1);
    gps.lon=nmea_to_decimal(fabsf(lon_raw))*(lon_raw<0?-1:1);
    gps.altitude=alt;gps.fix=(uint8_t)fix;gps.satellites=(uint8_t)sats;gps.updated=true;
}
static void gps_parse_gprmc(char*s){
    char*p=s;
    for(int i=0;i<1;i++){p=strchr(p,',');if(!p)return;p++;}
    p=strchr(p,',');if(!p)return;p++;if(*p!='A')return;
    for(int i=0;i<4;i++){p=strchr(p,',');if(!p)return;p++;}
    float spd=strtof(p,&p);if(!p||*p!=',')return;p++;
    float crs=strtof(p,&p);
    gps.speed=spd*0.514444f;gps.course=crs;
}
static void gps_read(){
    while(GPS_SERIAL.available()){
        char c=GPS_SERIAL.read();
        if(c=='$'){gps_idx=0;gps_buf[0]='$';gps_buf[1]=0;}
        else if(c=='\n'){gps_buf[gps_idx]=0;if(strncmp(gps_buf,"$GPGGA",6)==0)gps_parse_gpgga(gps_buf);else if(strncmp(gps_buf,"$GPRMC",6)==0)gps_parse_gprmc(gps_buf);gps_idx=0;}
        else if(gps_idx<127){gps_buf[gps_idx++]=c;gps_buf[gps_idx]=0;}
    }
}

// ======================== BME280 ========================
static bool bme280_init(){
    if(r8(BME280_ADDR,BME280_CHIP_ID)!=0x60)return false;
    uint8_t buf[26];rBuf(BME280_ADDR,BME280_CALIB_START,buf,26);
    dig_T1=buf[0]|(buf[1]<<8);dig_T2=buf[2]|(buf[3]<<8);dig_T3=buf[4]|(buf[5]<<8);
    dig_P1=buf[6]|(buf[7]<<8);dig_P2=buf[8]|(buf[9]<<8);dig_P3=buf[10]|(buf[11]<<8);dig_P4=buf[12]|(buf[13]<<8);
    dig_P5=buf[14]|(buf[15]<<8);dig_P6=buf[16]|(buf[17]<<8);dig_P7=buf[18]|(buf[19]<<8);dig_P8=buf[20]|(buf[21]<<8);
    dig_P9=buf[22]|(buf[23]<<8);dig_H1=buf[25];rBuf(BME280_ADDR,0xE1,buf,7);
    dig_H2=buf[0]|(buf[1]<<8);dig_H3=buf[2];dig_H4=((int16_t)buf[3]<<4)|(buf[4]&0x0F);dig_H5=((int16_t)buf[5]<<4)|(buf[4]>>4);dig_H6=(int8_t)buf[6];
    w8(BME280_ADDR,BME280_CTRL_HUM,0x05);w8(BME280_ADDR,BME280_CONFIG,(5<<2));w8(BME280_ADDR,BME280_CTRL_MEAS,(5<<5)|(5<<2)|3);return true;
}
static void bme280_read(float&t,float&p,float&h,float&a){
    uint8_t buf[8];rBuf(BME280_ADDR,BME280_DATA_START,buf,8);
    int32_t ap=((int32_t)buf[0]<<12)|((int32_t)buf[1]<<4)|(buf[2]>>4);
    int32_t at=((int32_t)buf[3]<<12)|((int32_t)buf[4]<<4)|(buf[5]>>4);int32_t ah=((int32_t)buf[6]<<8)|buf[7];
    int32_t v1=(((at>>3)-((int32_t)dig_T1<<1))*dig_T2)>>11;
    int32_t v2=(((((at>>4)-(int32_t)dig_T1)*((at>>4)-(int32_t)dig_T1))>>12)*dig_T3)>>14;
    t_fine=v1+v2;t=(float)((t_fine*5+128)>>8)*BME280_TEMP_SCALE;
    int64_t pv1=(int64_t)t_fine-128000LL,pv2=pv1*pv1*(int64_t)dig_P6;
    pv2+=((pv1*(int64_t)dig_P5)<<17);pv2+=((int64_t)dig_P4)<<35;
    pv1=((pv1*pv1*(int64_t)dig_P3)>>8)+((pv1*(int64_t)dig_P2)<<12);pv1=((((int64_t)1)<<47)+pv1)*(int64_t)dig_P1>>33;
    if(pv1){int64_t pr=1048576LL-ap;pr=(((pr<<31)-pv2)*3125LL)/pv1;
        pv1=((int64_t)dig_P9*(pr>>13)*(pr>>13))>>25;pv2=((int64_t)dig_P8*pr)>>19;
        pr=((pr+pv1+pv2)>>8)+((int64_t)dig_P7<<4);p=(float)pr*BME280_PRESS_SCALE;}else p=0;
    int32_t hv=t_fine-76800;
    hv=((((ah<<14)-((int32_t)dig_H4<<20)-((int32_t)dig_H5*hv))+16384)>>15)
       *(((((((hv*(int32_t)dig_H6)>>10)*(((hv*(int32_t)dig_H3)>>11)+32768))>>10)+2097152)*(int32_t)dig_H2+8192)>>14);
    hv-=((((hv>>15)*(hv>>15))>>7)*(int32_t)dig_H1)>>4;if(hv<0)hv=0;if(hv>419430400)hv=419430400;
    h=(float)(hv>>12)*BME280_HUM_SCALE;a=44330.0f*(1.0f-powf(p/SEA_LEVEL_HPA,0.1903f));
}

// ======================== AHT20 ========================
static bool aht20_init(){Wire.beginTransmission(AHT20_ADDR);Wire.write(AHT20_CMD_INIT);Wire.write(0x08);Wire.write(0x00);if(Wire.endTransmission())return false;delay(10);uint8_t s=r8(AHT20_ADDR,0x71);if(s&AHT20_STAT_CAL)return true;delay(40);s=r8(AHT20_ADDR,0x71);return(s&AHT20_STAT_CAL)!=0;}
static void aht20_trigger(){Wire.beginTransmission(AHT20_ADDR);Wire.write(AHT20_CMD_TRIG);Wire.write(0x33);Wire.write(0x00);Wire.endTransmission();aht_triggered=true;aht_trigger_ms=millis();}
static void aht20_read_finish(float&t,float&h){if(!aht_triggered)return;if(millis()-aht_trigger_ms<80)return;uint8_t buf[7];rBuf(AHT20_ADDR,0x00,buf,7);aht_triggered=false;if(buf[0]&AHT20_STAT_BUSY){t=NAN;h=NAN;return;}uint32_t rh=((uint32_t)buf[1]<<12)|((uint32_t)buf[2]<<4)|(buf[3]>>4);uint32_t rt=(((uint32_t)buf[3]&0x0F)<<16)|((uint32_t)buf[4]<<8)|buf[5];h=(float)rh*9.5367431640625e-5f;t=(float)rt*1.9073486328125e-4f-50.0f;}

// ======================== BNO055 ========================
static bool bno055_init(){
    if(r8(BNO055_ADDR,BNO055_CHIP_ID)!=0xA0)return false;
    w8(BNO055_ADDR,BNO055_OPR_MODE,0x00);delay(30);   // CONFIG rejimi
    w8(BNO055_ADDR,BNO055_PWR_MODE,0x00);delay(10);   // normal guc
    w8(BNO055_ADDR,BNO055_PAGE_ID,0x00);              // page 0 (data registerleri)
    w8(BNO055_ADDR,BNO055_UNIT_SEL,0x00);             // m/s2, dps, deg, C
    w8(BNO055_ADDR,BNO055_SYS_TRIG,0x80);delay(50);   // CLK_SRC: xarici kristal
    w8(BNO055_ADDR,BNO055_OPR_MODE,0x0C);delay(200);  // NDOF fusion
    return true;
}
static void bno055_read_raw(float&ax,float&ay,float&az,float&gx,float&gy,float&gz,float&mx,float&my,float&mz){
    uint8_t buf[6];
    rBuf(BNO055_ADDR,BNO055_ACC_START,buf,6);ax=(int16_t)((buf[1]<<8)|buf[0])*BNO055_ACC_SCALE;ay=(int16_t)((buf[3]<<8)|buf[2])*BNO055_ACC_SCALE;az=(int16_t)((buf[5]<<8)|buf[4])*BNO055_ACC_SCALE;
    rBuf(BNO055_ADDR,BNO055_GYRO_START,buf,6);gx=(int16_t)((buf[1]<<8)|buf[0])*BNO055_GYRO_SCALE;gy=(int16_t)((buf[3]<<8)|buf[2])*BNO055_GYRO_SCALE;gz=(int16_t)((buf[5]<<8)|buf[4])*BNO055_GYRO_SCALE;
    rBuf(BNO055_ADDR,BNO055_MAG_START,buf,6);mx=(int16_t)((buf[1]<<8)|buf[0])*BNO055_MAG_SCALE;my=(int16_t)((buf[3]<<8)|buf[2])*BNO055_MAG_SCALE;mz=(int16_t)((buf[5]<<8)|buf[4])*BNO055_MAG_SCALE;
}

// ======================== I2C SCAN + DIAQNOSTIKA ========================
static void i2c_scan_diag(){
    uint8_t addrs[16]; uint8_t n=0;
    for(uint8_t a=0x03;a<0x78;a++){
        Wire.beginTransmission(a);
        if(Wire.endTransmission()==0){ if(n<16) addrs[n++]=a; }
    }
    bool bno=false,bme=false,aht=false;
    for(uint8_t i=0;i<n;i++){
        if(addrs[i]==BNO055_ADDR)bno=true;
        else if(addrs[i]==BME280_ADDR)bme=true;
        else if(addrs[i]==AHT20_ADDR)aht=true;
    }
    // BNO055 diagnostika: OPR_MODE (0x0C=NDOF, 0x00=CONFIG) + kalibrasiya
    uint8_t opmode = bno ? r8(BNO055_ADDR, BNO055_OPR_MODE) : 0xFF;
    uint8_t calib  = bno ? r8(BNO055_ADDR, BNO055_CALIB_STAT) : 0xFF;

    // USB
    Serial.print(F("[I2C] "));Serial.print(n);Serial.print(F(" dev | BNO055="));Serial.print(bno?1:0);
    Serial.print(F(" BME280="));Serial.print(bme?1:0);Serial.print(F(" AHT20="));Serial.print(aht?1:0);
    Serial.print(F(" | mode=0x"));Serial.print(opmode,HEX);
    Serial.print(F(" cal=0x"));Serial.print(calib,HEX);
    Serial.print(F(" | "));
    for(uint8_t i=0;i<n;i++){Serial.print(F("0x"));Serial.print(addrs[i],HEX);Serial.print(' ');}
    Serial.println();
    // RF (RFD900X) diaqnostik sətir
    RF_SERIAL.print(F("DIAG,"));RF_SERIAL.print(millis());RF_SERIAL.print(',');
    RF_SERIAL.print(bno?1:0);RF_SERIAL.print(',');RF_SERIAL.print(bme?1:0);RF_SERIAL.print(',');RF_SERIAL.print(aht?1:0);
    RF_SERIAL.print(',');RF_SERIAL.print(n);
    for(uint8_t i=0;i<n;i++){RF_SERIAL.print(',');RF_SERIAL.print(F("0x"));RF_SERIAL.print(addrs[i],HEX);}
    RF_SERIAL.print(',');RF_SERIAL.print(F("mode=0x"));RF_SERIAL.print(opmode,HEX);
    RF_SERIAL.print(',');RF_SERIAL.print(F("cal=0x"));RF_SERIAL.print(calib,HEX);
    RF_SERIAL.println();
}

// ======================== SETUP ========================
void setup(){
    pinMode(LED_PIN,OUTPUT);digitalWrite(LED_PIN,HIGH);
    esc_init();   // ESC-ler guc acilanda derhal 1000 us alir (tehlukesizlik)
    Serial.begin(115200);delay(200);

    Serial.println(F("\n=== TEENSY 4.1 " DEVICE_NAME " ==="));
    Serial.println(F("  IMU: AX,AY,AZ,GX,GY,GZ,MX,MY,MZ"));
    Serial.println(F("  BME280 + AHT20 + GPS + Kalman + Attitude EKF\n"));

    rf_command_init();
    altvel.init();
    flight.init();
    Serial.println(F("[ESC] 50 Hz PWM: pin 15 + 23 (1000..2000 us)"));
    Serial.println(F("[RF-CMD] RX: '1'=ARM, '0'=DISARM"));

    RF_SERIAL.begin(RF_BAUD);

    Wire.begin();Wire.setClock(I2C_FREQ);
    Serial.print(F("[I2C] "));Serial.print(I2C_FREQ/1000);Serial.println(F(" kHz"));
    i2c_scan_diag();

    ok.bno055=bno055_init();
    Serial.print(F("BNO055: "));Serial.println(ok.bno055?F("100 Hz"):F("FAIL"));
    if(ok.bno055){uint8_t cal=r8(BNO055_ADDR,BNO055_CALIB_STAT);
        Serial.print(F("  Cal:"));Serial.print(cal>>6);Serial.print('/');Serial.print((cal>>4)&3);Serial.print('/');Serial.print((cal>>2)&3);Serial.print('/');Serial.println(cal&3);}

    ok.bme280=bme280_init();
    Serial.print(F("BME280: "));
    if(ok.bme280){Serial.println(F("25 Hz IIR x16"));float t,p,h,a;bme280_read(t,p,h,a);
        kalmanTemp.init(0.001f,0.5f,t);kalmanAlt.init(0.01f,2.0f,a);kalman_ready=true;
        Serial.print(F("  T="));Serial.print(t,1);Serial.print(F(" P="));Serial.print(p,1);Serial.println(F("hPa"));}
    else Serial.println(F("FAIL"));

    ok.aht20=aht20_init();
    Serial.print(F("AHT20:  "));Serial.println(ok.aht20?F("1 Hz async"):F("FAIL"));
    if(ok.aht20)aht20_trigger();

    GPS_SERIAL.begin(GPS_BAUD);
    Serial.print(F("GPS:    Serial7 @ "));Serial.print(GPS_BAUD);Serial.println(F(" baud"));

    ekf.init(0.0f, 0.0f, 9.80665f);
    Serial.println(F("[FILTER] Attitude EKF: 7-state quaternion (gyro+accel), 100 Hz"));

    Serial.print(F("[RF] Serial2 @ "));Serial.print(RF_BAUD);Serial.print(F(" baud, Header: "));
    Serial.print(DEVICE_HEADER);Serial.println(F(", 15 Hz CSV"));
    Serial.println(F("Packet: XX,ts,AX..AZ,GX..GZ,MX..MZ,T,P,H,Alt,aT,aH,lat,lon,gps_alt,spd,crs,roll,pitch,yaw\n"));

    uint32_t now=millis();
    lastBno=lastBme=lastAht=lastGps=lastPrn=lastRf=lastI2cDiag=now;
    lastFlight=now;
    digitalWrite(LED_PIN,LOW);
}

// ======================== LOOP ========================
void loop(){
    uint32_t now=millis();

    rf_command_update();

    if(now-lastFlight>=FLIGHT_PERIOD){lastFlight=now;
        static bool wasLevel=false;
        float tilt=fmaxf(fabsf(mad_roll),fabsf(mad_pitch));
        bool level;
        if(wasLevel) level=(tilt<=8.0f);
        else         level=(tilt<=5.0f);
        if(!ok.bno055){level=false;wasLevel=false;} else wasLevel=level;
        bool descending = (altvel.vel < 0.0f);
        flight.update(rf_armed(), level, descending, altvel.rel_alt, now);
    }

    if(now-lastBno>=BNO055_PERIOD){lastBno=now;
        if(ok.bno055){bno055_read_raw(ax,ay,az,gx,gy,gz,mx,my,mz);ekf.predict(gx,gy,gz,0.01f);ekf.update(ax,ay,az);ekf.getEulerDeg(mad_roll,mad_pitch,mad_yaw);}
    }
    if(now-lastBme>=BME280_PERIOD){lastBme=now;
        if(ok.bme280){bme280_read(bme_t,bme_p,bme_h,bme_a);bme_tk=kalmanTemp.update(bme_t);bme_ak=kalmanAlt.update(bme_a);
            altvel.update(bme_p, az, ax, ay);
        }
    }
    if(ok.aht20&&now-lastAht>=AHT20_PERIOD){lastAht=now;if(aht_triggered)aht20_read_finish(aht_t,aht_h);aht20_trigger();}
    gps_read();if(now-lastGps>=GPS_PERIOD){lastGps=now;ok.gps_fix=(gps.fix>0);}
    if(now-lastI2cDiag>=I2C_DIAG_PERIOD){lastI2cDiag=now;i2c_scan_diag();}

    static bool led=false;if(now&0x200){if(!led){digitalWrite(LED_PIN,HIGH);led=true;}}else{if(led){digitalWrite(LED_PIN,LOW);led=false;}}

    if(now-lastPrn>=PRINT_PERIOD){lastPrn=now;
        Serial.print(now);Serial.print(' ');
        if(ok.bno055){Serial.print(F("A:"));Serial.print(ax,2);Serial.print(',');Serial.print(ay,2);Serial.print(',');Serial.print(az,2);Serial.print(F(" G:"));Serial.print(gx,3);Serial.print(',');Serial.print(gy,3);Serial.print(',');Serial.print(gz,3);}
        else Serial.print(F("IMU:OFF"));
        Serial.print(F(" | T:"));if(ok.bme280){Serial.print(bme_tk,1);Serial.print('/');Serial.print(bme_ak,1);}else Serial.print(F("OFF"));
        Serial.print(F(" | A:"));if(ok.aht20){Serial.print(aht_t,1);Serial.print('/');Serial.print(aht_h,1);}else Serial.print(F("OFF"));
        Serial.print(F(" | GPS:"));if(ok.gps_fix){Serial.print(gps.lat,5);Serial.print(',');Serial.print(gps.lon,5);}else Serial.print(F("NO"));
        Serial.print(F(" | FLT:"));Serial.print(rf_armed()?F("ARM"):F("DISARM"));
        Serial.print('/');Serial.print((int)flight.state_code());
        Serial.print(F(" alt="));Serial.print(altvel.rel_alt,1);
        Serial.print(F(" vel="));Serial.print(altvel.vel,1);
        Serial.print(F(" g="));Serial.print(altvel.g_force,2);
        Serial.print(F(" pwm="));Serial.print(flight.throttle());
        Serial.println();
    }

    if(now-lastRf>=RF_PERIOD){lastRf=now;
        RF_SERIAL.print(DEVICE_HEADER);RF_SERIAL.print(',');
        RF_SERIAL.print(now);RF_SERIAL.print(',');
        if(ok.bno055){
            RF_SERIAL.print(ax,3);RF_SERIAL.print(',');RF_SERIAL.print(ay,3);RF_SERIAL.print(',');RF_SERIAL.print(az,3);RF_SERIAL.print(',');
            RF_SERIAL.print(gx,4);RF_SERIAL.print(',');RF_SERIAL.print(gy,4);RF_SERIAL.print(',');RF_SERIAL.print(gz,4);RF_SERIAL.print(',');
            RF_SERIAL.print(mx,2);RF_SERIAL.print(',');RF_SERIAL.print(my,2);RF_SERIAL.print(',');RF_SERIAL.print(mz,2);
        }else RF_SERIAL.print(F("N,N,N,N,N,N,N,N,N"));
        RF_SERIAL.print(',');
        if(ok.bme280){RF_SERIAL.print(bme_tk,1);RF_SERIAL.print(',');RF_SERIAL.print(bme_p,1);RF_SERIAL.print(',');RF_SERIAL.print(bme_h,1);RF_SERIAL.print(',');RF_SERIAL.print(bme_ak,1);}
        else RF_SERIAL.print(F("N,N,N,N"));
        RF_SERIAL.print(',');
        if(ok.aht20){RF_SERIAL.print(aht_t,1);RF_SERIAL.print(',');RF_SERIAL.print(aht_h,1);}else RF_SERIAL.print(F("N,N"));
        RF_SERIAL.print(',');
        if(ok.gps_fix){RF_SERIAL.print(gps.lat,6);RF_SERIAL.print(',');RF_SERIAL.print(gps.lon,6);RF_SERIAL.print(',');RF_SERIAL.print(gps.altitude,1);RF_SERIAL.print(',');RF_SERIAL.print(gps.speed,2);RF_SERIAL.print(',');RF_SERIAL.print(gps.course,1);}
        else RF_SERIAL.print(F("N,N,N,N,N"));
        RF_SERIAL.print(',');
        RF_SERIAL.print(mad_roll,2);RF_SERIAL.print(',');RF_SERIAL.print(mad_pitch,2);RF_SERIAL.print(',');RF_SERIAL.print(mad_yaw,2);
        RF_SERIAL.print(',');RF_SERIAL.print(altvel.rel_alt,2);
        RF_SERIAL.print(',');RF_SERIAL.print(altvel.vel,2);
        RF_SERIAL.print(',');RF_SERIAL.print(altvel.g_force,3);
        RF_SERIAL.print(',');RF_SERIAL.print(altvel.dpdt,3);
        RF_SERIAL.print(',');RF_SERIAL.print(rf_armed()?1:0);
        RF_SERIAL.print(',');RF_SERIAL.print((int)flight.state_code());
        RF_SERIAL.print(',');RF_SERIAL.print(flight.throttle());
        RF_SERIAL.println();
    }
}