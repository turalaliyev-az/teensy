#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <EEPROM.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BME280.h>
#include <Adafruit_AHTX0.h>

#include <TinyGPS++.h>

// ======================== SENSOR OBYEKTLERI ========================
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BME280 bme;
Adafruit_AHTX0 aht;

// ======================== GPS (TinyGPS++) ========================
static TinyGPSPlus tgps;

// ======================== ESC (50 Hz PWM) ========================
#define ESC1_PIN      15
#define ESC2_PIN      23
#define ESC_PWM_FREQ  50.0f
#define ESC_US_MIN    1000
#define ESC_US_MAX    2000
#define ESC_US_OFF    1000
#define ESC_US_RUN    1480

void esc_init();
void esc_write_us(uint16_t us);
void esc_write_us(uint16_t us1, uint16_t us2);

static uint32_t us_to_duty(uint16_t us) {
    if (us < ESC_US_MIN) us = ESC_US_MIN;
    if (us > ESC_US_MAX) us = ESC_US_MAX;
    return (uint32_t)us * 65536UL / 20000UL;
}

void esc_init() {
    pinMode(ESC1_PIN, OUTPUT);
    pinMode(ESC2_PIN, OUTPUT);
    analogWriteFrequency(ESC1_PIN, ESC_PWM_FREQ);
    analogWriteFrequency(ESC2_PIN, ESC_PWM_FREQ);
    analogWriteResolution(16);
    esc_write_us(ESC_US_OFF);
}

void esc_write_us(uint16_t us1, uint16_t us2) {
    analogWrite(ESC1_PIN, us_to_duty(us1));
    analogWrite(ESC2_PIN, us_to_duty(us2));
}

void esc_write_us(uint16_t us) {
    esc_write_us(us, us);
}

// ======================== RF EMRLERI ========================
static bool _armed = true;
static bool _last_ack = true;

void rf_command_init() {
    _armed = true;
    _last_ack = true;
}

void rf_command_update() {
    while (Serial2.available()) {
        char c = (char)Serial2.read();
        if (c == '1')      _armed = true;
        else if (c == '0') _armed = false;
        else continue;

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
struct AltVel {
    float rel_alt;
    float vel;
    float g_force;
    float dpdt;

    void init();
    void update(float pressure_hpa, float az, float ax, float ay, bool imu_ok);
    bool calibrated() const { return _calibrated; }

private:
    float _p0;
    float _p_smooth;
    float _prev_p;
    float _dpdt_smooth;
    float _a_smooth;
    float _g_smooth;
    uint32_t _last_us;
    bool _have_prev;

    float _P00, _P01, _P11;

    uint32_t _calib_start_ms;
    float _calib_sum;
    uint32_t _calib_count;
    bool _calibrated;
};

#define GRAVITY      9.80665f
#define CALIB_MS     2500UL
#define CALIB_MIN_N  15
#define P_ALPHA      0.30f
#define DPDT_ALPHA   0.25f
#define A_ALPHA      0.25f
#define G_ALPHA      0.15f
#define Q_ALT        0.05f
#define Q_VEL        0.6f
#define R_ALT        1.0f

void AltVel::init() {
    rel_alt = 0.0f; vel = 0.0f; g_force = 0.0f; dpdt = 0.0f;
    _p0 = 1013.25f; _p_smooth = 0.0f; _prev_p = 0.0f; _dpdt_smooth = 0.0f;
    _a_smooth = 0.0f; _g_smooth = 0.0f; _last_us = 0; _have_prev = false;
    _P00 = 1.0f; _P01 = 0.0f; _P11 = 1.0f;
    _calib_start_ms = 0; _calib_sum = 0.0f; _calib_count = 0; _calibrated = false;
}

void AltVel::update(float pressure_hpa, float az, float ax, float ay, bool imu_ok) {
    if (isnan(pressure_hpa) || isinf(pressure_hpa)) return;

    uint32_t now_us = micros();
    float dt = 0.0f;
    if (_last_us != 0) {
        dt = (float)(now_us - _last_us) * 1.0e-6f;
        if (dt < 0.0f) dt = 0.0f;
        if (dt > 0.5f) dt = 0.5f;
    }
    _last_us = now_us;

    float g_raw;
    if (imu_ok) {
        g_raw = sqrtf(ax*ax + ay*ay + az*az) / GRAVITY;
    } else {
        g_raw = 1.0f;
    }

    if (_g_smooth <= 0.0f) {
        _g_smooth = g_raw;
    } else {
        _g_smooth += G_ALPHA * (g_raw - _g_smooth);
    }
    g_force = _g_smooth;

    if (!_have_prev) {
        _p_smooth = pressure_hpa;
        _prev_p = pressure_hpa;
        _have_prev = true;
    } else {
        _p_smooth += P_ALPHA * (pressure_hpa - _p_smooth);
    }

    if (dt > 1.0e-4f) {
        float raw = (_p_smooth - _prev_p) / dt;
        _dpdt_smooth += DPDT_ALPHA * (raw - _dpdt_smooth);
    }
    _prev_p = _p_smooth;
    dpdt = _dpdt_smooth;

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
            Serial.print(F("[ALT] Kalibrasiya tamam: P0="));
            Serial.print(_p0, 2);
            Serial.println(F(" hPa"));
        }
        return;
    }

    float z = 0.0f;
    if (_p0 > 1.0f) {
        z = 44330.0f * (1.0f - powf(_p_smooth / _p0, 0.1903f));
    }

    float a_vert;
    if (imu_ok) {
        a_vert = az - GRAVITY;
    } else {
        a_vert = 0.0f;
    }
    _a_smooth += A_ALPHA * (a_vert - _a_smooth);

    float alt_p = rel_alt + vel*dt + 0.5f*_a_smooth*dt*dt;
    float vel_p = vel + _a_smooth*dt;
    float P00_p = _P00 + 2.0f*dt*_P01 + dt*dt*_P11 + Q_ALT;
    float P01_p = _P01 + dt*_P11;
    float P11_p = _P11 + Q_VEL;

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

// ======================== UCUS NEZARETCISI ========================
enum FlightState : uint8_t { FS_DISARMED = 0, FS_ARMED = 1, FS_MOTORS_ON = 2 };

struct FlightCtrl {
    FlightState state;
    float throttle_us;

    void init();
    void update(bool armed, bool level_ok, bool descending, float rel_alt, uint32_t now_ms);
    uint16_t throttle() const { return (uint16_t)throttle_us; }
    uint8_t state_code() const { return (uint8_t)state; }

private:
    uint32_t _last_ms;
};

#define ALT_TRIGGER_M   500.0f
#define RAMP_MS         500UL
#define RAMP_STEP_US    ((float)(ESC_US_RUN - ESC_US_OFF) / (float)RAMP_MS)

void FlightCtrl::init() {
    state = FS_DISARMED;
    throttle_us = (float)ESC_US_OFF;
    _last_ms = 0;
}

void FlightCtrl::update(bool armed, bool level_ok, bool descending, float rel_alt, uint32_t now_ms) {
    uint32_t dt_ms = 0;
    if (_last_ms != 0) {
        dt_ms = now_ms - _last_ms;
        if (dt_ms > 100) dt_ms = 100;
    }
    _last_ms = now_ms;

    if (!armed) {
        state = FS_DISARMED;
        throttle_us = (float)ESC_US_OFF;
        esc_write_us((uint16_t)throttle_us);
        return;
    }

    if (state == FS_DISARMED) {
        state = FS_ARMED;
        throttle_us = (float)ESC_US_OFF;
    }

    if (state == FS_ARMED) {
        if (level_ok && descending && (rel_alt <= ALT_TRIGGER_M)) {
            state = FS_MOTORS_ON;
        }
    }

    if (state == FS_MOTORS_ON) {
        if (!level_ok) {
            esc_write_us((uint16_t)throttle_us);
            return;
        }

        if (throttle_us < (float)ESC_US_RUN) {
            throttle_us += RAMP_STEP_US * (float)dt_ms;
            if (throttle_us > (float)ESC_US_RUN) {
                throttle_us = (float)ESC_US_RUN;
            }
        }
    }
    esc_write_us((uint16_t)throttle_us);
}

// ======================== QUATERNION ROTATION ========================
static void quat_rotate_vec(const float q[4], const float v[3], float out[3]) {
    float qw = q[0], qx = q[1], qy = q[2], qz = q[3];

    float tx = 2.0f * (qy * v[2] - qz * v[1]);
    float ty = 2.0f * (qz * v[0] - qx * v[2]);
    float tz = 2.0f * (qx * v[1] - qy * v[0]);

    out[0] = v[0] + qw * tx + (qy * tz - qz * ty);
    out[1] = v[1] + qw * ty + (qz * tx - qx * tz);
    out[2] = v[2] + qw * tz + (qx * ty - qy * tx);
}

static void quat_rotate_vec_inv(const float q[4], const float v[3], float out[3]) {
    float qinv[4] = {q[0], -q[1], -q[2], -q[3]};
    quat_rotate_vec(qinv, v, out);
}

// ======================== 7-DOVLETLI QUATERNION EKF + MAG YAW ========================
struct AttitudeEKF {
    float q[4];
    float b[3];
    float P[7][7];

    float _mag_norm_ref;
    float _mag_ref[3];
    bool _mag_ref_valid;

    void init(float ax, float ay, float az);
    void predict(float gx, float gy, float gz, float dt);
    void update(float ax, float ay, float az);
    void updateMag(float mx, float my, float mz);
    void getEulerDeg(float &roll, float &pitch, float &yaw) const;
    void symmetrize();

private:
    void updateVectorMeasurement(const float meas[3], const float ref[3], float r);
};

#define RAD2DEG 57.29577951308232f
#define GYRO_NOISE  0.02f
#define BIAS_NOISE  0.0005f
#define R_BASE      0.003f
#define R_ADAPT     2.0f
#define ACC_MIN     3.0f
#define ACC_MAX     25.0f

#define MAG_PERIOD          50
#define MAG_MIN             15.0f
#define MAG_MAX             120.0f
#define R_MAG_BASE          0.02f
#define R_MAG_ADAPT         0.5f
#define MAG_NORM_ALPHA      0.02f
#define MAG_REF_ALPHA       0.05f

static void quat_norm(float q[4]) {
    float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n < 1e-8f) {
        q[0]=1.0f; q[1]=q[2]=q[3]=0.0f;
        return;
    }
    n = 1.0f / n;
    q[0]*=n; q[1]*=n; q[2]*=n; q[3]*=n;
}

void AttitudeEKF::init(float ax, float ay, float az) {
    float n = sqrtf(ax*ax + ay*ay + az*az);
    if (n < 1e-4f) n = 1.0f;
    ax /= n; ay /= n; az /= n;
    float axis_x = ay, axis_y = -ax, axis_z = 0.0f;
    float d = az;
    if (d > 0.999999f) {
        q[0]=1.0f; q[1]=q[2]=q[3]=0.0f;
    } else if (d < -0.999999f) {
        q[0]=0.0f; q[1]=1.0f; q[2]=q[3]=0.0f;
    } else {
        float ang = acosf(d); float s = sinf(ang * 0.5f);
        float an = sqrtf(axis_x*axis_x + axis_y*axis_y);
        if (an < 1e-8f) an = 1.0f;
        q[0] = cosf(ang * 0.5f); q[1] = s * axis_x / an; q[2] = s * axis_y / an; q[3] = s * axis_z / an;
    }
    b[0] = b[1] = b[2] = 0.0f;
    for (int i=0;i<7;i++) for (int j=0;j<7;j++) P[i][j] = 0.0f;
    P[0][0]=P[1][1]=P[2][2]=P[3][3]=0.1f;
    P[4][4]=P[5][5]=P[6][6]=0.1f;

    _mag_norm_ref = 0.0f;
    _mag_ref_valid = false;
    _mag_ref[0] = 1.0f;
    _mag_ref[1] = 0.0f;
    _mag_ref[2] = 0.0f;
}

void AttitudeEKF::predict(float gx, float gy, float gz, float dt) {
    if (dt < 1e-6f) return;

    float wx = gx - b[0], wy = gy - b[1], wz = gz - b[2];
    float qd[4];
    qd[0] = 0.5f * (-wx*q[1] - wy*q[2] - wz*q[3]);
    qd[1] = 0.5f * ( wx*q[0] + wz*q[2] - wy*q[3]);
    qd[2] = 0.5f * ( wy*q[0] - wz*q[1] + wx*q[3]);
    qd[3] = 0.5f * ( wz*q[0] + wy*q[1] - wx*q[2]);
    q[0] += qd[0]*dt; q[1] += qd[1]*dt; q[2] += qd[2]*dt; q[3] += qd[3]*dt;
    quat_norm(q);

    float F[7][7];
    for (int i=0;i<7;i++) for (int j=0;j<7;j++) F[i][j] = 0.0f;
    F[0][0]=1.0f; F[0][1]=-0.5f*wx*dt; F[0][2]=-0.5f*wy*dt; F[0][3]=-0.5f*wz*dt;
    F[1][0]= 0.5f*wx*dt; F[1][1]=1.0f; F[1][2]= 0.5f*wz*dt; F[1][3]=-0.5f*wy*dt;
    F[2][0]= 0.5f*wy*dt; F[2][1]=-0.5f*wz*dt; F[2][2]=1.0f; F[2][3]= 0.5f*wx*dt;
    F[3][0]= 0.5f*wz*dt; F[3][1]= 0.5f*wy*dt; F[3][2]=-0.5f*wx*dt; F[3][3]=1.0f;
    F[0][4]= 0.5f*q[1]*dt; F[0][5]= 0.5f*q[2]*dt; F[0][6]= 0.5f*q[3]*dt;
    F[1][4]=-0.5f*q[0]*dt; F[1][5]= 0.5f*q[3]*dt; F[1][6]=-0.5f*q[2]*dt;
    F[2][4]=-0.5f*q[3]*dt; F[2][5]=-0.5f*q[0]*dt; F[2][6]= 0.5f*q[1]*dt;
    F[3][4]= 0.5f*q[2]*dt; F[3][5]=-0.5f*q[1]*dt; F[3][6]=-0.5f*q[0]*dt;
    F[4][4]=F[5][5]=F[6][6]=1.0f;

    float Q[7][7];
    for (int i=0;i<7;i++) for (int j=0;j<7;j++) Q[i][j] = 0.0f;
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

    float FP[7][7];
    for (int i=0;i<7;i++) {
        for (int j=0;j<7;j++) {
            float sum=0.0f;
            for (int k=0;k<7;k++) sum += F[i][k] * P[k][j];
            FP[i][j] = sum;
        }
    }
    for (int i=0;i<7;i++) {
        for (int j=0;j<7;j++) {
            float sum=0.0f;
            for (int k=0;k<7;k++) sum += FP[i][k] * F[j][k];
            P[i][j] = sum + Q[i][j];
        }
    }

    symmetrize();
}

void AttitudeEKF::update(float ax, float ay, float az) {
    if (isnan(ax) || isinf(ax) || isnan(ay) || isinf(ay) || isnan(az) || isinf(az)) return;

    float amag = sqrtf(ax*ax + ay*ay + az*az);
    if (amag < ACC_MIN || amag > ACC_MAX) return;
    float n = 1.0f / amag;
    float meas[3] = {ax * n, ay * n, az * n};

    float dev = fabsf(amag - GRAVITY) / GRAVITY;
    float r = R_BASE + R_ADAPT * dev * dev;

    float ref[3] = {0.0f, 0.0f, 1.0f};
    updateVectorMeasurement(meas, ref, r);
}

void AttitudeEKF::updateMag(float mx, float my, float mz) {
    if (isnan(mx) || isinf(mx) ||
        isnan(my) || isinf(my) ||
        isnan(mz) || isinf(mz)) {
        return;
    }

    float mmag = sqrtf(mx*mx + my*my + mz*mz);
    if (!(mmag >= MAG_MIN && mmag <= MAG_MAX)) return;

    if (_mag_norm_ref <= 1.0f) {
        _mag_norm_ref = mmag;
    } else {
        _mag_norm_ref += MAG_NORM_ALPHA * (mmag - _mag_norm_ref);
    }

    float dev = fabsf(mmag - _mag_norm_ref) / _mag_norm_ref;
    if (dev > 0.45f) return;

    float meas[3] = {mx / mmag, my / mmag, mz / mmag};

    float m_world[3];
    quat_rotate_vec(q, meas, m_world);

    float norm_ref = sqrtf(m_world[0]*m_world[0] + m_world[2]*m_world[2]);
    if (norm_ref < 0.2f) return;

    float ref_raw[3];
    ref_raw[0] = m_world[0] / norm_ref;
    ref_raw[1] = 0.0f;
    ref_raw[2] = m_world[2] / norm_ref;

    if (!_mag_ref_valid) {
        _mag_ref[0] = ref_raw[0];
        _mag_ref[1] = ref_raw[1];
        _mag_ref[2] = ref_raw[2];
        _mag_ref_valid = true;
    } else {
        _mag_ref[0] += MAG_REF_ALPHA * (ref_raw[0] - _mag_ref[0]);
        _mag_ref[1] = 0.0f;
        _mag_ref[2] += MAG_REF_ALPHA * (ref_raw[2] - _mag_ref[2]);

        float rn = sqrtf(_mag_ref[0]*_mag_ref[0] + _mag_ref[2]*_mag_ref[2]);
        if (rn < 0.2f) return;

        _mag_ref[0] /= rn;
        _mag_ref[1] = 0.0f;
        _mag_ref[2] /= rn;
    }

    float ref[3] = {_mag_ref[0], _mag_ref[1], _mag_ref[2]};
    float r = R_MAG_BASE + R_MAG_ADAPT * dev * dev;
    updateVectorMeasurement(meas, ref, r);
}

void AttitudeEKF::symmetrize() {
    for (int i=0; i<7; i++) {
        if (isnan(P[i][i]) || isinf(P[i][i]) || P[i][i] < 1e-12f) {
            P[i][i] = 1e-12f;
        }
    }
    for (int i=0; i<7; i++) {
        for (int j=i+1; j<7; j++) {
            float avg = 0.5f * (P[i][j] + P[j][i]);
            if (isnan(avg) || isinf(avg)) avg = 0.0f;
            P[i][j] = P[j][i] = avg;
        }
    }
}

void AttitudeEKF::updateVectorMeasurement(const float meas[3], const float ref[3], float r) {
    if (!(r > 1e-9f)) return;

    float h[3];
    quat_rotate_vec_inv(q, ref, h);

    float inn0 = meas[0] - h[0];
    float inn1 = meas[1] - h[1];
    float inn2 = meas[2] - h[2];

    if (isnan(inn0) || isinf(inn0) ||
        isnan(inn1) || isinf(inn1) ||
        isnan(inn2) || isinf(inn2)) {
        return;
    }

    float H[3][7];
    for (int i=0; i<3; i++) {
        for (int j=0; j<7; j++) H[i][j] = 0.0f;
    }

    float w  = q[0];
    float x  = q[1];
    float yy = q[2];
    float zz = q[3];

    float rx = ref[0];
    float ry = ref[1];
    float rz = ref[2];

    H[0][0] = 2.0f*w*rx  + 2.0f*zz*ry - 2.0f*yy*rz;
    H[0][1] = 2.0f*x*rx  + 2.0f*yy*ry + 2.0f*zz*rz;
    H[0][2] = -2.0f*yy*rx + 2.0f*x*ry - 2.0f*w*rz;
    H[0][3] = -2.0f*zz*rx + 2.0f*w*ry + 2.0f*x*rz;

    H[1][0] = -2.0f*zz*rx + 2.0f*w*ry + 2.0f*x*rz;
    H[1][1] = 2.0f*yy*rx  - 2.0f*x*ry + 2.0f*w*rz;
    H[1][2] = 2.0f*x*rx   + 2.0f*yy*ry + 2.0f*zz*rz;
    H[1][3] = -2.0f*w*rx  - 2.0f*zz*ry + 2.0f*yy*rz;

    H[2][0] = 2.0f*yy*rx - 2.0f*x*ry + 2.0f*w*rz;
    H[2][1] = 2.0f*zz*rx - 2.0f*w*ry - 2.0f*x*rz;
    H[2][2] = 2.0f*w*rx  + 2.0f*zz*ry - 2.0f*yy*rz;
    H[2][3] = 2.0f*x*rx  + 2.0f*yy*ry + 2.0f*zz*rz;

    float PHt[7][3];
    for (int i=0; i<7; i++) {
        for (int j=0; j<3; j++) {
            float sum=0.0f;
            for (int k=0; k<7; k++) sum += P[i][k] * H[j][k];
            PHt[i][j] = sum;
        }
    }

    float S[3][3];
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            float sum=0.0f;
            for (int k=0; k<7; k++) sum += H[i][k] * PHt[k][j];
            S[i][j] = sum;
        }
    }
    S[0][0] += r;
    S[1][1] += r;
    S[2][2] += r;

    float det = S[0][0]*(S[1][1]*S[2][2]-S[1][2]*S[2][1])
              - S[0][1]*(S[1][0]*S[2][2]-S[1][2]*S[2][0])
              + S[0][2]*(S[1][0]*S[2][1]-S[1][1]*S[2][0]);
    if (!(fabsf(det) > 1e-12f)) return;

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

    float K[7][3];
    for (int i=0; i<7; i++) {
        for (int j=0; j<3; j++) {
            float sum=0.0f;
            for (int k=0; k<3; k++) sum += PHt[i][k] * Si[k][j];
            K[i][j] = sum;
        }
    }

    q[0] += K[0][0]*inn0 + K[0][1]*inn1 + K[0][2]*inn2;
    q[1] += K[1][0]*inn0 + K[1][1]*inn1 + K[1][2]*inn2;
    q[2] += K[2][0]*inn0 + K[2][1]*inn1 + K[2][2]*inn2;
    q[3] += K[3][0]*inn0 + K[3][1]*inn1 + K[3][2]*inn2;
    quat_norm(q);

    b[0] += K[4][0]*inn0 + K[4][1]*inn1 + K[4][2]*inn2;
    b[1] += K[5][0]*inn0 + K[5][1]*inn1 + K[5][2]*inn2;
    b[2] += K[6][0]*inn0 + K[6][1]*inn1 + K[6][2]*inn2;

    float KH[7][7];
    for (int i=0; i<7; i++) {
        for (int j=0; j<7; j++) {
            float sum=0.0f;
            for (int k=0; k<3; k++) sum += K[i][k] * H[k][j];
            KH[i][j] = sum;
        }
    }
    float Pnew[7][7];
    for (int i=0; i<7; i++) {
        for (int j=0; j<7; j++) {
            float sum=0.0f;
            for (int k=0; k<7; k++) sum += ((i==k?1.0f:0.0f) - KH[i][k]) * P[k][j];
            Pnew[i][j] = sum;
        }
    }
    for (int i=0;i<7;i++) for (int j=0;j<7;j++) P[i][j] = Pnew[i][j];
    symmetrize();
}

void AttitudeEKF::getEulerDeg(float &roll, float &pitch, float &yaw) const {
    roll  = atan2f(2.0f*(q[0]*q[1] + q[2]*q[3]), 1.0f - 2.0f*(q[1]*q[1] + q[2]*q[2])) * RAD2DEG;

    float sp = 2.0f*(q[0]*q[2] - q[3]*q[1]);
    if (sp > 1.0f) sp = 1.0f;
    if (sp < -1.0f) sp = -1.0f;
    pitch = asinf(sp) * RAD2DEG;

    yaw   = atan2f(2.0f*(q[0]*q[3] + q[1]*q[2]), 1.0f - 2.0f*(q[2]*q[2] + q[3]*q[3])) * RAD2DEG;
}

// ======================== UMUMI SABITLER ========================
#define DEVICE_HEADER F("CC")
#define DEVICE_NAME   "DRONE (CC)"
#define LED_PIN         13
#define RF_SERIAL       Serial2
#define RF_BAUD         115200
#define GPS_SERIAL      Serial7
#define GPS_BAUD        9600
#define GPS_AGE_MAX_MS  3000UL
#define I2C_FREQ        400000UL
#define BNO055_PERIOD   10
#define BME280_PERIOD   40
#define AHT20_PERIOD    1000
#define GPS_PERIOD      200
#define PRINT_PERIOD    200
#define RF_PERIOD       66
#define FLIGHT_PERIOD   10
#define GPS_DEBUG_PERIOD 5000
#define SEA_LEVEL_HPA   1013.25f

// ======================== GPS DATA ========================
struct GPSData {
    float lat, lon, altitude, speed, course;
    uint8_t fix, satellites;
    bool updated;
    GPSData() : lat(0),lon(0),altitude(0),speed(0),course(0),fix(0),satellites(0),updated(false) {}
};
static GPSData gps;

static void gps_debug_dump() {
    Serial.print(F("[GPS] chars=")); Serial.print(tgps.charsProcessed());
    Serial.print(F(" fixSent=")); Serial.print(tgps.sentencesWithFix());
    Serial.print(F(" badCRC=")); Serial.print(tgps.failedChecksum());
    Serial.print(F(" | fix=")); Serial.print(gps.fix);
    Serial.print(F(" sats=")); Serial.print(gps.satellites);
    Serial.print(F(" lat=")); Serial.print(gps.lat, 5);
    Serial.print(F(" lon=")); Serial.print(gps.lon, 5);
    Serial.print(F(" alt=")); Serial.print(gps.altitude, 1);

    if (tgps.charsProcessed() == 0) {
        Serial.println(F("  << GPS-DEN HECH NE GELMIR! (Naqil/Baud yoxlayin)"));
    } else if (tgps.failedChecksum() > 10 && tgps.sentencesWithFix() == 0) {
        Serial.println(F("  << CHECKSUM XETASI! (Baud rate sehfdir)"));
    } else if (tgps.sentencesWithFix() == 0) {
        Serial.println(F("  << MESAJ VAR, FIX YOXDUR (Acik sema lazimdir)"));
    } else {
        Serial.println(F("  << OK"));
    }
}

// ======================== BNO055 EEPROM CALIBRATION ========================
#define BNO_CAL_EEPROM_ADDR   0
#define BNO_CAL_MAGIC         0xB0C0
#define BNO_CAL_VERSION       1

static bool bno_cal_saved = false;

static uint8_t checksum8(const uint8_t* data, size_t len) {
    uint8_t sum = 0;
    for (size_t i = 0; i < len; i++) sum += data[i];
    return sum;
}

static bool bno_load_calibration() {
    uint16_t magic = 0;
    uint8_t version = 0;
    uint8_t stored_cs = 0;

    EEPROM.get(BNO_CAL_EEPROM_ADDR + 0, magic);
    EEPROM.get(BNO_CAL_EEPROM_ADDR + 2, version);
    EEPROM.get(BNO_CAL_EEPROM_ADDR + 3, stored_cs);

    if (magic != BNO_CAL_MAGIC) return false;
    if (version != BNO_CAL_VERSION) return false;

    adafruit_bno055_offsets_t offsets;
    memset(&offsets, 0, sizeof(offsets));
    EEPROM.get(BNO_CAL_EEPROM_ADDR + 4, offsets);

    uint8_t buf[sizeof(offsets)];
    memcpy(buf, &offsets, sizeof(offsets));

    if (checksum8(buf, sizeof(buf)) != stored_cs) return false;

    bno.setSensorOffsets(offsets);
    return true;
}

static void bno_save_calibration() {
    adafruit_bno055_offsets_t offsets;
    memset(&offsets, 0, sizeof(offsets));
    bno.getSensorOffsets(offsets);

    uint8_t buf[sizeof(offsets)];
    memcpy(buf, &offsets, sizeof(offsets));
    uint8_t cs = checksum8(buf, sizeof(buf));

    EEPROM.put(BNO_CAL_EEPROM_ADDR + 0, (uint16_t)BNO_CAL_MAGIC);
    EEPROM.put(BNO_CAL_EEPROM_ADDR + 2, (uint8_t)BNO_CAL_VERSION);
    EEPROM.put(BNO_CAL_EEPROM_ADDR + 3, cs);
    EEPROM.put(BNO_CAL_EEPROM_ADDR + 4, offsets);

    bno_cal_saved = true;
}

// ======================== GLOBAL ========================
static struct{uint8_t bno055:1,bme280:1,aht20:1,gps_fix:1;} ok;
static AttitudeEKF ekf;
static uint32_t lastBno,lastBme,lastAht,lastGps,lastPrn,lastRf,lastGpsDbg;
static uint32_t lastFlight;
static uint32_t lastMag = 0;
static AltVel altvel;
static FlightCtrl flight;

static float ax,ay,az,gx,gy,gz,mx,my,mz;
static float bme_t,bme_p,bme_h,bme_a;
static float aht_t,aht_h;
static float mad_roll,mad_pitch,mad_yaw;

static uint32_t last_cal_check_ms = 0;
static uint32_t cal_good_start_ms = 0;

// ======================== BINARY RF PROTOCOL ========================
#define RF_PKT_SYNC1        0xAA
#define RF_PKT_SYNC2        0x55
#define RF_PROTO_VERSION    0x01
#define RF_DEVICE_ID        0xCC

#define RF_PKT_TELEM        0x01
#define RF_PKT_STATUS       0x02

#define FLAG_BNO_OK         0x0001
#define FLAG_BME_OK         0x0002
#define FLAG_AHT_OK         0x0004
#define FLAG_GPS_FIX        0x0008
#define FLAG_ARMED          0x0010
#define FLAG_MOTORS_ON      0x0020
#define FLAG_CAL_SAVED      0x0040

static uint16_t rf_seq = 0;

static uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) crc = (uint16_t)((crc << 1) ^ 0x1021);
            else crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void put_u8(uint8_t* buf, size_t &idx, uint8_t v) { buf[idx++] = v; }
static void put_u16(uint8_t* buf, size_t &idx, uint16_t v) { memcpy(buf + idx, &v, sizeof(v)); idx += sizeof(v); }
static void put_i16(uint8_t* buf, size_t &idx, int16_t v) { memcpy(buf + idx, &v, sizeof(v)); idx += sizeof(v); }
static void put_u32(uint8_t* buf, size_t &idx, uint32_t v) { memcpy(buf + idx, &v, sizeof(v)); idx += sizeof(v); }
static void put_i32(uint8_t* buf, size_t &idx, int32_t v) { memcpy(buf + idx, &v, sizeof(v)); idx += sizeof(v); }

static int16_t f_i16(float v, float scale) {
    if (isnan(v) || isinf(v)) return 0;
    float x = v * scale;
    if (x < -32768.0f) x = -32768.0f;
    if (x >  32767.0f) x =  32767.0f;
    return (int16_t)x;
}

static uint16_t f_u16(float v, float scale) {
    if (isnan(v) || isinf(v) || v < 0.0f) return 0;
    float x = v * scale;
    if (x > 65535.0f) x = 65535.0f;
    return (uint16_t)x;
}

static int32_t f_i32(float v, float scale) {
    if (isnan(v) || isinf(v)) return 0;
    double x = (double)v * (double)scale;
    if (x >  2147483647.0) x =  2147483647.0;
    if (x < -2147483648.0) x = -2147483648.0;
    return (int32_t)x;
}

static void rf_write_packet(uint8_t type, const uint8_t* payload, uint8_t len) {
    uint8_t buf[128];
    size_t i = 0;
    if (len > sizeof(buf) - 14) len = sizeof(buf) - 14;

    put_u8(buf, i, RF_PKT_SYNC1);
    put_u8(buf, i, RF_PKT_SYNC2);
    put_u8(buf, i, RF_PROTO_VERSION);
    put_u8(buf, i, RF_DEVICE_ID);
    put_u8(buf, i, type);
    put_u16(buf, i, rf_seq++);
    put_u32(buf, i, millis());
    put_u8(buf, i, len);

    if (len > 0) {
        memcpy(buf + i, payload, len);
        i += len;
    }

    uint16_t crc = crc16_ccitt(buf, i);
    put_u16(buf, i, crc);

    RF_SERIAL.write(buf, i);
}

static void rf_send_status_event() {
    uint8_t p[4];
    size_t i = 0;
    put_u8(p, i, rf_armed() ? 1 : 0);
    put_u8(p, i, flight.state_code());
    put_u16(p, i, flight.throttle());
    rf_write_packet(RF_PKT_STATUS, p, i);
}

static void rf_send_binary_telemetry() {
    uint8_t p[96];
    size_t i = 0;

    uint16_t flags = 0;
    if (ok.bno055) flags |= FLAG_BNO_OK;
    if (ok.bme280) flags |= FLAG_BME_OK;
    if (ok.aht20) flags |= FLAG_AHT_OK;
    if (ok.gps_fix) flags |= FLAG_GPS_FIX;
    if (rf_armed()) flags |= FLAG_ARMED;
    if (flight.state == FS_MOTORS_ON) flags |= FLAG_MOTORS_ON;
    if (bno_cal_saved) flags |= FLAG_CAL_SAVED;

    put_u16(p, i, flags);

    if (ok.bno055) {
        put_i16(p, i, f_i16(ax, 100.0f));
        put_i16(p, i, f_i16(ay, 100.0f));
        put_i16(p, i, f_i16(az, 100.0f));
        put_i16(p, i, f_i16(gx, 1000.0f));
        put_i16(p, i, f_i16(gy, 1000.0f));
        put_i16(p, i, f_i16(gz, 1000.0f));
        put_i16(p, i, f_i16(mx, 10.0f));
        put_i16(p, i, f_i16(my, 10.0f));
        put_i16(p, i, f_i16(mz, 10.0f));
    } else {
        for (uint8_t k = 0; k < 9; k++) put_i16(p, i, 0);
    }

    if (ok.bme280) {
        put_i16(p, i, f_i16(bme_t, 100.0f));
        put_u16(p, i, f_u16(bme_p, 10.0f));
        put_u16(p, i, f_u16(bme_h, 100.0f));
        put_i32(p, i, f_i32(bme_a, 100.0f));
    } else {
        put_i16(p, i, 0); put_u16(p, i, 0); put_u16(p, i, 0); put_i32(p, i, 0);
    }

    if (ok.aht20 && !isnan(aht_t)) {
        put_i16(p, i, f_i16(aht_t, 100.0f));
        put_u16(p, i, f_u16(aht_h, 100.0f));
    } else {
        put_i16(p, i, 0); put_u16(p, i, 0);
    }

    int32_t lat_e7 = 0, lon_e7 = 0, gps_alt_cm = 0;
    uint16_t gps_speed_cm_s = 0, gps_course_x100 = 0;
    uint8_t gps_sats = 0;

    if (ok.gps_fix) {
        lat_e7 = (int32_t)(gps.lat * 10000000.0);
        lon_e7 = (int32_t)(gps.lon * 10000000.0);
        gps_alt_cm = f_i32(gps.altitude, 100.0f);
        gps_speed_cm_s = f_u16(gps.speed, 100.0f);
        gps_course_x100 = f_u16(gps.course, 100.0f);
        gps_sats = gps.satellites;
    }

    put_i32(p, i, lat_e7);
    put_i32(p, i, lon_e7);
    put_i32(p, i, gps_alt_cm);
    put_u16(p, i, gps_speed_cm_s);
    put_u16(p, i, gps_course_x100);
    put_u8(p, i, gps_sats);

    put_i16(p, i, f_i16(mad_roll, 100.0f));
    put_i16(p, i, f_i16(mad_pitch, 100.0f));
    put_i16(p, i, f_i16(mad_yaw, 100.0f));

    put_i32(p, i, f_i32(altvel.rel_alt, 100.0f));
    put_i16(p, i, f_i16(altvel.vel, 100.0f));
    put_u16(p, i, f_u16(altvel.g_force, 1000.0f));
    put_i16(p, i, f_i16(altvel.dpdt, 1000.0f));

    put_u8(p, i, rf_armed() ? 1 : 0);
    put_u8(p, i, flight.state_code());
    put_u16(p, i, flight.throttle());

    rf_write_packet(RF_PKT_TELEM, p, i);
}

// ======================== BNO CALIBRATION MONITOR ========================
void bno_calibration_monitor(uint32_t now) {
    if (!ok.bno055) return;
    if (now - last_cal_check_ms < 1000) return;
    last_cal_check_ms = now;

    uint8_t sys, gyro, accel, mag;
    bno.getCalibration(&sys, &gyro, &accel, &mag);

    if (!bno_cal_saved && sys == 3 && gyro >= 2 && accel >= 2 && mag >= 2) {
        if (cal_good_start_ms == 0) {
            cal_good_start_ms = now;
        } else if (now - cal_good_start_ms >= 3000) {
            bno_save_calibration();
            Serial.println(F("[BNO] Kalibrasiya EEPROM-a avtomatik yazildi"));
        }
    } else {
        cal_good_start_ms = 0;
    }
}

// ======================== SETUP ========================
void setup(){
    pinMode(LED_PIN,OUTPUT);
    digitalWrite(LED_PIN,HIGH);
    esc_init();
    Serial.begin(115200);
    delay(200);
    Serial.println(F("\n=== TEENSY 4.1 " DEVICE_NAME " ==="));
    Serial.println(F("[REJIM] AVTOMATIK: 500m + Enish + +-5Derece -> ESC 1480us"));
    Serial.println(F("[RF] Default=ARMED | '0'=DISARM(ehtiyat) | '1'=ARM(aktiv)"));
    Serial.println(F("[RF] Binary Protocol Active (CRC16 + Sequence)"));
    Serial.println(F("[USB] 'c' = BNO kalibrasiyasini EEPROM-a yaz"));
    Serial.println(F("[USB] 'x' = EEPROM kalibrasiyasini sil"));

    rf_command_init();
    altvel.init();
    flight.init();
    RF_SERIAL.begin(RF_BAUD);

    Wire.begin();
    Wire.setClock(I2C_FREQ);

    // BNO055
    Serial.print(F("BNO055: "));
    if(!bno.begin()) {
        Serial.println(F("FAIL"));
        ok.bno055 = false;
    } else {
        ok.bno055 = true;
        bno.setExtCrystalUse(false);

        if (bno_load_calibration()) {
            bno_cal_saved = true;
            Serial.print(F("EEPROM calibration loaded, "));
        } else {
            Serial.print(F("No valid EEPROM cal, "));
        }

        delay(100);
        uint8_t sys, gyro, accel, mag;
        bno.getCalibration(&sys, &gyro, &accel, &mag);
        Serial.print(F("OK (Cal: ")); Serial.print(sys); Serial.print('/');
        Serial.print(gyro); Serial.print('/'); Serial.print(accel); Serial.print('/'); Serial.print(mag); Serial.println(F(")"));
    }

    // BME280
    Serial.print(F("BME280: "));
    if (!bme.begin(0x76, &Wire)) {
        Serial.println(F("FAIL"));
        ok.bme280 = false;
    } else {
        ok.bme280 = true;
        bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                        Adafruit_BME280::SAMPLING_X2,
                        Adafruit_BME280::SAMPLING_X16,
                        Adafruit_BME280::SAMPLING_X1,
                        Adafruit_BME280::FILTER_X16,
                        Adafruit_BME280::STANDBY_MS_0_5);
        Serial.println(F("25 Hz OK"));
        bme_t = bme.readTemperature();
        bme_a = bme.readAltitude(SEA_LEVEL_HPA);
    }

    // AHT20
    Serial.print(F("AHT20:  "));
    if (!aht.begin()) {
        Serial.println(F("FAIL/OFF"));
        ok.aht20 = false;
    } else {
        ok.aht20 = true;
        Serial.println(F("1 Hz OK"));
    }
    aht_t = 0.0f;
    aht_h = 0.0f;

    // GPS (TinyGPS++)
    GPS_SERIAL.begin(GPS_BAUD);
    Serial.print(F("GPS:    Serial7 @ ")); Serial.print(GPS_BAUD); Serial.println(F(" baud - TinyGPS++"));

    // EKF
    ekf.init(0.0f, 0.0f, 9.80665f);
    Serial.println(F("[FILTER] Attitude EKF initialized (Accel + Mag)"));

    Serial.print(F("[RF] Serial2 @ ")); Serial.print(RF_BAUD); Serial.println(F(" baud, Binary Protocol"));

    uint32_t now=millis();
    lastBno=lastBme=lastAht=lastGps=lastPrn=lastRf=lastGpsDbg=now;
    lastFlight=now;
    lastMag=now;
    digitalWrite(LED_PIN,LOW);
}

// ======================== LOOP ========================
void loop(){
    uint32_t now=millis();

    // USB commands: c = save cal, x = clear cal
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'c' || c == 'C') {
            if (ok.bno055) {
                bno_save_calibration();
                Serial.println(F("[BNO] Kalibrasiya EEPROM-a manual yazildi"));
            }
        } else if (c == 'x' || c == 'X') {
            uint16_t zero_magic = 0xFFFF;
            EEPROM.put(BNO_CAL_EEPROM_ADDR, zero_magic);
            bno_cal_saved = false;
            Serial.println(F("[BNO] EEPROM kalibrasiya bloku temizlendi"));
        }
    }

    rf_command_update();

    // GPS: TinyGPS++ feed (her loop-da, buffer itirmemek ucun)
    while (GPS_SERIAL.available() > 0) {
        tgps.encode(GPS_SERIAL.read());
    }

    // Ucus Nezaretcisi (100 Hz)
    if(now-lastFlight>=FLIGHT_PERIOD){
        lastFlight=now;
        float tilt=fmaxf(fabsf(mad_roll),fabsf(mad_pitch));
        bool level_ok = (tilt <= 5.0f);
        if(!ok.bno055){
            level_ok=false;
        }
        bool descending = (altvel.vel < 0.0f);

        uint8_t old_state = flight.state_code();
        flight.update(rf_armed(), level_ok, descending, altvel.rel_alt, now);
        uint8_t new_state = flight.state_code();

        if (old_state != new_state) {
            rf_send_status_event();
        }
    }

    // IMU / BNO055 (100 Hz)
    if(now-lastBno>=BNO055_PERIOD){
        lastBno=now;
        if(ok.bno055){
            sensors_event_t event;
            bno.getEvent(&event, Adafruit_BNO055::VECTOR_ACCELEROMETER);
            ax = event.acceleration.x;
            ay = event.acceleration.y;
            az = event.acceleration.z;

            bno.getEvent(&event, Adafruit_BNO055::VECTOR_GYROSCOPE);
            gx = event.gyro.x;
            gy = event.gyro.y;
            gz = event.gyro.z;

            bno.getEvent(&event, Adafruit_BNO055::VECTOR_MAGNETOMETER);
            mx = event.magnetic.x;
            my = event.magnetic.y;
            mz = event.magnetic.z;

            ekf.predict(gx, gy, gz, 0.01f);
            ekf.update(ax, ay, az);
            ekf.getEulerDeg(mad_roll, mad_pitch, mad_yaw);
        }
    }

    // Mag yaw correction (20 Hz)
    if (now - lastMag >= MAG_PERIOD) {
        lastMag = now;
        if (ok.bno055) {
            float g_norm = sqrtf(ax*ax + ay*ay + az*az);
            if (g_norm > 0.7f * GRAVITY && g_norm < 1.8f * GRAVITY) {
                ekf.updateMag(mx, my, mz);
            }
        }
    }

    // Baro / BME280 (25 Hz)
    if(now-lastBme>=BME280_PERIOD){
        lastBme=now;
        if(ok.bme280){
            bme_t = bme.readTemperature();
            bme_p = bme.readPressure() / 100.0f;
            bme_h = bme.readHumidity();
            bme_a = bme.readAltitude(SEA_LEVEL_HPA);

            altvel.update(bme_p, az, ax, ay, ok.bno055);
        }
    }

    // Temp / AHT20 (1 Hz)
    if(ok.aht20 && now-lastAht>=AHT20_PERIOD){
        lastAht=now;
        sensors_event_t humidity, temp;
        aht.getEvent(&humidity, &temp);
        aht_t = temp.temperature;
        aht_h = humidity.relative_humidity;
    }

    // GPS status yenileme (5 Hz) - TinyGPS++ obyektinden
    if (now - lastGps >= GPS_PERIOD) {
        lastGps = now;

        bool fresh = (tgps.location.age() < GPS_AGE_MAX_MS);

        if (tgps.location.isValid() && fresh) {
            gps.lat = (float)tgps.location.lat();
            gps.lon = (float)tgps.location.lng();
            gps.fix = 1;
        } else {
            gps.fix = 0;
        }

        if (tgps.altitude.isValid()) {
            gps.altitude = (float)tgps.altitude.meters();
        }
        if (tgps.speed.isValid()) {
            gps.speed = (float)tgps.speed.mps();
        }
        if (tgps.course.isValid()) {
            gps.course = (float)tgps.course.deg();
        }
        gps.satellites = (uint8_t)tgps.satellites.value();
        gps.updated = tgps.location.isUpdated();

        ok.gps_fix = (gps.fix > 0);
    }

    // GPS Debug Diaqnostika (5 saniyede bir)
    if (now - lastGpsDbg >= GPS_DEBUG_PERIOD) {
        lastGpsDbg = now;
        gps_debug_dump();
    }

    // BNO calibration monitor
    bno_calibration_monitor(now);

    // Status LED
    static bool led=false;
    if(now&0x200){
        if(!led){ digitalWrite(LED_PIN,HIGH); led=true; }
    }else{
        if(led){ digitalWrite(LED_PIN,LOW); led=false; }
    }

    // USB Serial Cixisi (5 Hz)
    if(now-lastPrn>=PRINT_PERIOD){
        lastPrn=now;
        Serial.print(now); Serial.print(' ');
        if(ok.bno055){
            Serial.print(F("A:")); Serial.print(ax,2); Serial.print(','); Serial.print(ay,2); Serial.print(','); Serial.print(az,2);
            Serial.print(F(" G:")); Serial.print(gx,3); Serial.print(','); Serial.print(gy,3); Serial.print(','); Serial.print(gz,3);
        } else {
            Serial.print(F("IMU:OFF"));
        }
        Serial.print(F(" | T:"));
        if(ok.bme280){ Serial.print(bme_t,1); Serial.print('/'); Serial.print(bme_a,1); }
        else { Serial.print(F("OFF")); }

        Serial.print(F(" | A:"));
        if(ok.aht20 && !isnan(aht_t)){ Serial.print(aht_t,1); Serial.print('/'); Serial.print(aht_h,1); }
        else { Serial.print(F("OFF")); }

        Serial.print(F(" | GPS:"));
        if(ok.gps_fix){ Serial.print(gps.lat,5); Serial.print(','); Serial.print(gps.lon,5); Serial.print(F(" sat:")); Serial.print(gps.satellites); }
        else { Serial.print(F("NO(chars:")); Serial.print(tgps.charsProcessed()); Serial.print(F(")")); }

        Serial.print(F(" | FLT:"));
        Serial.print(rf_armed()?F("ARM"):F("DISARM"));
        Serial.print('/'); Serial.print((int)flight.state_code());
        Serial.print(F(" alt=")); Serial.print(altvel.rel_alt,2);
        Serial.print(F(" vel=")); Serial.print(altvel.vel,2);
        Serial.print(F(" g=")); Serial.print(altvel.g_force,2);
        Serial.print(F(" pwm=")); Serial.print(flight.throttle());
        Serial.println();
    }

    // RF Binary Telemetry (15 Hz)
    if(now-lastRf>=RF_PERIOD){
        lastRf=now;
        rf_send_binary_telemetry();
    }
}