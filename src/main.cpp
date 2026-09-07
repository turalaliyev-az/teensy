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
static TinyGPSPlus tgps;

// ======================== GPS OBYEKTI ========================
struct GPSData {
    float lat, lon, altitude, speed, course;
    uint8_t fix, satellites;
    bool updated;
    GPSData() : lat(0),lon(0),altitude(0),speed(0),course(0),fix(0),satellites(0),updated(false) {}
};
static GPSData gps;

// ======================== UMUMI SABITLER ========================
#define DEVICE_HEADER F("CC")
#define DEVICE_NAME   "DRONE (CC)"
#define LED_PIN         13
#define RF_SERIAL       Serial2
#define RF_BAUD         115200

// QEYD: Serial6 istifadə edilir!
#define GPS_SERIAL      Serial6
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
#define SEA_LEVEL_HPA   1013.25f

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
        if (c == '1') _armed = true;
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
    uint16_t magic = 0; uint8_t version = 0, stored_cs = 0;
    EEPROM.get(BNO_CAL_EEPROM_ADDR + 0, magic);
    EEPROM.get(BNO_CAL_EEPROM_ADDR + 2, version);
    EEPROM.get(BNO_CAL_EEPROM_ADDR + 3, stored_cs);

    if (magic != BNO_CAL_MAGIC || version != BNO_CAL_VERSION) return false;

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

// ======================== ALT/VEL FILTER ========================
struct AltVel {
    float rel_alt;
    float vel;
    float g_force;
    float dpdt;

    void init();
    void update(float pressure_hpa, float accel_norm_ms2, float a_world_z_ms2, bool imu_ok);
    bool calibrated() const { return _calibrated; }

private:
    float _p0, _p_smooth, _prev_p, _dpdt_smooth, _a_smooth, _g_smooth;
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

void AltVel::update(float pressure_hpa, float accel_norm_ms2, float a_world_z_ms2, bool imu_ok) {
    if (isnan(pressure_hpa) || isinf(pressure_hpa)) return;
    if (isnan(accel_norm_ms2) || isinf(accel_norm_ms2)) accel_norm_ms2 = GRAVITY;
    if (isnan(a_world_z_ms2) || isinf(a_world_z_ms2)) a_world_z_ms2 = GRAVITY;

    uint32_t now_us = micros();
    float dt = (_last_us != 0) ? fminf(fmaxf((now_us - _last_us) * 1.0e-6f, 0.0f), 0.5f) : 0.0f;
    _last_us = now_us;

    float g_raw = imu_ok ? fmaxf((accel_norm_ms2 / GRAVITY), 0.0f) : 1.0f;
    _g_smooth = (_g_smooth <= 0.0f) ? g_raw : _g_smooth + G_ALPHA * (g_raw - _g_smooth);
    g_force = _g_smooth;

    if (!_have_prev) { _p_smooth = _prev_p = pressure_hpa; _have_prev = true; } 
    else { _p_smooth += P_ALPHA * (pressure_hpa - _p_smooth); }

    if (dt > 1.0e-4f) {
        float raw = (_p_smooth - _prev_p) / dt;
        _dpdt_smooth += DPDT_ALPHA * (raw - _dpdt_smooth);
    }
    _prev_p = _p_smooth; dpdt = _dpdt_smooth;

    if (!_calibrated) {
        if (_calib_start_ms == 0) _calib_start_ms = millis();
        _calib_sum += pressure_hpa; _calib_count++;
        rel_alt = 0.0f; vel = 0.0f;
        if (millis() - _calib_start_ms >= CALIB_MS && _calib_count >= CALIB_MIN_N) {
            _p0 = _calib_sum / (float)_calib_count;
            _p_smooth = _prev_p = _p0; _calibrated = true;
        }
        return;
    }

    float z = (_p0 > 1.0f) ? 44330.0f * (1.0f - powf(_p_smooth / _p0, 0.1903f)) : 0.0f;
    float a_vert = imu_ok ? fmaxf(fminf((a_world_z_ms2 - GRAVITY), 50.0f), -50.0f) : 0.0f;
    _a_smooth += A_ALPHA * (a_vert - _a_smooth);

    float g_dev = fabsf(g_raw - 1.0f);
    float dyn_factor = fminf(1.0f + 4.0f * g_dev * g_dev, 10.0f);
    float r_alt = fminf(R_ALT * (1.0f + 3.0f * g_dev * g_dev), 20.0f);
    float q_alt = Q_ALT * dyn_factor; float q_vel = Q_VEL * dyn_factor;

    float alt_p = rel_alt + vel * dt + 0.5f * _a_smooth * dt * dt;
    float vel_p = vel + _a_smooth * dt;
    float P00_p = _P00 + 2.0f * dt * _P01 + dt * dt * _P11 + q_alt;
    float P01_p = _P01 + dt * _P11;
    float P11_p = _P11 + q_vel;

    float S = P00_p + r_alt; float K0 = P00_p / S; float K1 = P01_p / S; float innov = z - alt_p;
    rel_alt = alt_p + K0 * innov; vel = vel_p + K1 * innov;
    _P00 = (1.0f - K0) * P00_p; _P01 = (1.0f - K0) * P01_p; _P11 = P11_p - K1 * P01_p;
}

// ======================== QUATERNION ROTATION & EKF ========================
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

struct AttitudeEKF {
    float q[4], b[3], P[7][7], _mag_norm_ref, _mag_ref[3];
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

#define RAD2DEG 57.2957795f
static void quat_norm(float q[4]) {
    float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n < 1e-8f) { q[0]=1.0f; q[1]=q[2]=q[3]=0.0f; return; }
    n = 1.0f / n; q[0]*=n; q[1]*=n; q[2]*=n; q[3]*=n;
}

void AttitudeEKF::init(float ax, float ay, float az) {
    float n = sqrtf(ax*ax + ay*ay + az*az); if (n < 1e-4f) n = 1.0f;
    ax /= n; ay /= n; az /= n;
    float axis_x = ay, axis_y = -ax, axis_z = 0.0f; float d = az;
    if (d > 0.999999f) { q[0]=1.0f; q[1]=q[2]=q[3]=0.0f; } 
    else if (d < -0.999999f) { q[0]=0.0f; q[1]=1.0f; q[2]=q[3]=0.0f; } 
    else {
        float ang = acosf(d); float s = sinf(ang * 0.5f);
        float an = fmaxf(sqrtf(axis_x*axis_x + axis_y*axis_y), 1e-8f);
        q[0] = cosf(ang * 0.5f); q[1] = s * axis_x / an; q[2] = s * axis_y / an; q[3] = s * axis_z / an;
    }
    b[0] = b[1] = b[2] = 0.0f;
    memset(P, 0, sizeof(P));
    P[0][0]=P[1][1]=P[2][2]=P[3][3]=0.1f; P[4][4]=P[5][5]=P[6][6]=0.1f;
    _mag_norm_ref = 0.0f; _mag_ref_valid = false;
    _mag_ref[0] = 1.0f; _mag_ref[1] = 0.0f; _mag_ref[2] = 0.0f;
}

void AttitudeEKF::predict(float gx, float gy, float gz, float dt) {
    if (dt < 1e-6f) return;
    float wx = gx - b[0], wy = gy - b[1], wz = gz - b[2];
    q[0] += 0.5f * (-wx*q[1] - wy*q[2] - wz*q[3])*dt;
    q[1] += 0.5f * ( wx*q[0] + wz*q[2] - wy*q[3])*dt;
    q[2] += 0.5f * ( wy*q[0] - wz*q[1] + wx*q[3])*dt;
    q[3] += 0.5f * ( wz*q[0] + wy*q[1] - wx*q[2])*dt;
    quat_norm(q);

    float F[7][7] = {0};
    F[0][0]=1.0f; F[0][1]=-0.5f*wx*dt; F[0][2]=-0.5f*wy*dt; F[0][3]=-0.5f*wz*dt;
    F[1][0]= 0.5f*wx*dt; F[1][1]=1.0f; F[1][2]= 0.5f*wz*dt; F[1][3]=-0.5f*wy*dt;
    F[2][0]= 0.5f*wy*dt; F[2][1]=-0.5f*wz*dt; F[2][2]=1.0f; F[2][3]= 0.5f*wx*dt;
    F[3][0]= 0.5f*wz*dt; F[3][1]= 0.5f*wy*dt; F[3][2]=-0.5f*wx*dt; F[3][3]=1.0f;
    F[0][4]= 0.5f*q[1]*dt; F[0][5]= 0.5f*q[2]*dt; F[0][6]= 0.5f*q[3]*dt;
    F[1][4]=-0.5f*q[0]*dt; F[1][5]= 0.5f*q[3]*dt; F[1][6]=-0.5f*q[2]*dt;
    F[2][4]=-0.5f*q[3]*dt; F[2][5]=-0.5f*q[0]*dt; F[2][6]= 0.5f*q[1]*dt;
    F[3][4]= 0.5f*q[2]*dt; F[3][5]=-0.5f*q[1]*dt; F[3][6]=-0.5f*q[0]*dt;
    F[4][4]=F[5][5]=F[6][6]=1.0f;

    float Q[7][7] = {0};
    float Xi[4][3] = {{-q[1], -q[2], -q[3]}, { q[0], -q[3],  q[2]}, { q[3],  q[0], -q[1]}, {-q[2],  q[1],  q[0]}};
    float s = 0.25f * (0.02f * 0.02f) * dt * dt;

    for (int i=0; i<4; i++) for (int j=0; j<4; j++) {
        float sum = 0.0f; for (int k=0; k<3; k++) sum += Xi[i][k] * Xi[j][k];
        Q[i][j] = s * sum;
    }
    Q[0][0]+=1e-9f; Q[1][1]+=1e-9f; Q[2][2]+=1e-9f; Q[3][3]+=1e-9f;
    Q[4][4]=Q[5][5]=Q[6][6]= (0.0005f * 0.0005f * dt);

    float FP[7][7], Pnew[7][7];
    for (int i=0; i<7; i++) for (int j=0; j<7; j++) {
        float sum = 0.0f; for (int k=0; k<7; k++) sum += F[i][k] * P[k][j]; FP[i][j] = sum;
    }
    for (int i=0; i<7; i++) for (int j=0; j<7; j++) {
        float sum = 0.0f; for (int k=0; k<7; k++) sum += FP[i][k] * F[j][k]; Pnew[i][j] = sum + Q[i][j];
    }
    memcpy(P, Pnew, sizeof(P));
    symmetrize();
}

void AttitudeEKF::update(float ax, float ay, float az) {
    if (isnan(ax) || isinf(ax) || isnan(ay) || isinf(ay) || isnan(az) || isinf(az)) return;
    float amag = sqrtf(ax*ax + ay*ay + az*az);
    if (amag < 3.0f || amag > 25.0f) return;
    float meas[3] = {ax / amag, ay / amag, az / amag};
    float dev = fabsf(amag - GRAVITY) / GRAVITY;
    updateVectorMeasurement(meas, (const float[]){0.0f, 0.0f, 1.0f}, 0.003f + 2.0f * dev * dev);
}

void AttitudeEKF::updateMag(float mx, float my, float mz) {
    float mmag = sqrtf(mx*mx + my*my + mz*mz);
    if (mmag < 15.0f || mmag > 120.0f) return;
    _mag_norm_ref = (_mag_norm_ref <= 1.0f) ? mmag : _mag_norm_ref + 0.02f * (mmag - _mag_norm_ref);
    float dev = fabsf(mmag - _mag_norm_ref) / _mag_norm_ref;
    if (dev > 0.45f) return;

    float meas[3] = {mx / mmag, my / mmag, mz / mmag}, m_world[3];
    quat_rotate_vec(q, meas, m_world);
    float norm_ref = sqrtf(m_world[0]*m_world[0] + m_world[2]*m_world[2]);
    if (norm_ref < 0.2f) return;

    float ref_raw[3] = {m_world[0] / norm_ref, 0.0f, m_world[2] / norm_ref};
    if (!_mag_ref_valid) { memcpy(_mag_ref, ref_raw, sizeof(_mag_ref)); _mag_ref_valid = true; } 
    else {
        _mag_ref[0] += 0.05f * (ref_raw[0] - _mag_ref[0]);
        _mag_ref[2] += 0.05f * (ref_raw[2] - _mag_ref[2]);
        float rn = sqrtf(_mag_ref[0]*_mag_ref[0] + _mag_ref[2]*_mag_ref[2]);
        if (rn >= 0.2f) { _mag_ref[0] /= rn; _mag_ref[1] = 0.0f; _mag_ref[2] /= rn; }
    }
    updateVectorMeasurement(meas, _mag_ref, 0.02f + 0.5f * dev * dev);
}

void AttitudeEKF::getEulerDeg(float &roll, float &pitch, float &yaw) const {
    roll  = atan2f(2.0f*(q[0]*q[1] + q[2]*q[3]), 1.0f - 2.0f*(q[1]*q[1] + q[2]*q[2])) * RAD2DEG;
    pitch = asinf(fmaxf(fminf(2.0f*(q[0]*q[2] - q[3]*q[1]), 1.0f), -1.0f)) * RAD2DEG;
    yaw   = atan2f(2.0f*(q[0]*q[3] + q[1]*q[2]), 1.0f - 2.0f*(q[2]*q[2] + q[3]*q[3])) * RAD2DEG;
}

void AttitudeEKF::symmetrize() {
    for (int i=0; i<7; i++) if (isnan(P[i][i]) || isinf(P[i][i]) || P[i][i] < 1e-12f) P[i][i] = 1e-12f;
    for (int i=0; i<7; i++) for (int j=i+1; j<7; j++) {
        float avg = 0.5f * (P[i][j] + P[j][i]); P[i][j] = P[j][i] = isnan(avg) ? 0.0f : avg;
    }
}

void AttitudeEKF::updateVectorMeasurement(const float meas[3], const float ref[3], float r) {
    if (r <= 1e-9f) return;
    float h[3]; quat_rotate_vec_inv(q, ref, h);
    float inn0 = meas[0] - h[0], inn1 = meas[1] - h[1], inn2 = meas[2] - h[2];
    
    float H[3][7] = {0};
    float w = q[0], x = q[1], yy = q[2], zz = q[3], rx = ref[0], ry = ref[1], rz = ref[2];
    H[0][0] = 2.0f*w*rx  + 2.0f*zz*ry - 2.0f*yy*rz; H[0][1] = 2.0f*x*rx  + 2.0f*yy*ry + 2.0f*zz*rz; H[0][2] = -2.0f*yy*rx + 2.0f*x*ry - 2.0f*w*rz; H[0][3] = -2.0f*zz*rx + 2.0f*w*ry + 2.0f*x*rz;
    H[1][0] = -2.0f*zz*rx + 2.0f*w*ry + 2.0f*x*rz; H[1][1] = 2.0f*yy*rx  - 2.0f*x*ry + 2.0f*w*rz; H[1][2] = 2.0f*x*rx   + 2.0f*yy*ry + 2.0f*zz*rz; H[1][3] = -2.0f*w*rx  - 2.0f*zz*ry + 2.0f*yy*rz;
    H[2][0] = 2.0f*yy*rx - 2.0f*x*ry + 2.0f*w*rz; H[2][1] = 2.0f*zz*rx - 2.0f*w*ry - 2.0f*x*rz; H[2][2] = 2.0f*w*rx  + 2.0f*zz*ry - 2.0f*yy*rz; H[2][3] = 2.0f*x*rx  + 2.0f*yy*ry + 2.0f*zz*rz;

    float PHt[7][3], S[3][3];
    for (int i=0; i<7; i++) for (int j=0; j<3; j++) { float sum=0; for (int k=0; k<7; k++) sum += P[i][k] * H[j][k]; PHt[i][j] = sum; }
    for (int i=0; i<3; i++) for (int j=0; j<3; j++) { float sum=0; for (int k=0; k<7; k++) sum += H[i][k] * PHt[k][j]; S[i][j] = sum; }
    S[0][0] += r; S[1][1] += r; S[2][2] += r;

    float det = S[0][0]*(S[1][1]*S[2][2]-S[1][2]*S[2][1]) - S[0][1]*(S[1][0]*S[2][2]-S[1][2]*S[2][0]) + S[0][2]*(S[1][0]*S[2][1]-S[1][1]*S[2][0]);
    if (fabsf(det) < 1e-12f) return;
    float id = 1.0f / det; float Si[3][3];
    Si[0][0]=(S[1][1]*S[2][2]-S[1][2]*S[2][1])*id; Si[0][1]=(S[0][2]*S[2][1]-S[0][1]*S[2][2])*id; Si[0][2]=(S[0][1]*S[1][2]-S[0][2]*S[1][1])*id;
    Si[1][0]=(S[1][2]*S[2][0]-S[1][0]*S[2][2])*id; Si[1][1]=(S[0][0]*S[2][2]-S[0][2]*S[2][0])*id; Si[1][2]=(S[0][2]*S[1][0]-S[0][0]*S[1][2])*id;
    Si[2][0]=(S[1][0]*S[2][1]-S[1][1]*S[2][0])*id; Si[2][1]=(S[0][1]*S[2][0]-S[0][0]*S[2][1])*id; Si[2][2]=(S[0][0]*S[1][1]-S[0][1]*S[1][0])*id;

    float K[7][3];
    for (int i=0; i<7; i++) for (int j=0; j<3; j++) { float sum=0; for (int k=0; k<3; k++) sum += PHt[i][k] * Si[k][j]; K[i][j] = sum; }

    q[0] += K[0][0]*inn0 + K[0][1]*inn1 + K[0][2]*inn2; q[1] += K[1][0]*inn0 + K[1][1]*inn1 + K[1][2]*inn2;
    q[2] += K[2][0]*inn0 + K[2][1]*inn1 + K[2][2]*inn2; q[3] += K[3][0]*inn0 + K[3][1]*inn1 + K[3][2]*inn2;
    quat_norm(q);
    b[0] += K[4][0]*inn0 + K[4][1]*inn1 + K[4][2]*inn2; b[1] += K[5][0]*inn0 + K[5][1]*inn1 + K[5][2]*inn2; b[2] += K[6][0]*inn0 + K[6][1]*inn1 + K[6][2]*inn2;

    float KH[7][7], Pnew[7][7];
    for (int i=0; i<7; i++) for (int j=0; j<7; j++) { float sum=0; for (int k=0; k<3; k++) sum += K[i][k] * H[k][j]; KH[i][j] = sum; }
    for (int i=0; i<7; i++) for (int j=0; j<7; j++) { float sum=0; for (int k=0; k<7; k++) sum += ((i==k?1.0f:0.0f) - KH[i][k]) * P[k][j]; Pnew[i][j] = sum; }
    memcpy(P, Pnew, sizeof(P)); symmetrize();
}

// ======================== UCUS NEZARETCISI ========================
enum FlightState : uint8_t { 
    FS_DISARMED = 0, 
    FS_ASCENDING = 1, 
    FS_DESCENDING_ARMED = 2, 
    FS_LANDED = 3 
};

struct FlightCtrl {
    FlightState state;
    float max_alt;
    float throttle_us;

    void init() {
        state = FS_DISARMED;
        max_alt = 0.0f;
        throttle_us = ESC_US_OFF;
    }

    // rf_armed_flag: '1' or '0' received from ground station
    // level_ok: is drone within +-15 degrees?
    // rel_alt: current relative altitude in meters
    // vel: current vertical velocity (m/s)
    void update(bool rf_armed_flag, bool level_ok, float rel_alt, float vel) {
        if (!rf_armed_flag) {
            state = FS_DISARMED;
            throttle_us = ESC_US_OFF;
            esc_write_us((uint16_t)throttle_us);
            return;
        }

        if (state == FS_DISARMED) {
            state = FS_ASCENDING;
            max_alt = rel_alt;
        }

        if (state == FS_ASCENDING) {
            if (rel_alt > max_alt) max_alt = rel_alt;

            // Qalxış bitdi, zirvədən ən az 3 metr aşağı düşürük (Apex detection)
            // və sürət mənfidir (həqiqətən düşürük)
            if (max_alt > 5.0f && (max_alt - rel_alt > 3.0f) && vel < -0.5f) {
                state = FS_DESCENDING_ARMED; 
                // Sistem artıq ARM olundu!
            }
        }

        if (state == FS_DESCENDING_ARMED) {
            // Yere düşdükdə (hündürlük 1 metr və ondan az olduqda) matorları birdəfəlik bağla
            if (rel_alt <= 1.0f) {
                state = FS_LANDED;
            }
        }

        // MOTOR (ESC) İdarəetmə Məntiqi
        if (state == FS_DESCENDING_ARMED) {
            if (level_ok) {
                // Sırf +-15 dərəcə daxilində ESC-yə PWM (1480us) göndərir
                throttle_us = ESC_US_RUN;
            } else {
                // Bucaq pozulursa motorları söndürürük
                throttle_us = ESC_US_OFF;
            }
        } else {
            // Digər bütün hallarda (Qalxış, Yerə düşmə, Disarm) matorlar OFF
            throttle_us = ESC_US_OFF;
        }

        esc_write_us((uint16_t)throttle_us);
    }

    uint8_t state_code() const { return (uint8_t)state; }
    uint16_t throttle() const { return (uint16_t)throttle_us; }
};

// ======================== GLOBAL DEYISHENLER ========================
static struct { uint8_t bno055:1, bme280:1, aht20:1, gps_fix:1; } ok;
static AttitudeEKF ekf;
static AltVel altvel;
static FlightCtrl flight;

static float ax,ay,az,gx,gy,gz,mx,my,mz;
static float bme_t,bme_p,bme_h,bme_a;
static float aht_t,aht_h;
static float mad_roll,mad_pitch,mad_yaw;
static float fast_g = 1.0f;
static bool fast_g_valid = false;
#define FAST_G_ALPHA 0.45f

static uint32_t lastBno, lastBme, lastAht, lastGps, lastPrn, lastRf, lastGpsDbg;
static uint32_t lastMag = 0, lastFlight = 0, last_imu_us = 0;
static uint32_t last_cal_check_ms = 0, cal_good_start_ms = 0;

static uint8_t serial2_tx_buf[256];
static uint8_t serial2_rx_buf[128];

// ======================== GPS AUTO-SWAP MƏNTİQİ ========================
static uint32_t last_gps_rx_time = 0;
static bool gps_is_swapped = false;
static uint32_t last_gps_chars_processed = 0;

void gps_auto_swap_update() {
    if (tgps.charsProcessed() > last_gps_chars_processed) {
        last_gps_chars_processed = tgps.charsProcessed();
        last_gps_rx_time = millis(); // Siqnal gəlir, vaxtı yenilə
    }

    // Əgər 4 saniyə ərzində məlumat gəlməyibsə, UART RX/TX qütblərini dəyiş (Auto-Swap)
    if (millis() - last_gps_rx_time > 4000) {
        gps_is_swapped = !gps_is_swapped;
        last_gps_rx_time = millis();
        
        #if defined(ARDUINO_TEENSY41)
        // Serial6 Teensy 4.1-də LPUART1 modulu üzərindən işləyir
        // LPUART_CTRL_RXRTX registri RX və TX-i daxildən dəyişdirir.
        if (gps_is_swapped) {
            LPUART1_CTRL |= (1<<25); // LPUART_CTRL_RXRTX bitini yandır
        } else {
            LPUART1_CTRL &= ~(1<<25); // Biti söndür
        }
        Serial.println(F("[GPS] XƏTA: Məlumat gəlmir! RX/TX avtomatik dəyişdirildi. (Teensy Swap Register)"));
        #else
        Serial.println(F("[GPS] XƏTA: Məlumat gəlmir. Zəhmət olmasa RX/TX pinlərini yoxlayın."));
        #endif
    }
}

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
    if (isnan(v) || isinf(v)) return 0; float x = v * scale;
    return (int16_t)fmaxf(fminf(x, 32767.0f), -32768.0f);
}
static uint16_t f_u16(float v, float scale) {
    if (isnan(v) || isinf(v) || v < 0.0f) return 0; float x = v * scale;
    return (uint16_t)fminf(x, 65535.0f);
}
static int32_t f_i32(float v, float scale) {
    if (isnan(v) || isinf(v)) return 0; double x = (double)v * (double)scale;
    return (int32_t)fmax(fmin(x, 2147483647.0), -2147483648.0);
}

static void rf_write_packet(uint8_t type, const uint8_t* payload, uint8_t len) {
    uint8_t buf[128]; size_t i = 0; if (len > sizeof(buf) - 14) len = sizeof(buf) - 14;
    put_u8(buf, i, RF_PKT_SYNC1); put_u8(buf, i, RF_PKT_SYNC2); put_u8(buf, i, RF_PROTO_VERSION); put_u8(buf, i, RF_DEVICE_ID);
    put_u8(buf, i, type); put_u16(buf, i, rf_seq++); put_u32(buf, i, millis()); put_u8(buf, i, len);
    if (len > 0) { memcpy(buf + i, payload, len); i += len; }
    put_u16(buf, i, crc16_ccitt(buf, i));
    RF_SERIAL.write(buf, i);
}

static void rf_send_status_event() {
    uint8_t p[4]; size_t i = 0;
    put_u8(p, i, rf_armed() ? 1 : 0); put_u8(p, i, flight.state_code()); put_u16(p, i, flight.throttle());
    rf_write_packet(RF_PKT_STATUS, p, i);
}

static void rf_send_binary_telemetry() {
    uint8_t p[96]; size_t i = 0;
    uint16_t flags = 0;
    if (ok.bno055) flags |= FLAG_BNO_OK; if (ok.bme280) flags |= FLAG_BME_OK;
    if (ok.aht20) flags |= FLAG_AHT_OK; if (ok.gps_fix) flags |= FLAG_GPS_FIX;
    if (rf_armed()) flags |= FLAG_ARMED; if (flight.throttle_us > ESC_US_OFF) flags |= FLAG_MOTORS_ON;
    put_u16(p, i, flags);

    if (ok.bno055) {
        put_i16(p, i, f_i16(ax, 100.0f)); put_i16(p, i, f_i16(ay, 100.0f)); put_i16(p, i, f_i16(az, 100.0f));
        put_i16(p, i, f_i16(gx, 1000.0f)); put_i16(p, i, f_i16(gy, 1000.0f)); put_i16(p, i, f_i16(gz, 1000.0f));
        put_i16(p, i, f_i16(mx, 10.0f)); put_i16(p, i, f_i16(my, 10.0f)); put_i16(p, i, f_i16(mz, 10.0f));
    } else { for (uint8_t k = 0; k < 9; k++) put_i16(p, i, 0); }

    if (ok.bme280) {
        put_i16(p, i, f_i16(bme_t, 100.0f)); put_u16(p, i, f_u16(bme_p, 10.0f));
        put_u16(p, i, f_u16(bme_h, 100.0f)); put_i32(p, i, f_i32(bme_a, 100.0f));
    } else { put_i16(p, i, 0); put_u16(p, i, 0); put_u16(p, i, 0); put_i32(p, i, 0); }

    if (ok.aht20 && !isnan(aht_t)) { put_i16(p, i, f_i16(aht_t, 100.0f)); put_u16(p, i, f_u16(aht_h, 100.0f)); } 
    else { put_i16(p, i, 0); put_u16(p, i, 0); }

    int32_t lat_e7 = 0, lon_e7 = 0, gps_alt_cm = 0; uint16_t gps_speed_cm_s = 0, gps_course_x100 = 0; uint8_t gps_sats = 0;
    if (ok.gps_fix) {
        lat_e7 = (int32_t)(gps.lat * 10000000.0); lon_e7 = (int32_t)(gps.lon * 10000000.0);
        gps_alt_cm = f_i32(gps.altitude, 100.0f); gps_speed_cm_s = f_u16(gps.speed, 100.0f);
        gps_course_x100 = f_u16(gps.course, 100.0f); gps_sats = gps.satellites;
    }
    put_i32(p, i, lat_e7); put_i32(p, i, lon_e7); put_i32(p, i, gps_alt_cm);
    put_u16(p, i, gps_speed_cm_s); put_u16(p, i, gps_course_x100); put_u8(p, i, gps_sats);

    put_i16(p, i, f_i16(mad_roll, 100.0f)); put_i16(p, i, f_i16(mad_pitch, 100.0f)); put_i16(p, i, f_i16(mad_yaw, 100.0f));
    put_i32(p, i, f_i32(altvel.rel_alt, 100.0f)); put_i16(p, i, f_i16(altvel.vel, 100.0f));
    put_u16(p, i, f_u16(altvel.g_force, 1000.0f)); put_i16(p, i, f_i16(altvel.dpdt, 1000.0f));

    put_u8(p, i, rf_armed() ? 1 : 0); put_u8(p, i, flight.state_code()); put_u16(p, i, flight.throttle());
    rf_write_packet(RF_PKT_TELEM, p, i);
}

// ======================== GPS DEBUG ========================
static void gps_debug_dump() {
    Serial.print(F("[GPS] chars=")); Serial.print(tgps.charsProcessed());
    Serial.print(F(" fixSent=")); Serial.print(tgps.sentencesWithFix());
    Serial.print(F(" | fix=")); Serial.print(gps.fix);
    Serial.print(F(" sats=")); Serial.print(gps.satellites);
    Serial.print(F(" lat=")); Serial.print(gps.lat, 5);
    Serial.print(F(" lon=")); Serial.println(gps.lon, 5);
}

// ======================== SETUP ========================
void setup(){
    pinMode(LED_PIN,OUTPUT); digitalWrite(LED_PIN,HIGH);
    esc_init(); Serial.begin(115200); delay(200);
    
    Serial.println(F("\n=== TEENSY 4.1 " DEVICE_NAME " ==="));
    Serial.println(F("[UÇUŞ REJİMİ] Ascent -> Zirvə -> Düşüşdə ARM (Yalnız +-15 Dərəcə) -> Zəmin 0m'da STOP"));
    Serial.println(F("[GPS REJİMİ] Serial6 istifadə olunur (Auto RX/TX Swap Aktivdir)"));
    Serial.println(F("[RF] Binary Protocol Active"));

    rf_command_init(); altvel.init(); flight.init();
    
    Serial2.addMemoryForWrite(serial2_tx_buf, sizeof(serial2_tx_buf));
    Serial2.addMemoryForRead(serial2_rx_buf, sizeof(serial2_rx_buf));
    RF_SERIAL.begin(RF_BAUD); Wire.begin(); Wire.setClock(I2C_FREQ);

    Serial.print(F("BNO055: "));
    if(!bno.begin()) { Serial.println(F("FAIL")); ok.bno055 = false; } 
    else {
        ok.bno055 = true; bno.setExtCrystalUse(false);
        if (bno_load_calibration()) { Serial.print(F("EEPROM calibration loaded, ")); } 
        else { Serial.print(F("No valid EEPROM cal, ")); }
        delay(100);
        uint8_t sys, gyro, accel, mag; bno.getCalibration(&sys, &gyro, &accel, &mag);
        Serial.print(F("OK (Cal: ")); Serial.print(sys); Serial.print('/');
        Serial.print(gyro); Serial.print('/'); Serial.print(accel); Serial.print('/'); Serial.print(mag); Serial.println(F(")"));
    }

    Serial.print(F("BME280: "));
    if (!bme.begin(0x76, &Wire)) { Serial.println(F("FAIL")); ok.bme280 = false; } 
    else {
        ok.bme280 = true;
        bme.setSampling(Adafruit_BME280::MODE_NORMAL, Adafruit_BME280::SAMPLING_X2, Adafruit_BME280::SAMPLING_X16,
                        Adafruit_BME280::SAMPLING_X1, Adafruit_BME280::FILTER_X16, Adafruit_BME280::STANDBY_MS_0_5);
        Serial.println(F("25 Hz OK"));
        bme_t = bme.readTemperature(); bme_a = bme.readAltitude(SEA_LEVEL_HPA);
    }

    Serial.print(F("AHT20:  "));
    if (!aht.begin()) { Serial.println(F("FAIL/OFF")); ok.aht20 = false; } 
    else { ok.aht20 = true; Serial.println(F("1 Hz OK")); }
    aht_t = 0.0f; aht_h = 0.0f;

    // Serial6 (Teensy 4.1'də default olaraq Pin 25-RX, Pin 24-TX)
    GPS_SERIAL.begin(GPS_BAUD);
    Serial.print(F("GPS:    Serial6 @ ")); Serial.print(GPS_BAUD); Serial.println(F(" baud"));

    ekf.init(0.0f, 0.0f, 9.80665f);
    Serial.println(F("[FILTER] Attitude EKF initialized"));

    uint32_t now=millis();
    lastBno=lastBme=lastAht=lastGps=lastPrn=lastRf=lastGpsDbg=lastMag=lastFlight=last_gps_rx_time=now;
    digitalWrite(LED_PIN,LOW);
}

// ======================== LOOP ========================
void loop(){
    uint32_t now=millis();
    rf_command_update();

    while (GPS_SERIAL.available() > 0) {
        tgps.encode(GPS_SERIAL.read());
    }
    
    // GPS Auto-Swap çağırışı
    gps_auto_swap_update();

    // Ucus Nezaretcisi (100 Hz)
    if(now-lastFlight>=FLIGHT_PERIOD){
        lastFlight=now;
        
        // Cihaz +-15 dərəcə daxilindədirmi?
        float tilt = fmaxf(fabsf(mad_roll), fabsf(mad_pitch));
        bool level_ok = (tilt <= 15.0f);
        if (!ok.bno055) level_ok = false;
        
        uint8_t old_state = flight.state_code();
        
        // Uçuş alqoritmini yeniləyirik
        flight.update(rf_armed(), level_ok, altvel.rel_alt, altvel.vel);
        
        uint8_t new_state = flight.state_code();
        if (old_state != new_state) {
            rf_send_status_event(); // Rejim dəyişdikdə Quru Stansiyasına (Yerə) məlumat ver
        }
    }

    // IMU / BNO055 (100 Hz)
    if(now-lastBno>=BNO055_PERIOD){
        lastBno=now;
        if(ok.bno055){
            sensors_event_t event;
            bno.getEvent(&event, Adafruit_BNO055::VECTOR_ACCELEROMETER);
            ax = event.acceleration.x; ay = event.acceleration.y; az = event.acceleration.z;
            bno.getEvent(&event, Adafruit_BNO055::VECTOR_GYROSCOPE);
            gx = event.gyro.x; gy = event.gyro.y; gz = event.gyro.z;
            bno.getEvent(&event, Adafruit_BNO055::VECTOR_MAGNETOMETER);
            mx = event.magnetic.x; my = event.magnetic.y; mz = event.magnetic.z;

            bool imu_finite = !(isnan(ax) || isinf(ax) || isnan(ay) || isinf(ay) || isnan(az) || isinf(az) ||
                                isnan(gx) || isinf(gx) || isnan(gy) || isinf(gy) || isnan(gz) || isinf(gz));

            if (imu_finite) {
                uint32_t now_us = micros();
                float dt_imu = (last_imu_us != 0) ? fminf(fmaxf((now_us - last_imu_us) * 1.0e-6f, 0.001f), 0.05f) : 0.01f;
                last_imu_us = now_us;

                float raw_g = sqrtf(ax*ax + ay*ay + az*az) / GRAVITY;
                if (isnan(raw_g) || isinf(raw_g)) raw_g = 1.0f;
                fast_g += FAST_G_ALPHA * (raw_g - fast_g);
                fast_g_valid = true;

                ekf.predict(gx, gy, gz, dt_imu);
                ekf.update(ax, ay, az);
                ekf.getEulerDeg(mad_roll, mad_pitch, mad_yaw);
            } else { fast_g_valid = false; }
        } else { fast_g_valid = false; }
    }

    // Mag yaw correction (20 Hz)
    if (now - lastMag >= 50) {
        lastMag = now;
        if (ok.bno055 && fast_g_valid && fast_g > 0.7f && fast_g < 1.8f) {
            ekf.updateMag(mx, my, mz);
        }
    }

    // Baro / BME280 (25 Hz)
    if(now-lastBme>=BME280_PERIOD){
        lastBme=now;
        if(ok.bme280){
            bme_t = bme.readTemperature(); bme_p = bme.readPressure() / 100.0f;
            bme_h = bme.readHumidity(); bme_a = bme.readAltitude(SEA_LEVEL_HPA);

            if (!isnan(bme_p)) {
                float accel_norm = GRAVITY, a_world_z = GRAVITY;
                if (ok.bno055 && fast_g_valid && !(isnan(ax)||isinf(ax)||isnan(ay)||isinf(ay)||isnan(az)||isinf(az))) {
                    accel_norm = sqrtf(ax*ax + ay*ay + az*az);
                    float v_world[3]; quat_rotate_vec(ekf.q, (const float[]){ax, ay, az}, v_world);
                    a_world_z = v_world[2];
                }
                altvel.update(bme_p, accel_norm, a_world_z, ok.bno055);
            }
        }
    }

    // Temp / AHT20 (1 Hz)
    if(ok.aht20 && now-lastAht>=AHT20_PERIOD){
        lastAht=now; sensors_event_t humidity, temp;
        aht.getEvent(&humidity, &temp); aht_t = temp.temperature; aht_h = humidity.relative_humidity;
    }

    // GPS Parsing State (5 Hz)
    if (now - lastGps >= GPS_PERIOD) {
        lastGps = now;
        bool fresh = (tgps.location.age() < GPS_AGE_MAX_MS);
        if (tgps.location.isValid() && fresh) {
            gps.lat = (float)tgps.location.lat(); gps.lon = (float)tgps.location.lng(); gps.fix = 1;
        } else { gps.fix = 0; }

        if (tgps.altitude.isValid()) gps.altitude = (float)tgps.altitude.meters();
        if (tgps.speed.isValid()) gps.speed = (float)tgps.speed.mps();
        if (tgps.course.isValid()) gps.course = (float)tgps.course.deg();
        gps.satellites = (uint8_t)tgps.satellites.value(); gps.updated = tgps.location.isUpdated();
        ok.gps_fix = (gps.fix > 0);
    }

    // BNO Kalibrasiyasına nəzarət (EEPROM-a yazmaq üçün)
    if (ok.bno055 && now - last_cal_check_ms >= 1000) {
        last_cal_check_ms = now;
        uint8_t sys, gyro, accel, mag; bno.getCalibration(&sys, &gyro, &accel, &mag);
        if (!bno_cal_saved && sys == 3 && gyro >= 2 && accel >= 2 && mag >= 2) {
            if (cal_good_start_ms == 0) cal_good_start_ms = now;
            else if (now - cal_good_start_ms >= 3000) {
                bno_save_calibration(); Serial.println(F("[BNO] Kalibrasiya EEPROM-a avtomatik yazildi"));
            }
        } else { cal_good_start_ms = 0; }
    }

    // Status LED
    static bool led=false;
    if(now&0x200){ if(!led){ digitalWrite(LED_PIN,HIGH); led=true; } }
    else { if(led){ digitalWrite(LED_PIN,LOW); led=false; } }

    // GPS Debug Info (5 saniyədə 1)
    if (now - lastGpsDbg >= 5000) {
        lastGpsDbg = now;
        gps_debug_dump();
    }

    // USB Serial Cixisi (5 Hz)
    if(now-lastPrn>=PRINT_PERIOD){
        lastPrn=now;
        Serial.print(now); Serial.print(' ');
        if(ok.bno055){
            Serial.print(F("A:")); Serial.print(ax,2); Serial.print(','); Serial.print(ay,2); Serial.print(','); Serial.print(az,2);
            Serial.print(F(" G:")); Serial.print(gx,3); Serial.print(','); Serial.print(gy,3); Serial.print(','); Serial.print(gz,3);
        } else { Serial.print(F("IMU:OFF")); }
        
        Serial.print(F(" | T:"));
        if(ok.bme280){ Serial.print(bme_t,1); Serial.print('/'); Serial.print(bme_a,1); } else { Serial.print(F("OFF")); }

        Serial.print(F(" | A:"));
        if(ok.aht20 && !isnan(aht_t)){ Serial.print(aht_t,1); Serial.print('/'); Serial.print(aht_h,1); } else { Serial.print(F("OFF")); }

        Serial.print(F(" | GPS:"));
        if(ok.gps_fix){ Serial.print(gps.lat,5); Serial.print(','); Serial.print(gps.lon,5); } else { Serial.print(F("NO")); }

        Serial.print(F(" | FLT:"));
        Serial.print(rf_armed()?F("ARM"):F("DISARM"));
        Serial.print('/'); Serial.print((int)flight.state_code());
        Serial.print(F(" alt=")); Serial.print(altvel.rel_alt,2);
        Serial.print(F(" vel=")); Serial.print(altvel.vel,2);
        Serial.print(F(" pwm=")); Serial.print(flight.throttle());
        Serial.println();
    }

    // RF Binary Telemetry (15 Hz)
    if(now-lastRf>=RF_PERIOD){
        lastRf=now;
        rf_send_binary_telemetry();
    }
}