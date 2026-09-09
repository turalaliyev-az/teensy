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
    // SƏNAYE STANDARTI: Koordinatlar dəqiqlik itməməsi üçün mütləq double olmalıdır!
    double lat, lon; 
    float altitude, speed, course;
    uint8_t fix, satellites;
    GPSData() : lat(0.0),lon(0.0),altitude(0.0f),speed(0.0f),course(0.0f),fix(0),satellites(0) {}
};
static GPSData gps;

// ======================== UMUMI SABITLER ========================
#define DEVICE_HEADER F("CC")
#define DEVICE_NAME   "DRONE (CC) - INDUSTRIAL PRO"
#define LED_PIN         13
#define RF_SERIAL       Serial2
#define RF_BAUD         115200

#define GPS_SERIAL      Serial6
#define GPS_BAUD        9600
#define GPS_AGE_MAX_MS  3000UL
#define MAX_UART_READS  64

#define I2C_FREQ        400000UL
#define BNO055_PERIOD   10   
#define BME280_PERIOD   40   
#define AHT20_PERIOD    1000 
#define GPS_PERIOD      200  
#define PRINT_PERIOD    200
#define RF_PERIOD       66   
#define FLIGHT_PERIOD   10   
#define SEA_LEVEL_HPA   1013.25f
#define GRAVITY         9.80665f

// ======================== ESC & SLEW RATE CONTROLLER ========================
#define ESC1_PIN        2
#define ESC2_PIN        3
#define ESC_PWM_FREQ    50.0f
#define ESC_US_MIN      1000
#define ESC_US_MAX      1500
#define ESC_US_OFF      1000
#define ESC_US_RUN      1480

// Saniyədə nə qədər PWM artsın/azalsın?
#define ESC_SLEW_RATE_US_PER_S 250.0f 

void esc_init();
void esc_write_us(float us1, float us2);
static float _current_esc1_us = ESC_US_OFF;
static float _current_esc2_us = ESC_US_OFF;
static bool _armed = true;

static uint32_t us_to_duty(float us) {
    us = fmaxf(fminf(us, ESC_US_MAX), ESC_US_MIN);
    return (uint32_t)us * 65536UL / 20000UL;
}

void esc_init() {
    pinMode(ESC1_PIN, OUTPUT);
    pinMode(ESC2_PIN, OUTPUT);
    analogWriteFrequency(ESC1_PIN, ESC_PWM_FREQ);
    analogWriteFrequency(ESC2_PIN, ESC_PWM_FREQ);
    analogWriteResolution(16);
    esc_write_us(ESC_US_OFF, ESC_US_OFF);
}

void esc_update_target(float target_us1, float target_us2, float dt) {
    if (!_armed) {
        _current_esc1_us = ESC_US_OFF;
        _current_esc2_us = ESC_US_OFF;
    } else {
        float step = ESC_SLEW_RATE_US_PER_S * dt;
        
        if (target_us1 > _current_esc1_us) {
            _current_esc1_us = fminf(_current_esc1_us + step, target_us1);
        } else {
            _current_esc1_us = fmaxf(_current_esc1_us - step, target_us1);
        }

        if (target_us2 > _current_esc2_us) {
            _current_esc2_us = fminf(_current_esc2_us + step, target_us2);
        } else {
            _current_esc2_us = fmaxf(_current_esc2_us - step, target_us2);
        }
    }

    if (_current_esc1_us <= ESC_US_OFF + 0.1f) _current_esc1_us = ESC_US_OFF;
    if (_current_esc2_us <= ESC_US_OFF + 0.1f) _current_esc2_us = ESC_US_OFF;

    analogWrite(ESC1_PIN, us_to_duty(_current_esc1_us));
    analogWrite(ESC2_PIN, us_to_duty(_current_esc2_us));
}

void esc_write_us(float us1, float us2) {
    _current_esc1_us = us1; _current_esc2_us = us2;
    analogWrite(ESC1_PIN, us_to_duty(us1));
    analogWrite(ESC2_PIN, us_to_duty(us2));
}

// ======================== RF EMRLERI ========================
void rf_command_update() {
    uint8_t reads = 0;
    while (Serial2.available() && reads++ < MAX_UART_READS) {
        char c = (char)Serial2.read();
        if (c == '1') _armed = true;
        else if (c == '0') _armed = false;
    }
}
bool rf_armed() { return _armed; }

// ======================== BNO055 EEPROM ========================
#define BNO_CAL_EEPROM_ADDR   0
#define BNO_CAL_MAGIC         0xB0C0
#define BNO_CAL_VERSION       1
static bool bno_cal_saved = false;

static uint8_t checksum8(const uint8_t* data, size_t len) {
    uint8_t sum = 0; for (size_t i = 0; i < len; i++) sum += data[i]; return sum;
}

static bool bno_load_calibration() {
    uint16_t magic = 0; uint8_t version = 0, stored_cs = 0;
    EEPROM.get(BNO_CAL_EEPROM_ADDR + 0, magic);
    EEPROM.get(BNO_CAL_EEPROM_ADDR + 2, version);
    EEPROM.get(BNO_CAL_EEPROM_ADDR + 3, stored_cs);

    if (magic != BNO_CAL_MAGIC || version != BNO_CAL_VERSION) return false;

    adafruit_bno055_offsets_t offsets;
    EEPROM.get(BNO_CAL_EEPROM_ADDR + 4, offsets);
    uint8_t buf[sizeof(offsets)]; memcpy(buf, &offsets, sizeof(offsets));
    
    if (checksum8(buf, sizeof(buf)) != stored_cs) return false;
    bno.setSensorOffsets(offsets);
    return true;
}

static void bno_save_calibration() {
    adafruit_bno055_offsets_t offsets; bno.getSensorOffsets(offsets);
    uint8_t buf[sizeof(offsets)]; memcpy(buf, &offsets, sizeof(offsets));
    uint8_t cs = checksum8(buf, sizeof(buf));

    EEPROM.put(BNO_CAL_EEPROM_ADDR + 0, (uint16_t)BNO_CAL_MAGIC);
    EEPROM.put(BNO_CAL_EEPROM_ADDR + 2, (uint8_t)BNO_CAL_VERSION);
    EEPROM.put(BNO_CAL_EEPROM_ADDR + 3, cs);
    EEPROM.put(BNO_CAL_EEPROM_ADDR + 4, offsets);
    bno_cal_saved = true;
}

// ======================== ALT/VEL FILTER ========================
struct AltVel {
    float rel_alt, vel, g_force, dpdt;
    void init();
    void update(float pressure_hpa, float accel_norm_ms2, float a_world_z_ms2, bool imu_ok);
    bool calibrated() const { return _calibrated; }
private:
    float _p0, _p_smooth, _prev_p, _dpdt_smooth, _a_smooth, _g_smooth;
    uint32_t _last_us;
    float _P00, _P01, _P11, _calib_sum;
    uint32_t _calib_start_ms, _calib_count;
    bool _calibrated;
};

void AltVel::init() {
    rel_alt = vel = g_force = dpdt = 0.0f;
    _p0 = 1013.25f; _p_smooth = _prev_p = _dpdt_smooth = _a_smooth = _g_smooth = 0.0f;
    _last_us = 0; _P00 = _P11 = 1.0f; _P01 = 0.0f;
    _calib_start_ms = _calib_sum = _calib_count = 0; _calibrated = false;
}

void AltVel::update(float pressure_hpa, float accel_norm_ms2, float a_world_z_ms2, bool imu_ok) {
    if (isnan(pressure_hpa) || isinf(pressure_hpa)) return;

    uint32_t now_us = micros();
    float dt = (_last_us != 0) ? fminf(fmaxf((now_us - _last_us) * 1.0e-6f, 0.001f), 0.1f) : 0.01f;
    _last_us = now_us;

    if (!_calibrated) {
        if (_calib_start_ms == 0) _calib_start_ms = millis();
        _calib_sum += pressure_hpa; _calib_count++;
        if (millis() - _calib_start_ms >= 2500UL && _calib_count >= 15) {
            _p0 = _calib_sum / (float)_calib_count;
            _p_smooth = _prev_p = _p0; _calibrated = true;
        }
        return;
    }

    float g_raw = imu_ok ? fmaxf((accel_norm_ms2 / GRAVITY), 0.0f) : 1.0f;
    _g_smooth += 0.15f * (g_raw - _g_smooth); g_force = _g_smooth;
    _p_smooth += 0.30f * (pressure_hpa - _p_smooth);
    _dpdt_smooth += 0.25f * (((_p_smooth - _prev_p) / dt) - _dpdt_smooth);
    _prev_p = _p_smooth; dpdt = _dpdt_smooth;

    float z = 44330.0f * (1.0f - powf(_p_smooth / _p0, 0.1903f));
    float a_vert = imu_ok ? fmaxf(fminf((a_world_z_ms2 - GRAVITY), 50.0f), -50.0f) : 0.0f;
    _a_smooth += 0.25f * (a_vert - _a_smooth);

    float dyn_factor = fminf(1.0f + 4.0f * fabsf(g_raw - 1.0f), 10.0f);
    float r_alt = 1.0f * dyn_factor; float q_alt = 0.05f * dyn_factor; float q_vel = 0.6f * dyn_factor;

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
    float tx = 2.0f * (qy * v[2] - qz * v[1]), ty = 2.0f * (qz * v[0] - qx * v[2]), tz = 2.0f * (qx * v[1] - qy * v[0]);
    out[0] = v[0] + qw * tx + (qy * tz - qz * ty); out[1] = v[1] + qw * ty + (qz * tx - qx * tz); out[2] = v[2] + qw * tz + (qx * ty - qy * tx);
}
static void quat_rotate_vec_inv(const float q[4], const float v[3], float out[3]) {
    float qinv[4] = {q[0], -q[1], -q[2], -q[3]}; quat_rotate_vec(qinv, v, out);
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

static void quat_norm(float q[4]) {
    float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n < 1e-8f) { q[0]=1.0f; q[1]=q[2]=q[3]=0.0f; return; }
    n = 1.0f / n; q[0]*=n; q[1]*=n; q[2]*=n; q[3]*=n;
}

void AttitudeEKF::init(float ax, float ay, float az) {
    float n = sqrtf(ax*ax + ay*ay + az*az); if (n < 1e-4f) n = 1.0f;
    ax /= n; ay /= n; az /= n;
    float d = az;
    if (d > 0.999999f) { q[0]=1.0f; q[1]=q[2]=q[3]=0.0f; } 
    else if (d < -0.999999f) { q[0]=0.0f; q[1]=1.0f; q[2]=q[3]=0.0f; } 
    else {
        float ang = acosf(d); float s = sinf(ang * 0.5f);
        float an = sqrtf(ay*ay + ax*ax);
        if (an < 1e-8f) an = 1e-8f; 
        q[0] = cosf(ang * 0.5f); q[1] = s * ay / an; q[2] = s * -ax / an; q[3] = 0.0f;
    }
    b[0] = b[1] = b[2] = 0.0f; memset(P, 0, sizeof(P));
    P[0][0]=P[1][1]=P[2][2]=P[3][3]=P[4][4]=P[5][5]=P[6][6]=0.1f;
    _mag_norm_ref = 0.0f; _mag_ref_valid = false;
    _mag_ref[0] = 1.0f; _mag_ref[1] = 0.0f; _mag_ref[2] = 0.0f;
}

void AttitudeEKF::predict(float gx, float gy, float gz, float dt) {
    if (dt < 1e-6f) return;
    float wx = gx - b[0], wy = gy - b[1], wz = gz - b[2];
    q[0] += 0.5f * (-wx*q[1] - wy*q[2] - wz*q[3])*dt; q[1] += 0.5f * ( wx*q[0] + wz*q[2] - wy*q[3])*dt;
    q[2] += 0.5f * ( wy*q[0] - wz*q[1] + wx*q[3])*dt; q[3] += 0.5f * ( wz*q[0] + wy*q[1] - wx*q[2])*dt;
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

    float Q[7][7] = {0}, Xi[4][3] = {{-q[1], -q[2], -q[3]}, { q[0], -q[3],  q[2]}, { q[3],  q[0], -q[1]}, {-q[2],  q[1],  q[0]}};
    float s = 0.25f * (0.02f * 0.02f) * dt * dt;
    for (int i=0; i<4; i++) for (int j=0; j<4; j++) { float sum = 0.0f; for (int k=0; k<3; k++) sum += Xi[i][k] * Xi[j][k]; Q[i][j] = s * sum; }
    Q[0][0]+=1e-9f; Q[1][1]+=1e-9f; Q[2][2]+=1e-9f; Q[3][3]+=1e-9f;
    Q[4][4]=Q[5][5]=Q[6][6]= (0.0005f * 0.0005f * dt);

    float FP[7][7], Pnew[7][7];
    for (int i=0; i<7; i++) for (int j=0; j<7; j++) { float sum = 0.0f; for (int k=0; k<7; k++) sum += F[i][k] * P[k][j]; FP[i][j] = sum; }
    for (int i=0; i<7; i++) for (int j=0; j<7; j++) { float sum = 0.0f; for (int k=0; k<7; k++) sum += FP[i][k] * F[j][k]; Pnew[i][j] = sum + Q[i][j]; }
    memcpy(P, Pnew, sizeof(P)); symmetrize();
}

void AttitudeEKF::update(float ax, float ay, float az) {
    float amag = sqrtf(ax*ax + ay*ay + az*az);
    if (amag < 3.0f || amag > 25.0f) return;
    float meas[3] = {ax / amag, ay / amag, az / amag}, dev = fabsf(amag - GRAVITY) / GRAVITY;
    updateVectorMeasurement(meas, (const float[]){0.0f, 0.0f, 1.0f}, 0.003f + 2.0f * dev * dev);
}

void AttitudeEKF::updateMag(float mx, float my, float mz) {
    float mmag = sqrtf(mx*mx + my*my + mz*mz);
    if (mmag < 15.0f || mmag > 120.0f) return;
    _mag_norm_ref = (_mag_norm_ref <= 1.0f) ? mmag : _mag_norm_ref + 0.02f * (mmag - _mag_norm_ref);
    if (fabsf(mmag - _mag_norm_ref) / _mag_norm_ref > 0.45f) return;

    float meas[3] = {mx / mmag, my / mmag, mz / mmag}, m_world[3]; quat_rotate_vec(q, meas, m_world);
    
    float norm_ref = sqrtf(m_world[0]*m_world[0] + m_world[1]*m_world[1]);
    if (norm_ref < 0.2f) return;

    float ref_raw[3] = {m_world[0] / norm_ref, m_world[1] / norm_ref, 0.0f};
    
    if (!_mag_ref_valid) { memcpy(_mag_ref, ref_raw, sizeof(_mag_ref)); _mag_ref_valid = true; } 
    else {
        _mag_ref[0] += 0.05f * (ref_raw[0] - _mag_ref[0]); 
        _mag_ref[1] += 0.05f * (ref_raw[1] - _mag_ref[1]); 
        float rn = sqrtf(_mag_ref[0]*_mag_ref[0] + _mag_ref[1]*_mag_ref[1]);
        if (rn >= 0.2f) { _mag_ref[0] /= rn; _mag_ref[1] /= rn; _mag_ref[2] = 0.0f; }
    }
    updateVectorMeasurement(meas, _mag_ref, 0.02f + 0.5f * powf(fabsf(mmag - _mag_norm_ref)/_mag_norm_ref, 2));
}

void AttitudeEKF::symmetrize() {
    for (int i=0; i<7; i++) {
        if (isnan(P[i][i]) || isinf(P[i][i]) || P[i][i] < 1e-12f) P[i][i] = 1e-12f;
        if (P[i][i] > 10.0f) P[i][i] = 10.0f; 
    }
    for (int i=0; i<7; i++) for (int j=i+1; j<7; j++) {
        float avg = 0.5f * (P[i][j] + P[j][i]); P[i][j] = P[j][i] = isnan(avg) ? 0.0f : avg;
    }
}

void AttitudeEKF::updateVectorMeasurement(const float meas[3], const float ref[3], float r) {
    if (r <= 1e-9f) return;
    float h[3]; quat_rotate_vec_inv(q, ref, h);
    float inn0 = meas[0] - h[0], inn1 = meas[1] - h[1], inn2 = meas[2] - h[2];
    
    float H[3][7] = {0}, w = q[0], x = q[1], yy = q[2], zz = q[3], rx = ref[0], ry = ref[1], rz = ref[2];
    H[0][0] = 2.0f*w*rx  + 2.0f*zz*ry - 2.0f*yy*rz; H[0][1] = 2.0f*x*rx  + 2.0f*yy*ry + 2.0f*zz*rz; H[0][2] = -2.0f*yy*rx + 2.0f*x*ry - 2.0f*w*rz; H[0][3] = -2.0f*zz*rx + 2.0f*w*ry + 2.0f*x*rz;
    H[1][0] = -2.0f*zz*rx + 2.0f*w*ry + 2.0f*x*rz; H[1][1] = 2.0f*yy*rx  - 2.0f*x*ry + 2.0f*w*rz; H[1][2] = 2.0f*x*rx   + 2.0f*yy*ry + 2.0f*zz*rz; H[1][3] = -2.0f*w*rx  - 2.0f*zz*ry + 2.0f*yy*rz;
    H[2][0] = 2.0f*yy*rx - 2.0f*x*ry + 2.0f*w*rz; H[2][1] = 2.0f*zz*rx - 2.0f*w*ry - 2.0f*x*rz; H[2][2] = 2.0f*w*rx  + 2.0f*zz*ry - 2.0f*yy*rz; H[2][3] = 2.0f*x*rx  + 2.0f*yy*ry + 2.0f*zz*rz;

    float PHt[7][3], S[3][3];
    for (int i=0; i<7; i++) for (int j=0; j<3; j++) { float sum=0; for (int k=0; k<7; k++) sum += P[i][k] * H[j][k]; PHt[i][j] = sum; }
    for (int i=0; i<3; i++) for (int j=0; j<3; j++) { float sum=0; for (int k=0; k<7; k++) sum += H[i][k] * PHt[k][j]; S[i][j] = sum; }
    S[0][0] += r; S[1][1] += r; S[2][2] += r;

    float det = S[0][0]*(S[1][1]*S[2][2]-S[1][2]*S[2][1]) - S[0][1]*(S[1][0]*S[2][2]-S[1][2]*S[2][0]) + S[0][2]*(S[1][0]*S[2][1]-S[1][1]*S[2][0]);
    if (fabsf(det) < 1e-12f) return;
    float id = 1.0f / det, Si[3][3];
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

#define RAD2DEG 57.2957795f
void AttitudeEKF::getEulerDeg(float &roll, float &pitch, float &yaw) const {
    roll  = atan2f(2.0f*(q[0]*q[1] + q[2]*q[3]), 1.0f - 2.0f*(q[1]*q[1] + q[2]*q[2])) * RAD2DEG;
    pitch = asinf(fmaxf(fminf(2.0f*(q[0]*q[2] - q[3]*q[1]), 1.0f), -1.0f)) * RAD2DEG;
    yaw   = atan2f(2.0f*(q[0]*q[3] + q[1]*q[2]), 1.0f - 2.0f*(q[2]*q[2] + q[3]*q[3])) * RAD2DEG;
}

// ======================== UCUS NEZARETCISI (STATE MACHINE) ========================
enum FlightState : uint8_t { 
    FS_STANDBY = 0,       
    FS_LAUNCHED = 1,      
    FS_DESCENDING = 2,    
    FS_LANDED = 3         
};

struct FlightCtrl {
    FlightState state;
    float max_alt, target_esc1, target_esc2;
    bool tilt_hysteresis_ok;
    
    uint32_t esc1_active_start_time; 
    uint32_t landing_steady_start;   

    void init() {
        state = FS_STANDBY; max_alt = 0.0f; 
        target_esc1 = target_esc2 = ESC_US_OFF;
        tilt_hysteresis_ok = true; 
        esc1_active_start_time = landing_steady_start = 0;
    }

    void update(float rel_alt, float vel, float tilt, float dt, uint32_t now) {
        if (!_armed) {
            state = FS_STANDBY; target_esc1 = target_esc2 = ESC_US_OFF;
            esc_update_target(target_esc1, target_esc2, dt); return;
        }

        if (tilt > 15.0f) tilt_hysteresis_ok = false;
        else if (tilt < 10.0f) tilt_hysteresis_ok = true;

        if (state == FS_STANDBY) {
            if (rel_alt > 0.2f && vel > 0.2f) { state = FS_LAUNCHED; max_alt = rel_alt; }
        } 
        else if (state == FS_LAUNCHED) {
            if (rel_alt > max_alt) max_alt = rel_alt;
            if (vel < -0.2f && (max_alt - rel_alt > 0.1f)) state = FS_DESCENDING;
        } 
        else if (state == FS_DESCENDING) {
            if (rel_alt <= 0.1f) state = FS_LANDED;
            if (fabsf(vel) < 0.3f) { 
                if (landing_steady_start == 0) landing_steady_start = now;
                else if (now - landing_steady_start > 500UL) state = FS_LANDED;
            } else { landing_steady_start = 0; }
        }

        if (state == FS_DESCENDING) {
            if (tilt_hysteresis_ok) {
                target_esc1 = ESC_US_RUN;
                if (esc1_active_start_time == 0) esc1_active_start_time = now;

                if (esc1_active_start_time != 0 && (now - esc1_active_start_time >= 1000)) {
                    target_esc2 = ESC_US_RUN;
                } else { target_esc2 = ESC_US_OFF; }

            // TƏHLÜKƏSİZLİK HƏLLİ: Slew Rate-in pozulmaması üçün taymer qorunur, yalnız motorlar müvəqqəti sönür
            } else {
                target_esc1 = ESC_US_OFF; target_esc2 = ESC_US_OFF;
                // esc1_active_start_time = 0; <--- SİLİNDİ: Taymer sıfırlanmır ki, dron düzələndə 1 saniyə daha gözləməsin.
            }
        } else {
            target_esc1 = ESC_US_OFF; target_esc2 = ESC_US_OFF;
            esc1_active_start_time = 0;
        }

        esc_update_target(target_esc1, target_esc2, dt);
    }

    uint8_t state_code() const { return (uint8_t)state; }
};

// ======================== GLOBAL ========================
static struct { uint8_t bno055:1, bme280:1, aht20:1, gps_fix:1; } ok;
static AttitudeEKF ekf; static AltVel altvel; static FlightCtrl flight;

static float ax,ay,az,gx,gy,gz,mx,my,mz, bme_t,bme_p,bme_h,bme_a, aht_t, aht_h; 
static float mad_roll,mad_pitch,mad_yaw, fast_g = 1.0f;
static bool fast_g_valid = false;

static uint32_t lastBno, lastBme, lastAht, lastGps, lastPrn, lastRf, lastGpsDbg, lastMag, lastFlight;
static uint32_t last_imu_us = 0, last_cal_check_ms = 0, cal_good_start_ms = 0;

static uint8_t serial2_tx_buf[256], serial2_rx_buf[128];

// ======================== GPS AUTO-SWAP ========================
static uint32_t last_gps_rx_time = 0;
static bool gps_is_swapped = false;
static uint32_t last_gps_chars_processed = 0;
static uint32_t system_boot_time = 0; // Boot vaxtı qeydiyyatı

void gps_auto_swap_update() {
    if (tgps.charsProcessed() > last_gps_chars_processed) {
        last_gps_chars_processed = tgps.charsProcessed();
        last_gps_rx_time = millis(); 
    }

    // TƏHLÜKƏSİZLİK HƏLLİ: Boot-dan sonra ilk 10 saniyə heç vaxt Swap etmə (GPS-in açılmasını gözlə)
    if (millis() - system_boot_time > 10000) {
        if (millis() - last_gps_rx_time > 4000) {
            gps_is_swapped = !gps_is_swapped;
            last_gps_rx_time = millis();
            
            #if defined(ARDUINO_TEENSY41)
            if (gps_is_swapped) LPUART1_CTRL |= (1<<25); else LPUART1_CTRL &= ~(1<<25);
            GPS_SERIAL.clear();
            Serial.println(F("[GPS] XƏTA: Siqnal yoxdur! RX/TX Hard-Swap edildi (Teensy)."));
            #endif
        }
    }
}

// ======================== RF PROTOKOL (Cüt CRC16) ========================
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
#define FLAG_IMPACT         0x0080

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
    return (int16_t)fmaxf(fminf(x, 32767.0f), -32768.0f);
}

static uint16_t f_u16(float v, float scale) {
    if (isnan(v) || isinf(v) || v < 0.0f) return 0;
    float x = v * scale;
    return (uint16_t)fminf(x, 65535.0f);
}

static int32_t f_i32(float v, float scale) {
    if (isnan(v) || isinf(v)) return 0;
    double x = (double)v * (double)scale;
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

static void rf_send_binary_telemetry() {
    uint8_t p[96]; size_t i = 0;
    uint16_t flags = 0;
    
    if (ok.bno055) flags |= FLAG_BNO_OK; 
    if (ok.bme280) flags |= FLAG_BME_OK;
    if (ok.aht20) flags |= FLAG_AHT_OK; 
    if (ok.gps_fix) flags |= FLAG_GPS_FIX;
    if (rf_armed()) flags |= FLAG_ARMED; 
    if (flight.state_code() == FS_DESCENDING) flags |= FLAG_MOTORS_ON;
    if (bno_cal_saved) flags |= FLAG_CAL_SAVED;
    
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

    if (ok.aht20 && !isnan(aht_t)) { 
        put_i16(p, i, f_i16(aht_t, 100.0f)); 
        put_u16(p, i, f_u16(aht_h, 100.0f)); 
    } else { put_i16(p, i, 0); put_u16(p, i, 0); }

    int32_t lat_e7 = 0, lon_e7 = 0, gps_alt_cm = 0; 
    uint16_t gps_speed_cm_s = 0, gps_course_x100 = 0; 
    uint8_t gps_sats = 0;
    
    if (ok.gps_fix || (gps.lat != 0.0 && gps.lon != 0.0)) {
        lat_e7 = (int32_t)(gps.lat * 10000000.0); lon_e7 = (int32_t)(gps.lon * 10000000.0);
        gps_alt_cm = f_i32(gps.altitude, 100.0f); gps_speed_cm_s = f_u16(gps.speed, 100.0f);
        gps_course_x100 = f_u16(gps.course, 100.0f); gps_sats = gps.satellites;
    }
    
    put_i32(p, i, lat_e7); put_i32(p, i, lon_e7); put_i32(p, i, gps_alt_cm);
    put_u16(p, i, gps_speed_cm_s); put_u16(p, i, gps_course_x100); put_u8(p, i, gps_sats);

    put_i16(p, i, f_i16(mad_roll, 100.0f)); put_i16(p, i, f_i16(mad_pitch, 100.0f)); put_i16(p, i, f_i16(mad_yaw, 100.0f));
    put_i32(p, i, f_i32(altvel.rel_alt, 100.0f)); put_i16(p, i, f_i16(altvel.vel, 100.0f));
    put_u16(p, i, f_u16(altvel.g_force, 1000.0f)); put_i16(p, i, f_i16(altvel.dpdt, 1000.0f));

    put_u8(p, i, rf_armed() ? 1 : 0); 
    put_u8(p, i, flight.state_code()); 
    put_u16(p, i, (uint16_t)_current_esc1_us);

    rf_write_packet(RF_PKT_TELEM, p, i);
}

// ======================== SETUP ========================
void setup(){
    system_boot_time = millis(); // Boot vaxtını yadda saxlayır
    
    pinMode(LED_PIN,OUTPUT); digitalWrite(LED_PIN,HIGH);
    esc_init(); Serial.begin(115200); delay(200);
    
    Serial.println(F("\n=== TEENSY 4.1 " DEVICE_NAME " ==="));
    Serial.println(F("[MOTOR MƏNTİQİ] Düşüş zamanı ESC1 xətti artır -> 1 saniyə sonra ESC2 artır"));
    Serial.println(F("[SON KƏSİM] Yerdən 1m hündürlükdə (və ya ağacın üstündə) motorlar birdəfəlik susur"));

    altvel.init(); flight.init();
    
    Serial2.addMemoryForWrite(serial2_tx_buf, sizeof(serial2_tx_buf));
    Serial2.addMemoryForRead(serial2_rx_buf, sizeof(serial2_rx_buf));
    RF_SERIAL.begin(RF_BAUD); Wire.begin(); Wire.setClock(I2C_FREQ);

    if(!bno.begin()) { ok.bno055 = false; } 
    else { ok.bno055 = true; bno.setExtCrystalUse(false); bno_load_calibration(); delay(50); }

    if (!bme.begin(0x76, &Wire)) { ok.bme280 = false; } 
    else {
        ok.bme280 = true;
        bme.setSampling(Adafruit_BME280::MODE_NORMAL, Adafruit_BME280::SAMPLING_X2, Adafruit_BME280::SAMPLING_X16,
                        Adafruit_BME280::SAMPLING_X1, Adafruit_BME280::FILTER_X16, Adafruit_BME280::STANDBY_MS_0_5);
        bme_t = bme.readTemperature(); bme_a = bme.readAltitude(SEA_LEVEL_HPA);
    }
    if (!aht.begin()) { ok.aht20 = false; } else { ok.aht20 = true; }

    GPS_SERIAL.begin(GPS_BAUD);
    ekf.init(0.0f, 0.0f, 9.80665f);

    uint32_t now=millis();
    lastBno=lastBme=lastAht=lastGps=lastPrn=lastRf=lastGpsDbg=lastMag=lastFlight=last_gps_rx_time=now;
    digitalWrite(LED_PIN,LOW);
}

// ======================== MAIN LOOP ========================
void loop(){
    uint32_t now=millis();
    rf_command_update();

    uint8_t reads = 0;
    while (GPS_SERIAL.available() > 0 && reads++ < MAX_UART_READS) {
        tgps.encode(GPS_SERIAL.read());
    }
    gps_auto_swap_update();

    // --- Uçuş Nəzarətçisi (100 Hz) ---
    if(now-lastFlight>=FLIGHT_PERIOD){
        float dt_flt = (now - lastFlight) * 1.0e-3f;
        lastFlight=now;
        
        float tilt = fmaxf(fabsf(mad_roll), fabsf(mad_pitch));
        uint8_t old_state = flight.state_code();
        
        flight.update(altvel.rel_alt, altvel.vel, tilt, dt_flt, now);
        
        if (old_state != flight.state_code()) {
            uint8_t p[2] = {(uint8_t)(_armed?1:0), flight.state_code()};
            rf_write_packet(RF_PKT_STATUS, p, 2);
        }
    }

    // --- IMU / BNO055 (100 Hz) ---
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

            bool imu_finite = !(isnan(ax)||isinf(ax)||isnan(ay)||isinf(ay)||isnan(az)||isinf(az)||isnan(gx)||isinf(gx));
            if (imu_finite) {
                uint32_t now_us = micros();
                float dt_imu = (last_imu_us != 0) ? fminf(fmaxf((now_us - last_imu_us) * 1.0e-6f, 0.001f), 0.05f) : 0.01f;
                last_imu_us = now_us;

                float raw_g = sqrtf(ax*ax + ay*ay + az*az) / GRAVITY;
                fast_g += 0.45f * ((isnan(raw_g) ? 1.0f : raw_g) - fast_g);
                fast_g_valid = true;

                ekf.predict(gx, gy, gz, dt_imu);
                ekf.update(ax, ay, az);
                ekf.getEulerDeg(mad_roll, mad_pitch, mad_yaw);
            } else { fast_g_valid = false; }
        }
    }

    // --- Mag yaw correction (20 Hz) ---
    if (now - lastMag >= 50) {
        lastMag = now;
        if (ok.bno055 && fast_g_valid && fast_g > 0.7f && fast_g < 1.8f) ekf.updateMag(mx, my, mz);
    }

    // --- Baro / BME280 (25 Hz) ---
    if(now-lastBme>=BME280_PERIOD){
        lastBme=now;
        if(ok.bme280){
            bme_t = bme.readTemperature(); bme_p = bme.readPressure() / 100.0f;
            bme_h = bme.readHumidity(); bme_a = bme.readAltitude(SEA_LEVEL_HPA);

            if (!isnan(bme_p)) {
                float accel_norm = GRAVITY, a_world_z = GRAVITY;
                if (ok.bno055 && fast_g_valid) {
                    accel_norm = sqrtf(ax*ax + ay*ay + az*az);
                    float v_world[3]; quat_rotate_vec(ekf.q, (const float[]){ax, ay, az}, v_world);
                    a_world_z = v_world[2];
                }
                altvel.update(bme_p, accel_norm, a_world_z, ok.bno055);
            }
        }
    }

    // --- Temp / AHT20 (1 Hz) ---
    if (ok.aht20 && now - lastAht >= AHT20_PERIOD) {
        lastAht = now;
        sensors_event_t humidity, temp;
        aht.getEvent(&humidity, &temp);
        aht_t = temp.temperature;
        aht_h = humidity.relative_humidity;
    }

    // --- GPS Parsing State (5 Hz) ---
    if (now - lastGps >= GPS_PERIOD) {
        lastGps = now;
        bool fresh = (tgps.location.age() < GPS_AGE_MAX_MS);
        if (tgps.location.isValid() && fresh) {
            gps.lat = tgps.location.lat(); 
            gps.lon = tgps.location.lng();
            gps.fix = 1;
        } else { gps.fix = 0; }

        if (tgps.altitude.isValid()) gps.altitude = (float)tgps.altitude.meters();
        if (tgps.speed.isValid()) gps.speed = (float)tgps.speed.mps();
        if (tgps.course.isValid()) gps.course = (float)tgps.course.deg();
        gps.satellites = (uint8_t)tgps.satellites.value();
        ok.gps_fix = (gps.fix > 0);
    }

    // --- BNO Kalibrasiya EEPROM yazımı (1 Hz) ---
    if (ok.bno055 && now - last_cal_check_ms >= 1000) {
        last_cal_check_ms = now;
        uint8_t sys, gyro, accel, mag; bno.getCalibration(&sys, &gyro, &accel, &mag);
        if (!bno_cal_saved && sys == 3 && gyro >= 2 && accel >= 2 && mag >= 2) {
            if (cal_good_start_ms == 0) cal_good_start_ms = now;
            else if (now - cal_good_start_ms > 3000) bno_save_calibration();
        } else { cal_good_start_ms = 0; }
    }

    // --- USB Debug (5 Hz) ---
    if(now-lastPrn>=PRINT_PERIOD){
        lastPrn=now;
        Serial.print(now); Serial.print(F(" | ST:")); Serial.print((int)flight.state_code());
        Serial.print(F(" | Z:")); Serial.print(altvel.rel_alt, 1);
        Serial.print(F("m Vz:")); Serial.print(altvel.vel, 1);
        Serial.print(F("m/s | E1:")); Serial.print(_current_esc1_us, 0); 
        Serial.print(F(" E2:")); Serial.print(_current_esc2_us, 0); Serial.println(F("us"));
    }

    // --- RF Binary Telemetry (15 Hz) ---
    if (now - lastRf >= RF_PERIOD) {
        lastRf = now;
        rf_send_binary_telemetry();
    }
}