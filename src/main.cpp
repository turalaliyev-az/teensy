#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <EEPROM.h>
#include <Watchdog_t4.h> 
#include <arm_math.h>    

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BME280.h>
#include <Adafruit_AHTX0.h>
#include <TinyGPS++.h>

WDT_T4<WDT1> wdt; 

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BME280 bme;
Adafruit_AHTX0 aht;
static TinyGPSPlus tgps;

struct GPSData {
    double lat, lon;
    float altitude, speed, course, hdop;
    uint8_t fix, satellites, fix_quality;
    GPSData() : lat(0.0),lon(0.0),altitude(0.0f),speed(0.0f),course(0.0f),hdop(0.0f),fix(0),satellites(0),fix_quality(0) {}
};
static GPSData gps;

#define DEVICE_NAME     "DRONE (CC) - INDUSTRIAL PRO V3"
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

#define ESC1_PIN        15
#define ESC2_PIN        23
#define ESC_PWM_FREQ    50.0f
#define ESC_US_MIN      1000
#define ESC_US_MAX      2000
#define ESC_US_OFF      1000
#define ESC_US_RUN      1480
#define ESC_SLEW_RATE_US_PER_S 1500.0f

void esc_init();
void esc_write_us(float us1, float us2);
static float _current_esc1_us = ESC_US_OFF;
static float _current_esc2_us = ESC_US_OFF;
static bool _armed = false;

static uint32_t us_to_duty(float us) {
    us = fmaxf(fminf(us, ESC_US_MAX), ESC_US_MIN);
    return ((uint32_t)(us + 0.5f)) * 65536UL / 20000UL;
}

void esc_init() {
    pinMode(ESC1_PIN, OUTPUT); pinMode(ESC2_PIN, OUTPUT);
    analogWriteFrequency(ESC1_PIN, ESC_PWM_FREQ); analogWriteFrequency(ESC2_PIN, ESC_PWM_FREQ);
    analogWriteResolution(16);
    esc_write_us(ESC_US_OFF, ESC_US_OFF);
}

void esc_update_target(float target_us1, float target_us2, float dt) {
    if (!_armed) {
        _current_esc1_us = ESC_US_OFF; _current_esc2_us = ESC_US_OFF;
    } else {
        float step = ESC_SLEW_RATE_US_PER_S * dt;
        _current_esc1_us = (target_us1 > _current_esc1_us) ? fminf(_current_esc1_us + step, target_us1) : fmaxf(_current_esc1_us - step, target_us1);
        _current_esc2_us = (target_us2 > _current_esc2_us) ? fminf(_current_esc2_us + step, target_us2) : fmaxf(_current_esc2_us - step, target_us2);
    }
    if (_current_esc1_us <= ESC_US_OFF + 0.1f) _current_esc1_us = ESC_US_OFF;
    if (_current_esc2_us <= ESC_US_OFF + 0.1f) _current_esc2_us = ESC_US_OFF;

    analogWrite(ESC1_PIN, us_to_duty(_current_esc1_us));
    analogWrite(ESC2_PIN, us_to_duty(_current_esc2_us));
}
void esc_write_us(float us1, float us2) {
    _current_esc1_us = us1; _current_esc2_us = us2;
    analogWrite(ESC1_PIN, us_to_duty(us1)); analogWrite(ESC2_PIN, us_to_duty(us2));
}

#define RF_PROTO_VERSION     0x01
#define RF_DEVICE_ID         0xCC
#define RF_CMD_SYNC1         0xAA
#define RF_CMD_SYNC2         0x55
#define RF_CMD_ARM           0x01
#define RF_CMD_DISARM        0x00
#define RF_CMD_FORCE_LAUNCH  0x02  

static uint16_t crc16_ccitt(const uint8_t* data, size_t len); 
static uint8_t rf_cmd_state = 0, rf_cmd_buf[5], rf_cmd_idx = 0;
static bool _force_launch_cmd = false;
static uint8_t _rf_tx_seq = 0;

void rf_command_update() {
    uint8_t reads = 0;
    while (Serial2.available() && reads++ < MAX_UART_READS) {
        uint8_t c = (uint8_t)Serial2.read();
        switch (rf_cmd_state) {
            case 0: if (c == RF_CMD_SYNC1) rf_cmd_state = 1; break;
            case 1: rf_cmd_state = (c == RF_CMD_SYNC2) ? 2 : (c == RF_CMD_SYNC1 ? 1 : 0); break;
            case 2:
                rf_cmd_buf[rf_cmd_idx++] = c;
                if (rf_cmd_idx >= 5) {
                    rf_cmd_idx = 0;
                    if (rf_cmd_buf[0] == RF_PROTO_VERSION && rf_cmd_buf[1] == RF_DEVICE_ID && crc16_ccitt(rf_cmd_buf, 3) == (uint16_t)(rf_cmd_buf[3] | ((uint16_t)rf_cmd_buf[4] << 8))) {
                        if      (rf_cmd_buf[2] == RF_CMD_ARM)    _armed = true;
                        else if (rf_cmd_buf[2] == RF_CMD_DISARM) _armed = false;
                        else if (rf_cmd_buf[2] == RF_CMD_FORCE_LAUNCH && _armed) _force_launch_cmd = true;
                    }
                    rf_cmd_state = 0;
                }
                break;
        }
    }
}
bool rf_armed() { return _armed; }

#define BNO_CAL_EEPROM_ADDR   0
#define BNO_CAL_MAGIC         0xB0C0
#define BNO_CAL_VERSION       1
static bool bno_cal_saved = false;

static uint8_t checksum8(const uint8_t* data, size_t len) {
    uint8_t sum = 0; for (size_t i = 0; i < len; i++) sum += data[i]; return sum;
}
static bool bno_load_calibration() {
    uint16_t magic = 0; uint8_t version = 0, stored_cs = 0;
    EEPROM.get(BNO_CAL_EEPROM_ADDR + 0, magic); EEPROM.get(BNO_CAL_EEPROM_ADDR + 2, version); EEPROM.get(BNO_CAL_EEPROM_ADDR + 3, stored_cs);
    if (magic != BNO_CAL_MAGIC || version != BNO_CAL_VERSION) return false;

    adafruit_bno055_offsets_t offsets; EEPROM.get(BNO_CAL_EEPROM_ADDR + 4, offsets);
    uint8_t buf[sizeof(offsets)]; memcpy(buf, &offsets, sizeof(offsets));
    if (checksum8(buf, sizeof(buf)) != stored_cs) return false;
    bno.setSensorOffsets(offsets); return true;
}
static void bno_save_calibration() {
    adafruit_bno055_offsets_t offsets; bno.getSensorOffsets(offsets);
    uint8_t buf[sizeof(offsets)]; memcpy(buf, &offsets, sizeof(offsets));
    EEPROM.put(BNO_CAL_EEPROM_ADDR + 0, (uint16_t)BNO_CAL_MAGIC); EEPROM.put(BNO_CAL_EEPROM_ADDR + 2, (uint8_t)BNO_CAL_VERSION);
    EEPROM.put(BNO_CAL_EEPROM_ADDR + 3, checksum8(buf, sizeof(buf))); EEPROM.put(BNO_CAL_EEPROM_ADDR + 4, offsets);
    bno_cal_saved = true;
}

struct AltVel {
    float rel_alt, vel, g_force, dpdt;
    void init();
    void update(float pressure_hpa, float accel_norm_ms2, float a_world_z_ms2, bool imu_ok);
    void updateFromGPS(float alt_m, uint32_t now_ms);
private:
    float _p0, _p_smooth, _prev_p, _dpdt_smooth, _a_smooth, _g_smooth;
    uint32_t _last_us, _calib_start_ms, _calib_count;
    float _P00, _P01, _P11, _calib_sum;
    bool _calibrated, _gps_base_set;
    float _gps_base, _gps_prev_alt, _gps_prev_rel;
    uint32_t _gps_prev_ms;
};

void AltVel::init() {
    rel_alt = vel = g_force = dpdt = 0.0f; _p0 = 1013.25f; _p_smooth = _prev_p = _dpdt_smooth = _a_smooth = _g_smooth = 0.0f;
    _last_us = _calib_start_ms = _calib_count = _gps_prev_ms = 0; _P00 = _P11 = 1.0f; _P01 = _calib_sum = _gps_base = _gps_prev_alt = _gps_prev_rel = 0.0f;
    _calibrated = _gps_base_set = false;
}
void AltVel::update(float pressure_hpa, float accel_norm_ms2, float a_world_z_ms2, bool imu_ok) {
    if (isnan(pressure_hpa) || isinf(pressure_hpa)) return;
    uint32_t now_us = micros(); float dt = (_last_us != 0) ? fminf(fmaxf((now_us - _last_us) * 1.0e-6f, 0.001f), 0.1f) : 0.01f; _last_us = now_us;
    if (!_calibrated) {
        if (_calib_start_ms == 0) _calib_start_ms = millis();
        _calib_sum += pressure_hpa; _calib_count++;
        if (millis() - _calib_start_ms >= 2500UL && _calib_count >= 15) {
            _p0 = _calib_sum / (float)_calib_count; _p_smooth = _prev_p = _p0; _calibrated = true;
        } return;
    }
    float g_raw = imu_ok ? fmaxf((accel_norm_ms2 / GRAVITY), 0.0f) : 1.0f;
    _g_smooth += 0.15f * (g_raw - _g_smooth); g_force = _g_smooth;
    _p_smooth += 0.60f * (pressure_hpa - _p_smooth); _dpdt_smooth += 0.50f * (((_p_smooth - _prev_p) / dt) - _dpdt_smooth); _prev_p = _p_smooth; dpdt = _dpdt_smooth;
    float z = 44330.0f * (1.0f - powf(_p_smooth / _p0, 0.1903f)), a_vert = imu_ok ? fmaxf(fminf((a_world_z_ms2 - GRAVITY), 50.0f), -50.0f) : 0.0f;
    _a_smooth += 0.50f * (a_vert - _a_smooth);
    float dyn_factor = fminf(1.0f + 4.0f * fabsf(g_raw - 1.0f), 10.0f);
    float alt_p = rel_alt + vel * dt + 0.5f * _a_smooth * dt * dt, vel_p = vel + _a_smooth * dt;
    float P00_p = _P00 + 2.0f * dt * _P01 + dt * dt * _P11 + (0.05f * dyn_factor), P01_p = _P01 + dt * _P11, P11_p = _P11 + (0.6f * dyn_factor);
    float S = P00_p + (0.3f * dyn_factor), K0 = P00_p / S, K1 = P01_p / S, innov = z - alt_p;
    rel_alt = alt_p + K0 * innov; vel = vel_p + K1 * innov;
    _P00 = (1.0f - K0) * P00_p; _P01 = (1.0f - K0) * P01_p; _P11 = P11_p - K1 * P01_p;
}
void AltVel::updateFromGPS(float alt_m, uint32_t now_ms) {
    if (isnan(alt_m) || isinf(alt_m)) return;
    if (!_gps_base_set) { _gps_base = alt_m; _gps_base_set = true; _gps_prev_alt = alt_m; _gps_prev_ms = now_ms; rel_alt = vel = 0.0f; return; }
    float dt = fminf(fmaxf((float)(now_ms - _gps_prev_ms) * 1.0e-3f, 0.05f), 1.0f);
    float new_rel = rel_alt + 0.15f * ((alt_m - _gps_base) - rel_alt);
    vel = 0.3f * vel + 0.7f * ((new_rel - _gps_prev_rel) / dt); rel_alt = _gps_prev_rel = new_rel; _gps_prev_alt = alt_m; _gps_prev_ms = now_ms;
}

static void quat_rotate_vec(const float q[4], const float v[3], float out[3]) {
    float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    float tx = 2.0f * (qy * v[2] - qz * v[1]), ty = 2.0f * (qz * v[0] - qx * v[2]), tz = 2.0f * (qx * v[1] - qy * v[0]);
    out[0] = v[0] + qw * tx + (qy * tz - qz * ty); out[1] = v[1] + qw * ty + (qz * tx - qx * tz); out[2] = v[2] + qw * tz + (qx * ty - qy * tx);
}
static void quat_rotate_vec_inv(const float q[4], const float v[3], float out[3]) { float qinv[4] = {q[0], -q[1], -q[2], -q[3]}; quat_rotate_vec(qinv, v, out); }

struct AttitudeEKF {
    float q[4], b[3], P[49], _mag_norm_ref, _mag_ref[3]; bool _mag_ref_valid; 
    void init(float ax, float ay, float az);
    void predict(float gx, float gy, float gz, float dt);
    void update(float ax, float ay, float az);
    void updateMag(float mx, float my, float mz, float gps_course, bool use_gps);
    void getEulerDeg(float &roll, float &pitch, float &yaw) const;
    void symmetrize();
private:
    void updateVectorMeasurement(const float meas[3], const float ref[3], float r);
};

static void quat_norm(float q[4]) {
    float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n < 1e-8f) { q[0]=1.0f; q[1]=q[2]=q[3]=0.0f; return; } n = 1.0f / n; q[0]*=n; q[1]*=n; q[2]*=n; q[3]*=n;
}

void AttitudeEKF::init(float ax, float ay, float az) {
    float n = sqrtf(ax*ax + ay*ay + az*az); if (n < 1e-4f) n = 1.0f; ax /= n; ay /= n; az /= n; float d = az;
    if (d > 0.999999f) { q[0]=1.0f; q[1]=q[2]=q[3]=0.0f; } else if (d < -0.999999f) { q[0]=0.0f; q[1]=1.0f; q[2]=q[3]=0.0f; }
    else { float ang = acosf(d), s = sinf(ang * 0.5f), an = fmaxf(sqrtf(ay*ay + ax*ax), 1e-8f); q[0] = cosf(ang * 0.5f); q[1] = s * ay / an; q[2] = s * -ax / an; q[3] = 0.0f; }
    b[0] = b[1] = b[2] = 0.0f; memset(P, 0, sizeof(P)); for (int i=0; i<7; i++) P[i*7 + i] = 0.1f;
    _mag_norm_ref = 0.0f; _mag_ref_valid = false; _mag_ref[0] = 1.0f; _mag_ref[1] = _mag_ref[2] = 0.0f;
}

void AttitudeEKF::predict(float gx, float gy, float gz, float dt) {
    if (dt < 1e-6f) return;
    float wx = gx - b[0], wy = gy - b[1], wz = gz - b[2];
    q[0] += 0.5f * (-wx*q[1] - wy*q[2] - wz*q[3])*dt; q[1] += 0.5f * ( wx*q[0] + wz*q[2] - wy*q[3])*dt;
    q[2] += 0.5f * ( wy*q[0] - wz*q[1] + wx*q[3])*dt; q[3] += 0.5f * ( wz*q[0] + wy*q[1] - wx*q[2])*dt;
    quat_norm(q);

    float F[49] = {0}; 
    F[0*7+0]=1.0f; F[0*7+1]=-0.5f*wx*dt; F[0*7+2]=-0.5f*wy*dt; F[0*7+3]=-0.5f*wz*dt;
    F[1*7+0]= 0.5f*wx*dt; F[1*7+1]=1.0f; F[1*7+2]= 0.5f*wz*dt; F[1*7+3]=-0.5f*wy*dt;
    F[2*7+0]= 0.5f*wy*dt; F[2*7+1]=-0.5f*wz*dt; F[2*7+2]=1.0f; F[2*7+3]= 0.5f*wx*dt;
    F[3*7+0]= 0.5f*wz*dt; F[3*7+1]= 0.5f*wy*dt; F[3*7+2]=-0.5f*wx*dt; F[3*7+3]=1.0f;
    F[0*7+4]= 0.5f*q[1]*dt; F[0*7+5]= 0.5f*q[2]*dt; F[0*7+6]= 0.5f*q[3]*dt;
    F[1*7+4]=-0.5f*q[0]*dt; F[1*7+5]= 0.5f*q[3]*dt; F[1*7+6]=-0.5f*q[2]*dt;
    F[2*7+4]=-0.5f*q[3]*dt; F[2*7+5]=-0.5f*q[0]*dt; F[2*7+6]= 0.5f*q[1]*dt;
    F[3*7+4]= 0.5f*q[2]*dt; F[3*7+5]=-0.5f*q[1]*dt; F[3*7+6]=-0.5f*q[0]*dt;
    F[4*7+4]=F[5*7+5]=F[6*7+6]=1.0f;

    float Q[49] = {0}, Xi[4][3] = {{-q[1], -q[2], -q[3]}, { q[0], -q[3],  q[2]}, { q[3],  q[0], -q[1]}, {-q[2],  q[1],  q[0]}};
    float s = 0.25f * (0.02f * 0.02f) * dt * dt;
    for (int i=0; i<4; i++) for (int j=0; j<4; j++) { float sum = 0.0f; for (int k=0; k<3; k++) sum += Xi[i][k] * Xi[j][k]; Q[i*7+j] = s * sum; }
    Q[0*7+0]+=1e-9f; Q[1*7+1]+=1e-9f; Q[2*7+2]+=1e-9f; Q[3*7+3]+=1e-9f; Q[4*7+4]=Q[5*7+5]=Q[6*7+6]= (0.0005f * 0.0005f * dt);

    arm_matrix_instance_f32 mat_F, mat_P, mat_Q, mat_FP, mat_F_T, mat_Pnew;
    float FP[49], F_T[49], Pnew[49];
    arm_mat_init_f32(&mat_F, 7, 7, F); arm_mat_init_f32(&mat_P, 7, 7, P);
    arm_mat_init_f32(&mat_FP, 7, 7, FP); arm_mat_init_f32(&mat_F_T, 7, 7, F_T);
    arm_mat_init_f32(&mat_Pnew, 7, 7, Pnew); arm_mat_init_f32(&mat_Q, 7, 7, Q);

    arm_mat_mult_f32(&mat_F, &mat_P, &mat_FP);       
    arm_mat_trans_f32(&mat_F, &mat_F_T);             
    arm_mat_mult_f32(&mat_FP, &mat_F_T, &mat_Pnew);  
    arm_mat_add_f32(&mat_Pnew, &mat_Q, &mat_Pnew);   

    memcpy(P, Pnew, sizeof(P)); symmetrize();
}

void AttitudeEKF::update(float ax, float ay, float az) {
    float amag = sqrtf(ax*ax + ay*ay + az*az); if (amag < 3.0f || amag > 25.0f) return;
    float meas[3] = {ax / amag, ay / amag, az / amag}, dev = fabsf(amag - GRAVITY) / GRAVITY;
    updateVectorMeasurement(meas, (const float[]){0.0f, 0.0f, 1.0f}, 0.003f + 2.0f * dev * dev);
}

void AttitudeEKF::updateMag(float mx, float my, float mz, float gps_course, bool use_gps) {
    float mmag = sqrtf(mx*mx + my*my + mz*mz); if (mmag < 15.0f || mmag > 120.0f) return;
    _mag_norm_ref = (_mag_norm_ref <= 1.0f) ? mmag : _mag_norm_ref + 0.02f * (mmag - _mag_norm_ref);
    if (fabsf(mmag - _mag_norm_ref) / _mag_norm_ref > 0.45f) return;
    float meas[3] = {mx / mmag, my / mmag, mz / mmag}, m_world[3]; quat_rotate_vec(q, meas, m_world);
    float norm_ref = sqrtf(m_world[0]*m_world[0] + m_world[1]*m_world[1]); if (norm_ref < 0.2f) return;

    float ref_raw[3] = {m_world[0] / norm_ref, m_world[1] / norm_ref, 0.0f};
    if (use_gps) { float rad = gps_course * 0.0174533f; ref_raw[0] = cosf(rad); ref_raw[1] = sinf(rad); }

    if (!_mag_ref_valid) { memcpy(_mag_ref, ref_raw, sizeof(_mag_ref)); _mag_ref_valid = true; }
    else {
        _mag_ref[0] += 0.05f * (ref_raw[0] - _mag_ref[0]); _mag_ref[1] += 0.05f * (ref_raw[1] - _mag_ref[1]);
        float rn = sqrtf(_mag_ref[0]*_mag_ref[0] + _mag_ref[1]*_mag_ref[1]);
        if (rn >= 0.2f) { _mag_ref[0] /= rn; _mag_ref[1] /= rn; _mag_ref[2] = 0.0f; }
    }
    updateVectorMeasurement(meas, _mag_ref, 0.02f + 0.5f * powf(fabsf(mmag - _mag_norm_ref)/_mag_norm_ref, 2));
}

void AttitudeEKF::symmetrize() {
    for (int i=0; i<7; i++) {
        if (isnan(P[i*7+i]) || isinf(P[i*7+i]) || P[i*7+i] < 1e-12f) P[i*7+i] = 1e-12f;
        if (P[i*7+i] > 10.0f) P[i*7+i] = 10.0f;
    }
    for (int i=0; i<7; i++) for (int j=i+1; j<7; j++) {
        float avg = 0.5f * (P[i*7+j] + P[j*7+i]); P[i*7+j] = P[j*7+i] = isnan(avg) ? 0.0f : avg;
    }
}

// FULL CMSIS-DSP VECTOR UPDATE
void AttitudeEKF::updateVectorMeasurement(const float meas[3], const float ref[3], float r) {
    if (r <= 1e-9f) return; float h[3]; quat_rotate_vec_inv(q, ref, h);
    float inn0 = meas[0] - h[0], inn1 = meas[1] - h[1], inn2 = meas[2] - h[2];
    float H[3][7] = {0}, w = q[0], x = q[1], yy = q[2], zz = q[3], rx = ref[0], ry = ref[1], rz = ref[2];
    H[0][0] = 2.0f*w*rx+2.0f*zz*ry-2.0f*yy*rz; H[0][1] = 2.0f*x*rx+2.0f*yy*ry+2.0f*zz*rz; H[0][2] = -2.0f*yy*rx+2.0f*x*ry-2.0f*w*rz; H[0][3] = -2.0f*zz*rx+2.0f*w*ry+2.0f*x*rz;
    H[1][0] = -2.0f*zz*rx+2.0f*w*ry+2.0f*x*rz; H[1][1] = 2.0f*yy*rx-2.0f*x*ry+2.0f*w*rz; H[1][2] = 2.0f*x*rx+2.0f*yy*ry+2.0f*zz*rz; H[1][3] = -2.0f*w*rx-2.0f*zz*ry+2.0f*yy*rz;
    H[2][0] = 2.0f*yy*rx-2.0f*x*ry+2.0f*w*rz; H[2][1] = 2.0f*zz*rx-2.0f*w*ry-2.0f*x*rz; H[2][2] = 2.0f*w*rx+2.0f*zz*ry-2.0f*yy*rz; H[2][3] = 2.0f*x*rx+2.0f*yy*ry+2.0f*zz*rz;

    float PHt[7][3], S[3][3];
    for (int i=0; i<7; i++) for (int j=0; j<3; j++) { float sum=0; for (int k=0; k<7; k++) sum += P[i*7+k] * H[j][k]; PHt[i][j] = sum; }
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
    q[2] += K[2][0]*inn0 + K[2][1]*inn1 + K[2][2]*inn2; q[3] += K[3][0]*inn0 + K[3][1]*inn1 + K[3][2]*inn2; quat_norm(q);
    b[0] += K[4][0]*inn0 + K[4][1]*inn1 + K[4][2]*inn2; b[1] += K[5][0]*inn0 + K[5][1]*inn1 + K[5][2]*inn2; b[2] += K[6][0]*inn0 + K[6][1]*inn1 + K[6][2]*inn2;

    // YENİ: CMSIS-DSP P Matris Yenilənməsi P = (I - KH) * P
    float K_flat[21], H_flat[21];
    for (int i=0; i<7; i++) { K_flat[i*3]=K[i][0]; K_flat[i*3+1]=K[i][1]; K_flat[i*3+2]=K[i][2]; }
    for (int i=0; i<3; i++) { H_flat[i*7]=H[i][0]; H_flat[i*7+1]=H[i][1]; H_flat[i*7+2]=H[i][2]; H_flat[i*7+3]=H[i][3]; H_flat[i*7+4]=H[i][4]; H_flat[i*7+5]=H[i][5]; H_flat[i*7+6]=H[i][6]; }

    arm_matrix_instance_f32 mat_K, mat_H, mat_KH, mat_I, mat_I_KH, mat_P, mat_Pnew;
    float KH_flat[49], I_KH_flat[49], Pnew_flat[49];
    float I_flat[49] = {0}; for(int i=0; i<7; i++) I_flat[i*7+i] = 1.0f;

    arm_mat_init_f32(&mat_K, 7, 3, K_flat); arm_mat_init_f32(&mat_H, 3, 7, H_flat);
    arm_mat_init_f32(&mat_KH, 7, 7, KH_flat); arm_mat_init_f32(&mat_I, 7, 7, I_flat);
    arm_mat_init_f32(&mat_I_KH, 7, 7, I_KH_flat); arm_mat_init_f32(&mat_P, 7, 7, P);
    arm_mat_init_f32(&mat_Pnew, 7, 7, Pnew_flat);

    arm_mat_mult_f32(&mat_K, &mat_H, &mat_KH);         // KH = K * H
    arm_mat_sub_f32(&mat_I, &mat_KH, &mat_I_KH);       // I_KH = I - KH
    arm_mat_mult_f32(&mat_I_KH, &mat_P, &mat_Pnew);    // Pnew = (I - KH) * P

    memcpy(P, Pnew_flat, sizeof(P));
    symmetrize();
}

#define RAD2DEG 57.2957795f
void AttitudeEKF::getEulerDeg(float &roll, float &pitch, float &yaw) const {
    roll  = atan2f(2.0f*(q[0]*q[1] + q[2]*q[3]), 1.0f - 2.0f*(q[1]*q[1] + q[2]*q[2])) * RAD2DEG;
    pitch = asinf(fmaxf(fminf(2.0f*(q[0]*q[2] - q[3]*q[1]), 1.0f), -1.0f)) * RAD2DEG;
    yaw   = atan2f(2.0f*(q[0]*q[3] + q[1]*q[2]), 1.0f - 2.0f*(q[2]*q[2] + q[3]*q[3])) * RAD2DEG;
}

enum FlightState : uint8_t { FS_STANDBY = 0, FS_LAUNCHED = 1, FS_DESCENDING = 2, FS_LANDED = 3 };

struct FlightCtrl {
    FlightState state; float max_alt, target_esc1, target_esc2; bool tilt_hysteresis_ok;
    uint32_t esc1_active_start_time, landing_steady_start;

    void init() { state = FS_STANDBY; max_alt = target_esc1 = target_esc2 = ESC_US_OFF; tilt_hysteresis_ok = true; esc1_active_start_time = landing_steady_start = 0; }
    void update(float rel_alt, float vel, float fast_g_val, float tilt, float dt, uint32_t now) {
        if (!_armed) { state = FS_STANDBY; target_esc1 = target_esc2 = ESC_US_OFF; esc_update_target(target_esc1, target_esc2, dt); return; }

        if (tilt > 60.0f) tilt_hysteresis_ok = false;
        else if (tilt < 45.0f && !tilt_hysteresis_ok) tilt_hysteresis_ok = true; 

        if (state == FS_STANDBY) {
            if ((rel_alt > 5.0f && vel > 2.0f) || _force_launch_cmd) { state = FS_LAUNCHED; max_alt = rel_alt; _force_launch_cmd = false; }
        }
        else if (state == FS_LAUNCHED) {
            if (rel_alt > max_alt) max_alt = rel_alt;
            if (vel < -1.0f && (max_alt - rel_alt > 2.0f)) state = FS_DESCENDING;
        }
        else if (state == FS_DESCENDING) {
            if (rel_alt <= 1.0f || fast_g_val > 4.0f) { state = FS_LANDED; landing_steady_start = 0; }
            else if (fabsf(vel) < 0.3f) {
                if (landing_steady_start == 0) landing_steady_start = now;
                else if (now - landing_steady_start > 2000UL) { state = FS_LANDED; landing_steady_start = 0; }
            } else { landing_steady_start = 0; }
        }

        if (state == FS_DESCENDING) {
            if (tilt_hysteresis_ok) {
                target_esc1 = ESC_US_RUN;
                if (esc1_active_start_time == 0) esc1_active_start_time = now;
                target_esc2 = (esc1_active_start_time != 0 && (now - esc1_active_start_time >= 1000)) ? ESC_US_RUN : ESC_US_OFF;
            } else { target_esc1 = ESC_US_OFF; target_esc2 = ESC_US_OFF; }
        } else { target_esc1 = ESC_US_OFF; target_esc2 = ESC_US_OFF; esc1_active_start_time = 0; }
        esc_update_target(target_esc1, target_esc2, dt);
    }
    uint8_t state_code() const { return (uint8_t)state; }
};

static struct { uint8_t bno055:1, bme280:1, aht20:1, gps_fix:1; } ok;
static AttitudeEKF ekf; static AltVel altvel; static FlightCtrl flight;
static float ax,ay,az,gx,gy,gz,mx,my,mz, bme_t,bme_p,bme_h,bme_a, aht_t, aht_h;
static float mad_roll,mad_pitch,mad_yaw, fast_g = 1.0f;
static bool fast_g_valid = false, gps_alt_valid = false;
static uint32_t lastBno, lastBme, lastAht, lastGps, lastPrn, lastRf, lastMag, lastFlight, last_imu_us = 0, last_cal_check_ms = 0, cal_good_start_ms = 0;

static uint8_t serial2_tx_buf[256], serial2_rx_buf[128], serial6_tx_buf[64], serial6_rx_buf[256];
static void rf_send_binary_telemetry(); 
static void led_show_state(uint8_t st);

void setup(){
    pinMode(LED_PIN,OUTPUT); digitalWrite(LED_PIN,HIGH);
    esc_init(); Serial.begin(115200); delay(200);

    altvel.init(); flight.init();

    Serial2.addMemoryForWrite(serial2_tx_buf, sizeof(serial2_tx_buf)); Serial2.addMemoryForRead(serial2_rx_buf, sizeof(serial2_rx_buf));
    RF_SERIAL.begin(RF_BAUD); 
    
    Wire.begin(); Wire.setClock(I2C_FREQ); Wire.setTimeout(2);   
    Wire1.begin(); Wire1.setClock(I2C_FREQ); Wire1.setTimeout(2); 

    if(!bno.begin(OPERATION_MODE_IMUPLUS)) ok.bno055 = false;
    else { ok.bno055 = true; bno.setExtCrystalUse(false); bno_load_calibration(); delay(50); }

    if (!bme.begin(0x76, &Wire1)) ok.bme280 = false; 
    else {
        ok.bme280 = true;
        bme.setSampling(Adafruit_BME280::MODE_NORMAL, Adafruit_BME280::SAMPLING_X2, Adafruit_BME280::SAMPLING_X16, Adafruit_BME280::SAMPLING_X1, Adafruit_BME280::FILTER_X16, Adafruit_BME280::STANDBY_MS_0_5);
        bme_t = bme.readTemperature(); bme_a = bme.readAltitude(SEA_LEVEL_HPA);
    }
    if (!aht.begin()) ok.aht20 = false; else ok.aht20 = true;

    GPS_SERIAL.addMemoryForWrite(serial6_tx_buf, sizeof(serial6_tx_buf)); GPS_SERIAL.addMemoryForRead(serial6_rx_buf, sizeof(serial6_rx_buf));
    GPS_SERIAL.begin(GPS_BAUD);
    ekf.init(0.0f, 0.0f, 9.80665f);

    uint32_t now=millis(); lastBno=lastBme=lastAht=lastGps=lastPrn=lastRf=lastMag=lastFlight=now; 
    digitalWrite(LED_PIN,LOW);

    WDT_timings_t wdt_config;
    wdt_config.timeout = 1.024f; /* ~1024 ms timeout (WDT1 çözünürlüğü 0.5 sn) */
    wdt.begin(wdt_config); 
}

void loop(){
    wdt.feed(); 
    
    uint32_t now=millis(); rf_command_update();
    uint8_t reads = 0; while (GPS_SERIAL.available() > 0 && reads++ < MAX_UART_READS) tgps.encode(GPS_SERIAL.read());
    ok.gps_fix = (gps.fix > 0);

    if(now-lastFlight>=FLIGHT_PERIOD){
        float dt_flt = (now - lastFlight) * 1.0e-3f; lastFlight=now;
        float tilt = fmaxf(fabsf(mad_roll), fabsf(mad_pitch));
        flight.update(altvel.rel_alt, altvel.vel, fast_g, tilt, dt_flt, now);
    }

    if(now-lastBno>=BNO055_PERIOD){
        lastBno=now;
        if(ok.bno055){
            sensors_event_t event; bno.getEvent(&event, Adafruit_BNO055::VECTOR_ACCELEROMETER); ax = event.acceleration.x; ay = event.acceleration.y; az = event.acceleration.z;
            bno.getEvent(&event, Adafruit_BNO055::VECTOR_GYROSCOPE); gx = event.gyro.x; gy = event.gyro.y; gz = event.gyro.z;
            bno.getEvent(&event, Adafruit_BNO055::VECTOR_MAGNETOMETER); mx = event.magnetic.x; my = event.magnetic.y; mz = event.magnetic.z;
            bool imu_finite = !(isnan(ax)||isinf(ax)||isnan(ay)||isinf(ay)||isnan(az)||isinf(az)||isnan(gx)||isinf(gx));
            if (imu_finite) {
                uint32_t now_us = micros(); float dt_imu = (last_imu_us != 0) ? fminf(fmaxf((now_us - last_imu_us) * 1.0e-6f, 0.001f), 0.05f) : 0.01f; last_imu_us = now_us;
                float raw_g = sqrtf(ax*ax + ay*ay + az*az) / GRAVITY; fast_g += 0.45f * ((isnan(raw_g) ? 1.0f : raw_g) - fast_g); fast_g_valid = true;
                ekf.predict(gx, gy, gz, dt_imu); ekf.update(ax, ay, az); ekf.getEulerDeg(mad_roll, mad_pitch, mad_yaw);
            } else fast_g_valid = false;
        }
    }

    if (now - lastMag >= 50) {
        lastMag = now;
        if (ok.bno055 && fast_g_valid && fast_g > 0.7f && fast_g < 1.8f) ekf.updateMag(mx, my, mz, gps.course, (ok.gps_fix && gps.speed > 2.0f));
    }

    if(now-lastBme>=BME280_PERIOD){
        lastBme=now;
        if(ok.bme280){
            bme_t = bme.readTemperature(); bme_p = bme.readPressure() / 100.0f; bme_h = bme.readHumidity(); bme_a = bme.readAltitude(SEA_LEVEL_HPA);
            if (!isnan(bme_p)) {
                float accel_norm = GRAVITY, a_world_z = GRAVITY;
                if (ok.bno055 && fast_g_valid) { accel_norm = sqrtf(ax*ax + ay*ay + az*az); float v_world[3]; quat_rotate_vec(ekf.q, (const float[]){ax, ay, az}, v_world); a_world_z = v_world[2]; }
                altvel.update(bme_p, accel_norm, a_world_z, ok.bno055);
            }
        }
    }

    if (ok.aht20 && now - lastAht >= AHT20_PERIOD) {
        lastAht = now;
        sensors_event_t humidity, temp; aht.getEvent(&humidity, &temp);
        aht_t = temp.temperature; aht_h = humidity.relative_humidity;
    }

    if (now - lastGps >= GPS_PERIOD) {
        lastGps = now; bool fresh = (tgps.location.age() < GPS_AGE_MAX_MS);
        if (tgps.location.isValid() && fresh) {
            gps.lat = tgps.location.lat(); gps.lon = tgps.location.lng(); gps.fix = 1;
            gps.satellites = tgps.satellites.value();
            gps.hdop = (float)tgps.hdop.hdop();
            if (tgps.altitude.isValid()) { gps.altitude = (float)tgps.altitude.meters(); gps_alt_valid = true; }
            if (tgps.speed.isValid()) gps.speed = (float)tgps.speed.mps();
            if (tgps.course.isValid()) gps.course = (float)tgps.course.deg();
        } else gps.fix = 0;
        if (!ok.bme280 && gps_alt_valid && gps.fix) altvel.updateFromGPS(gps.altitude, now);
    }

    if (ok.bno055 && now - last_cal_check_ms >= 1000) {
        last_cal_check_ms = now;
        uint8_t sys, gyro, accel, mag; bno.getCalibration(&sys, &gyro, &accel, &mag);
        if (!bno_cal_saved && sys == 3 && gyro >= 2 && accel >= 2 && mag >= 2 && flight.state_code() == FS_STANDBY) {
            if (cal_good_start_ms == 0) cal_good_start_ms = now;
            else if (now - cal_good_start_ms > 3000) bno_save_calibration();
        } else cal_good_start_ms = 0;
    }

    if (now - lastRf >= RF_PERIOD) {
        lastRf = now;
        rf_send_binary_telemetry();
    }
    led_show_state(flight.state_code());
}

static uint16_t crc16_ccitt(const uint8_t* data, size_t len) { 
    uint16_t crc = 0xFFFF; 
    for (size_t i = 0; i < len; i++) { 
        crc ^= (uint16_t)data[i] << 8; 
        for (uint8_t bit = 0; bit < 8; bit++) { 
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1); 
        } 
    } 
    return crc; 
}

// =============================================================================
// RF BİNARY TELEMETRY PAKETİ - SƏNAYE FORMATI
// =============================================================================
// Paket uzunluğu: 40 bytes (CRC daxil)
// | Offset | Size | Məzmun                       |
// |--------|------|------------------------------|
// | 0      | 2    | Sync (0xAA, 0x55)            |
// | 2      | 1    | Protocol Version (0x01)      |
// | 3      | 1    | Device ID (0xCC)             |
// | 4      | 1    | Sequence Number              |
// | 5      | 1    | Flight State                 |
// | 6      | 1    | Sensor Status Flags          |
// | 7      | 2    | Roll (int16, 0.01 deg)       |
// | 9      | 2    | Pitch (int16, 0.01 deg)      |
// | 11     | 2    | Yaw (int16, 0.01 deg)        |
// | 13     | 4    | Rel Alt (int32, mm)          |
// | 17     | 2    | Velocity (int16, cm/s)       |
// | 19     | 4    | GPS Lat (int32, 1e-7 deg)    |
// | 23     | 4    | GPS Lon (int32, 1e-7 deg)    |
// | 27     | 2    | GPS Alt (int16, 0.1 m)       |
// | 29     | 2    | GPS Speed (int16, 0.1 m/s)   |
// | 31     | 2    | GPS Course (int16, 0.01 deg) |
// | 33     | 1    | GPS Satellites               |
// | 34     | 2    | G-force (int16, 0.01 G)      |
// | 36     | 2    | BME Pressure (uint16, 0.1 hPa)|
// | 38     | 2    | CRC16-CCITT (little-endian)  |
// =============================================================================

static void rf_send_binary_telemetry() {
    uint8_t pkt[40];
    uint8_t idx = 0;
    
    // Header
    pkt[idx++] = RF_CMD_SYNC1;
    pkt[idx++] = RF_CMD_SYNC2;
    pkt[idx++] = RF_PROTO_VERSION;
    pkt[idx++] = RF_DEVICE_ID;
    pkt[idx++] = ++_rf_tx_seq;
    pkt[idx++] = flight.state_code();
    
    // Sensor status: bit0=GPS fix, bit1=AHT20, bit2=BME280, bit3=BNO055, bit4=armed
    uint8_t status = (ok.gps_fix << 0) | (ok.aht20 << 1) | (ok.bme280 << 2) | (ok.bno055 << 3) | (_armed ? 0x10 : 0);
    pkt[idx++] = status;
    
    // Attitude (deg * 100)
    int16_t roll_i  = (int16_t)fmaxf(fminf(mad_roll  * 100.0f,  18000), -18000);
    int16_t pitch_i = (int16_t)fmaxf(fminf(mad_pitch * 100.0f,   9000),  -9000);
    int16_t yaw_i   = (int16_t)fmaxf(fminf(mad_yaw   * 100.0f,  18000), -18000);
    memcpy(&pkt[idx], &roll_i,  2); idx += 2;
    memcpy(&pkt[idx], &pitch_i, 2); idx += 2;
    memcpy(&pkt[idx], &yaw_i,   2); idx += 2;
    
    // Altitude (mm) və Velocity (cm/s)
    int32_t alt_mm = (int32_t)fmaxf(fminf(altvel.rel_alt * 1000.0f,  5000000), -5000000);
    int16_t vel_cms = (int16_t)fmaxf(fminf(altvel.vel * 100.0f,  32000), -32000);
    memcpy(&pkt[idx], &alt_mm, 4); idx += 4;
    memcpy(&pkt[idx], &vel_cms, 2); idx += 2;
    
    // GPS (lat, lon 1e-7 deg formatında)
    int32_t lat_i7 = (int32_t)(gps.lat * 1e7);
    int32_t lon_i7 = (int32_t)(gps.lon * 1e7);
    memcpy(&pkt[idx], &lat_i7, 4); idx += 4;
    memcpy(&pkt[idx], &lon_i7, 4); idx += 4;
    
    // GPS Alt, Speed, Course
    int16_t gps_alt_01m = (int16_t)fmaxf(fminf(gps.altitude * 10.0f,  32000), -1000);
    int16_t gps_spd_01ms = (int16_t)fmaxf(fminf(gps.speed * 10.0f,  32000), 0);
    int16_t gps_crs_01d = (int16_t)fmaxf(fminf(gps.course * 100.0f,  36000), 0);
    memcpy(&pkt[idx], &gps_alt_01m, 2); idx += 2;
    memcpy(&pkt[idx], &gps_spd_01ms, 2); idx += 2;
    memcpy(&pkt[idx], &gps_crs_01d, 2); idx += 2;
    
    // GPS satellites
    pkt[idx++] = gps.satellites;
    
    // G-force (0.01 G)
    int16_t g_01 = (int16_t)fmaxf(fminf(fast_g * 100.0f,  32000), 0);
    memcpy(&pkt[idx], &g_01, 2); idx += 2;
    
    // BME Pressure (0.1 hPa)
    uint16_t pres_01 = (uint16_t)fmaxf(fminf(bme_p * 10.0f,  65535), 0);
    memcpy(&pkt[idx], &pres_01, 2); idx += 2;
    
    // CRC16 hesabla və əlavə et (little-endian)
    uint16_t crc = crc16_ccitt(pkt, idx);
    pkt[idx++] = (uint8_t)(crc & 0xFF);
    pkt[idx++] = (uint8_t)((crc >> 8) & 0xFF);
    
    RF_SERIAL.write(pkt, idx);
}

// =============================================================================
// LED VƏZİYYƏT GÖSTƏRİCİSİ
// =============================================================================
// FS_STANDBY    : 1 Hz yavaş yanıb-sönmə (sistem hazırdır, gözləyir)
// FS_LAUNCHED   : 8 Hz sürətli yanıb-sönmə (aktiv uçuş)
// FS_DESCENDING : 2 Hz orta yanıb-sönmə (paraşüt və ya eniş)
// FS_LANDED     : Sabit yanır 3 saniyə, sonra sönür (uçuş tamamlandı)
// =============================================================================

static void led_show_state(uint8_t st) {
    static uint32_t landed_off_time = 0;
    uint32_t t = millis();
    
    switch (st) {
        case FS_STANDBY:
            // 1 Hz (500ms ON, 500ms OFF)
            digitalWrite(LED_PIN, (t / 500) % 2);
            landed_off_time = 0;
            break;
            
        case FS_LAUNCHED:
            // 8 Hz (62.5ms ON, 62.5ms OFF) - aktiv uçuş göstəricisi
            digitalWrite(LED_PIN, (t / 62) % 2);
            landed_off_time = 0;
            break;
            
        case FS_DESCENDING:
            // 2 Hz (250ms ON, 250ms OFF)
            digitalWrite(LED_PIN, (t / 250) % 2);
            landed_off_time = 0;
            break;
            
        case FS_LANDED:
            // 3 saniyə sabit yan, sonra sön
            if (landed_off_time == 0) {
                landed_off_time = t + 3000UL;
                digitalWrite(LED_PIN, HIGH);
            } else if (t < landed_off_time) {
                digitalWrite(LED_PIN, HIGH);
            } else {
                digitalWrite(LED_PIN, LOW);
            }
            break;
            
        default:
            digitalWrite(LED_PIN, LOW);
            landed_off_time = 0;
            break;
    }
}