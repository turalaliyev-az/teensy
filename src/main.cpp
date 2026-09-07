#include <Arduino.h>
#include <Wire.h>
#include <math.h>

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

// ======================== GPS DATA (Funksiyalardan ƏVVƏL) ========================
struct GPSData {
    float lat, lon, altitude, speed, course;
    uint8_t fix, satellites;
    bool updated;
    GPSData() : lat(0),lon(0),altitude(0),speed(0),course(0),fix(0),satellites(0),updated(false) {}
};
static GPSData gps;

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
static bool _last_ack_armed = true;

#define RF_RX_BUF_SIZE 32
static char rf_rx_buf[RF_RX_BUF_SIZE];
static uint8_t rf_rx_idx = 0;

void rf_command_init() {
    _armed = true;
    _last_ack_armed = true;
    rf_rx_idx = 0;
}

void rf_command_update() {
    while (Serial2.available()) {
        char c = (char)Serial2.read();
        
        if (c == '\n' || c == '\r') {
            if (rf_rx_idx > 0) {
                rf_rx_buf[rf_rx_idx] = '\0';
                
                if (strcmp(rf_rx_buf, "ARM_ON") == 0) {
                    _armed = true;
                } else if (strcmp(rf_rx_buf, "ARM_OFF") == 0) {
                    _armed = false;
                }
                
                rf_rx_idx = 0;
            }
        } else if (rf_rx_idx < RF_RX_BUF_SIZE - 1) {
            rf_rx_buf[rf_rx_idx++] = c;
        } else {
            rf_rx_idx = 0;
        }
    }
    
    if (_armed != _last_ack_armed) {
        _last_ack_armed = _armed;
        Serial2.print(_armed ? F("ACK_ARM\n") : F("ACK_DISARM\n"));
    }
}

bool rf_armed() { return _armed; }

// ======================== HUNDURLUK + SURET (3-STATE KALMAN) ========================
// DÜZƏLİŞ: _P -> P_mat (Arduino makrosu ilə toqquşmaması üçün)
struct AltVel {
    float rel_alt;
    float vel;
    float accel_bias;
    float g_force;
    float dpdt;

    void init();
    void update(float pressure_hpa, const float q[4], float ax, float ay, float az, bool imu_ok);
    bool calibrated() const { return _calibrated; }

private:
    float _p0;
    float _p_smooth;
    float _prev_p;
    float _dpdt_smooth;
    float _g_smooth;
    uint32_t _last_us;
    bool _have_prev;
    
    float P_mat[3][3];  // DÜZƏLİŞ: _P -> P_mat
    float _Q[3];
    float _R_baro;
    
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
#define G_ALPHA      0.15f

void AltVel::init() {
    rel_alt = 0.0f;
    vel = 0.0f;
    accel_bias = 0.0f;
    g_force = 0.0f;
    dpdt = 0.0f;
    
    _p0 = 1013.25f;
    _p_smooth = 0.0f;
    _prev_p = 0.0f;
    _dpdt_smooth = 0.0f;
    _g_smooth = 0.0f;
    _last_us = 0;
    _have_prev = false;
    
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P_mat[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    _Q[0] = 0.01f;
    _Q[1] = 0.5f;
    _Q[2] = 0.001f;
    _R_baro = 1.0f;
    
    _calib_start_ms = 0;
    _calib_sum = 0.0f;
    _calib_count = 0;
    _calibrated = false;
}

void AltVel::update(float pressure_hpa, const float q[4], float ax, float ay, float az, bool imu_ok) {
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
        if (_calib_start_ms == 0) {
            _calib_start_ms = millis();
        }
        _calib_sum += pressure_hpa;
        _calib_count++;
        rel_alt = 0.0f;
        vel = 0.0f;
        accel_bias = 0.0f;
        
        if (millis() - _calib_start_ms >= CALIB_MS && _calib_count >= CALIB_MIN_N) {
            _p0 = _calib_sum / (float)_calib_count;
            _p_smooth = _p0;
            _prev_p = _p0;
            _calibrated = true;
            Serial.print(F("[ALT] Avtomatik kalibrasiya: P0="));
            Serial.print(_p0, 2);
            Serial.println(F(" hPa (sifir nokte)"));
        }
        return;
    }

    float z_baro = 0.0f;
    if (_p0 > 1.0f) {
        z_baro = 44330.0f * (1.0f - powf(_p_smooth / _p0, 0.1903f));
    }

    float a_world_z;
    if (imu_ok) {
        float R20 = 2.0f * (q[1]*q[3] - q[0]*q[2]);
        float R21 = 2.0f * (q[2]*q[3] + q[0]*q[1]);
        float R22 = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
        a_world_z = R20 * ax + R21 * ay + R22 * az;
    } else {
        a_world_z = GRAVITY;
    }
    
    float a_vert = a_world_z - GRAVITY;
    float a_corrected = a_vert - accel_bias;
    
    float alt_pred = rel_alt + vel*dt + 0.5f*a_corrected*dt*dt;
    float vel_pred = vel + a_corrected*dt;
    float bias_pred = accel_bias;
    
    float F[3][3];
    F[0][0] = 1.0f;  F[0][1] = dt;  F[0][2] = -0.5f*dt*dt;
    F[1][0] = 0.0f;  F[1][1] = 1.0f; F[1][2] = -dt;
    F[2][0] = 0.0f;  F[2][1] = 0.0f; F[2][2] = 1.0f;
    
    float P_pred[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 3; k++) {
                float Fk_sum = 0.0f;
                for (int l = 0; l < 3; l++) {
                    Fk_sum += F[k][l] * P_mat[l][j];
                }
                sum += F[i][k] * Fk_sum;
            }
            P_pred[i][j] = sum + ((i == j) ? _Q[i] : 0.0f);
        }
    }
    
    float H[3] = {1.0f, 0.0f, 0.0f};
    float S = P_pred[0][0] + _R_baro;
    
    float K[3];
    for (int i = 0; i < 3; i++) {
        K[i] = P_pred[i][0] / S;
    }
    
    float innov = z_baro - alt_pred;
    
    rel_alt = alt_pred + K[0] * innov;
    vel = vel_pred + K[1] * innov;
    accel_bias = bias_pred + K[2] * innov;
    
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 3; k++) {
                float I_minus_KH = ((i == k) ? 1.0f : 0.0f) - K[i] * H[k];
                sum += I_minus_KH * P_pred[k][j];
            }
            P_mat[i][j] = sum;  // DÜZƏLİŞ: _P -> P_mat
        }
    }
}

// ======================== UCUS NEZARETCISI ========================
enum FlightState : uint8_t { 
    FS_DISARMED = 0,
    FS_ARMED_WAIT = 1,
    FS_MOTORS_ON = 2,
    FS_MOTORS_HOLD = 3,
    FS_TOUCHDOWN = 4
};

struct FlightCtrl {
    FlightState state;
    float throttle_us;
    uint32_t ramp_start_ms;
    uint32_t touchdown_start_ms;

    void init();
    void update(bool armed, float tilt_deg, bool descending, float rel_alt, float vel, uint32_t now_ms);
    uint16_t throttle() const { return (uint16_t)throttle_us; }
    uint8_t state_code() const { return (uint8_t)state; }

private:
    uint32_t _last_ms;
};

#define ALT_TRIGGER_M     500.0f
#define TILT_ARM_DEG      5.0f
#define TILT_DISARM_DEG   15.0f
#define RAMP_MS           500UL
#define TOUCHDOWN_ALT_M   0.5f
#define TOUCHDOWN_VEL_MS  0.5f
#define TOUCHDOWN_MS      1000UL

void FlightCtrl::init() {
    state = FS_DISARMED;
    throttle_us = (float)ESC_US_OFF;
    ramp_start_ms = 0;
    touchdown_start_ms = 0;
    _last_ms = 0;
}

void FlightCtrl::update(bool armed, float tilt_deg, bool descending, float rel_alt, float vel, uint32_t now_ms) {
    uint32_t dt_ms = 0;
    if (_last_ms != 0) {
        dt_ms = now_ms - _last_ms;
        if (dt_ms > 100) dt_ms = 100;
    }
    _last_ms = now_ms;

    if (!armed) {
        state = FS_DISARMED;
        throttle_us = (float)ESC_US_OFF;
        ramp_start_ms = 0;
        touchdown_start_ms = 0;
        esc_write_us((uint16_t)throttle_us);
        return;
    }

    bool tilt_ok_arm    = (tilt_deg <= TILT_ARM_DEG);
    bool tilt_ok_hold   = (tilt_deg <= TILT_DISARM_DEG);

    switch (state) {
        case FS_DISARMED:
            state = FS_ARMED_WAIT;
            throttle_us = (float)ESC_US_OFF;
            touchdown_start_ms = 0;
            break;

        case FS_ARMED_WAIT:
            if (tilt_ok_arm && descending && (rel_alt <= ALT_TRIGGER_M)) {
                state = FS_MOTORS_ON;
                ramp_start_ms = now_ms;
            }
            break;

        case FS_MOTORS_ON:
            if ((rel_alt <= TOUCHDOWN_ALT_M) && (fabsf(vel) < TOUCHDOWN_VEL_MS)) {
                if (touchdown_start_ms == 0) {
                    touchdown_start_ms = now_ms;
                } else if ((now_ms - touchdown_start_ms) >= TOUCHDOWN_MS) {
                    state = FS_TOUCHDOWN;
                    throttle_us = (float)ESC_US_OFF;
                    esc_write_us((uint16_t)throttle_us);
                    break;
                }
            } else {
                touchdown_start_ms = 0;
            }
            
            if (!tilt_ok_hold) {
                state = FS_MOTORS_HOLD;
                throttle_us = (float)ESC_US_OFF;
                break;
            }
            
            if (throttle_us < (float)ESC_US_RUN) {
                uint32_t elapsed = now_ms - ramp_start_ms;
                float progress = (float)elapsed / (float)RAMP_MS;
                if (progress > 1.0f) progress = 1.0f;
                
                float eased = 1.0f - expf(-3.0f * progress);
                throttle_us = (float)ESC_US_OFF + ((float)ESC_US_RUN - (float)ESC_US_OFF) * eased;
                
                if (throttle_us > (float)ESC_US_RUN) {
                    throttle_us = (float)ESC_US_RUN;
                }
            }
            break;

        case FS_MOTORS_HOLD:
            if ((rel_alt <= TOUCHDOWN_ALT_M) && (fabsf(vel) < TOUCHDOWN_VEL_MS)) {
                if (touchdown_start_ms == 0) {
                    touchdown_start_ms = now_ms;
                } else if ((now_ms - touchdown_start_ms) >= TOUCHDOWN_MS) {
                    state = FS_TOUCHDOWN;
                    throttle_us = (float)ESC_US_OFF;
                    esc_write_us((uint16_t)throttle_us);
                    break;
                }
            } else {
                touchdown_start_ms = 0;
            }
            
            if (tilt_ok_arm && descending && (rel_alt <= ALT_TRIGGER_M)) {
                state = FS_MOTORS_ON;
                ramp_start_ms = now_ms;
            }
            break;

        case FS_TOUCHDOWN:
            throttle_us = (float)ESC_US_OFF;
            break;
    }
    
    esc_write_us((uint16_t)throttle_us);
}

// ======================== 7-DOVLETLI QUATERNION EKF ========================
struct AttitudeEKF {
    float q[4];
    float b[3];
    float P[7][7];

    void init(float ax, float ay, float az);
    void predict(float gx, float gy, float gz, float dt);
    void update(float ax, float ay, float az);
    void getEulerDeg(float &roll, float &pitch, float &yaw) const;

private:
    void enforceSymmetry();
};

#define RAD2DEG 57.29577951308232f
#define GYRO_NOISE  0.02f
#define BIAS_NOISE  0.0005f
#define R_BASE      0.003f
#define R_ADAPT     2.0f
#define ACC_MIN     3.0f
#define ACC_MAX     14.0f

static void quat_norm(float q[4]) {
    float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n < 1e-8f) {
        q[0]=1.0f; q[1]=q[2]=q[3]=0.0f;
        return;
    }
    n = 1.0f / n;
    q[0]*=n; q[1]*=n; q[2]*=n; q[3]*=n;
}

void AttitudeEKF::enforceSymmetry() {
    for (int i = 0; i < 7; i++) {
        for (int j = i + 1; j < 7; j++) {
            float avg = (P[i][j] + P[j][i]) * 0.5f;
            P[i][j] = avg;
            P[j][i] = avg;
        }
    }
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
    for (int i=0; i<7; i++) {
        for (int j=0; j<7; j++) {
            P[i][j] = 0.0f;
        }
    }
    P[0][0]=P[1][1]=P[2][2]=P[3][3]=0.1f;
    P[4][4]=P[5][5]=P[6][6]=0.1f;
}

void AttitudeEKF::predict(float gx, float gy, float gz, float dt) {
    float wx = gx - b[0], wy = gy - b[1], wz = gz - b[2];
    float qd[4];
    qd[0] = 0.5f * (-wx*q[1] - wy*q[2] - wz*q[3]);
    qd[1] = 0.5f * ( wx*q[0] + wz*q[2] - wy*q[3]);
    qd[2] = 0.5f * ( wy*q[0] - wz*q[1] + wx*q[3]);
    qd[3] = 0.5f * ( wz*q[0] + wy*q[1] - wx*q[2]);
    q[0] += qd[0]*dt; q[1] += qd[1]*dt; q[2] += qd[2]*dt; q[3] += qd[3]*dt;
    quat_norm(q);

    float F[7][7];
    for (int i=0; i<7; i++) {
        for (int j=0; j<7; j++) {
            F[i][j] = 0.0f;
        }
    }
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
    for (int i=0; i<7; i++) {
        for (int j=0; j<7; j++) {
            Q[i][j] = 0.0f;
        }
    }
    float Xi[4][3] = {
        {-q[1], -q[2], -q[3]},
        { q[0], -q[3],  q[2]},
        { q[3],  q[0], -q[1]},
        {-q[2],  q[1],  q[0]}
    };
    float qg = GYRO_NOISE * GYRO_NOISE;
    float s = 0.25f * qg * dt * dt;
    for (int i=0; i<4; i++) {
        for (int j=0; j<4; j++) {
            float sum = 0.0f;
            for (int k=0; k<3; k++) {
                sum += Xi[i][k] * Xi[j][k];
            }
            Q[i][j] = s * sum;
        }
    }
    Q[0][0]+=1e-9f; Q[1][1]+=1e-9f; Q[2][2]+=1e-9f; Q[3][3]+=1e-9f;
    float qb = BIAS_NOISE * BIAS_NOISE * dt;
    Q[4][4]=Q[5][5]=Q[6][6]=qb;

    float FP[7][7];
    for (int i=0; i<7; i++) {
        for (int j=0; j<7; j++) {
            float sum = 0.0f;
            for (int k=0; k<7; k++) {
                sum += F[i][k] * P[k][j];
            }
            FP[i][j] = sum;
        }
    }
    for (int i=0; i<7; i++) {
        for (int j=0; j<7; j++) {
            float sum = 0.0f;
            for (int k=0; k<7; k++) {
                sum += FP[i][k] * F[j][k];
            }
            P[i][j] = sum + Q[i][j];
        }
    }
    
    enforceSymmetry();
}

void AttitudeEKF::update(float ax, float ay, float az) {
    float amag = sqrtf(ax*ax + ay*ay + az*az);
    if (amag < ACC_MIN || amag > ACC_MAX) return;
    
    float n = 1.0f / amag;
    ax *= n; ay *= n; az *= n;
    float h0 = 2.0f * (q[1]*q[3] - q[0]*q[2]);
    float h1 = 2.0f * (q[2]*q[3] + q[0]*q[1]);
    float h2 = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
    float y0 = ax - h0;
    float y1 = ay - h1;
    float y2 = az - h2;

    float H[3][7];
    for (int j=0; j<7; j++) {
        H[0][j]=H[1][j]=H[2][j]=0.0f;
    }
    H[0][0]=-2.0f*q[2]; H[0][1]= 2.0f*q[3]; H[0][2]=-2.0f*q[0]; H[0][3]= 2.0f*q[1];
    H[1][0]= 2.0f*q[1]; H[1][1]= 2.0f*q[0]; H[1][2]= 2.0f*q[3]; H[1][3]= 2.0f*q[2];
    H[2][0]= 2.0f*q[0]; H[2][1]=-2.0f*q[1]; H[2][2]=-2.0f*q[2]; H[2][3]= 2.0f*q[3];

    float dev = fabsf(amag - 9.80665f) / 9.80665f;
    float r = R_BASE + R_ADAPT * dev * dev;

    float PHt[7][3];
    for (int i=0; i<7; i++) {
        for (int j=0; j<3; j++) {
            float sum = 0.0f;
            for (int k=0; k<7; k++) {
                sum += P[i][k] * H[j][k];
            }
            PHt[i][j] = sum;
        }
    }
    float S[3][3];
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            float sum = 0.0f;
            for (int k=0; k<7; k++) {
                sum += H[i][k] * PHt[k][j];
            }
            S[i][j] = sum;
        }
    }
    S[0][0]+=r; S[1][1]+=r; S[2][2]+=r;

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

    float K[7][3];
    for (int i=0; i<7; i++) {
        for (int j=0; j<3; j++) {
            float sum = 0.0f;
            for (int k=0; k<3; k++) {
                sum += PHt[i][k] * Si[k][j];
            }
            K[i][j] = sum;
        }
    }
    q[0] += K[0][0]*y0 + K[0][1]*y1 + K[0][2]*y2;
    q[1] += K[1][0]*y0 + K[1][1]*y1 + K[1][2]*y2;
    q[2] += K[2][0]*y0 + K[2][1]*y1 + K[2][2]*y2;
    q[3] += K[3][0]*y0 + K[3][1]*y1 + K[3][2]*y2;
    quat_norm(q);
    b[0] += K[4][0]*y0 + K[4][1]*y1 + K[4][2]*y2;
    b[1] += K[5][0]*y0 + K[5][1]*y1 + K[5][2]*y2;
    b[2] += K[6][0]*y0 + K[6][1]*y1 + K[6][2]*y2;

    float KH[7][7];
    for (int i=0; i<7; i++) {
        for (int j=0; j<7; j++) {
            float sum = 0.0f;
            for (int k=0; k<3; k++) {
                sum += K[i][k] * H[k][j];
            }
            KH[i][j] = sum;
        }
    }
    float Pnew[7][7];
    for (int i=0; i<7; i++) {
        for (int j=0; j<7; j++) {
            float sum = 0.0f;
            for (int k=0; k<7; k++) {
                sum += ((i==k?1.0f:0.0f) - KH[i][k]) * P[k][j];
            }
            Pnew[i][j] = sum;
        }
    }
    for (int i=0; i<7; i++) {
        for (int j=0; j<7; j++) {
            P[i][j] = Pnew[i][j];
        }
    }
    
    enforceSymmetry();
}

void AttitudeEKF::getEulerDeg(float &roll, float &pitch, float &yaw) const {
    roll  = atan2f(2.0f*(q[0]*q[1] + q[2]*q[3]), 1.0f - 2.0f*(q[1]*q[1] + q[2]*q[2])) * RAD2DEG;
    pitch = asinf(2.0f*(q[0]*q[2] - q[3]*q[1])) * RAD2DEG;
    yaw   = atan2f(2.0f*(q[0]*q[3] + q[1]*q[2]), 1.0f - 2.0f*(q[2]*q[2] + q[3]*q[3])) * RAD2DEG;
}

// ======================== UMUMI SABITLER ========================
#define DEVICE_NAME   "DRONE (CC)"
#define LED_PIN         13
#define RF_SERIAL       Serial2
#define RF_BAUD         115200
#define GPS_SERIAL      Serial7
#define GPS_BAUD        9600
#define GPS_AGE_MAX_MS  3000UL
#define I2C_FREQ        100000UL
#define BNO055_PERIOD   10
#define BME280_PERIOD   40
#define AHT20_PERIOD    1000
#define GPS_PERIOD      200
#define PRINT_PERIOD    200
#define RF_PERIOD       66
#define FLIGHT_PERIOD   10
#define GPS_DEBUG_PERIOD 5000
#define I2C_CHECK_PERIOD 3000
#define SEA_LEVEL_HPA   1013.25f

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
#define FLAG_TOUCHDOWN      0x0040
#define FLAG_DESCENDING     0x0080

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

static void rf_send_status_event(uint8_t flight_state, uint16_t throttle) {
    uint8_t p[4];
    size_t i = 0;
    put_u8(p, i, flight_state);
    put_u16(p, i, throttle);
    put_u8(p, i, rf_armed() ? 1 : 0);
    rf_write_packet(RF_PKT_STATUS, p, i);
}

static void rf_send_binary_telemetry(
    const GPSData& gps_ref,
    const AltVel& altvel,
    const FlightCtrl& flight,
    float mad_roll, float mad_pitch, float mad_yaw,
    float ax, float ay, float az,
    float gx, float gy, float gz,
    float mx, float my, float mz,
    float bme_tk, float bme_p, float bme_h, float bme_ak,
    float aht_t, float aht_h,
    bool ok_bno, bool ok_bme, bool ok_aht, bool ok_gps) {
    
    uint8_t p[120];
    size_t i = 0;

    uint16_t flags = 0;
    if (ok_bno) flags |= FLAG_BNO_OK;
    if (ok_bme) flags |= FLAG_BME_OK;
    if (ok_aht) flags |= FLAG_AHT_OK;
    if (ok_gps) flags |= FLAG_GPS_FIX;
    if (rf_armed()) flags |= FLAG_ARMED;
    if (flight.state == FS_MOTORS_ON) flags |= FLAG_MOTORS_ON;
    if (flight.state == FS_TOUCHDOWN) flags |= FLAG_TOUCHDOWN;
    if (altvel.vel < 0.0f) flags |= FLAG_DESCENDING;

    put_u16(p, i, flags);

    if (ok_bno) {
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

    if (ok_bme) {
        put_i16(p, i, f_i16(bme_tk, 100.0f));
        put_u16(p, i, f_u16(bme_p, 10.0f));
        put_u16(p, i, f_u16(bme_h, 100.0f));
        put_i32(p, i, f_i32(bme_ak, 100.0f));
    } else {
        put_i16(p, i, 0); put_u16(p, i, 0); put_u16(p, i, 0); put_i32(p, i, 0);
    }

    if (ok_aht && !isnan(aht_t)) {
        put_i16(p, i, f_i16(aht_t, 100.0f));
        put_u16(p, i, f_u16(aht_h, 100.0f));
    } else {
        put_i16(p, i, 0); put_u16(p, i, 0);
    }

    int32_t lat_e7 = 0, lon_e7 = 0, gps_alt_cm = 0;
    uint16_t gps_speed_cm_s = 0, gps_course_x100 = 0;
    uint8_t gps_sats = 0;

    if (ok_gps) {
        lat_e7 = (int32_t)(tgps.location.lat() * 10000000.0);
        lon_e7 = (int32_t)(tgps.location.lng() * 10000000.0);
        if (tgps.altitude.isValid()) {
            gps_alt_cm = f_i32((float)tgps.altitude.meters(), 100.0f);
        } else {
            gps_alt_cm = f_i32(gps_ref.altitude, 100.0f);
        }
        gps_speed_cm_s = f_u16(gps_ref.speed, 100.0f);
        gps_course_x100 = f_u16(gps_ref.course, 100.0f);
        gps_sats = gps_ref.satellites;
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
    put_i16(p, i, f_i16(altvel.accel_bias, 1000.0f));
    put_u8(p, i, flight.state_code());
    put_u16(p, i, flight.throttle());

    rf_write_packet(RF_PKT_TELEM, p, i);
}

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
        Serial.println(F("  << GPS-DEN HECH NE GELMIR!"));
    } else if (tgps.failedChecksum() > 10 && tgps.sentencesWithFix() == 0) {
        Serial.println(F("  << CHECKSUM XETASI!"));
    } else if (tgps.sentencesWithFix() == 0) {
        Serial.println(F("  << MESAJ VAR, FIX YOXDUR"));
    } else {
        Serial.println(F("  << OK"));
    }
}

struct Kalman1D {
    float Q,R,P,K,X;
    void init(float q,float r,float x0) {
        Q=q; R=r; P=1; K=0; X=x0;
    }
    float update(float z) {
        P += Q;
        K = P/(P+R);
        X += K*(z-X);
        P = (1-K)*P;
        return X;
    }
};

// ======================== GLOBAL ========================
static struct { uint8_t bno055:1, bme280:1, aht20:1, gps_fix:1; } ok;
static AttitudeEKF ekf;
static Kalman1D kalmanTemp, kalmanAlt;
static uint32_t lastBno, lastBme, lastAht, lastGps, lastPrn, lastRf, lastGpsDbg, lastI2cCheck;
static uint32_t lastFlight;
static AltVel altvel;
static FlightCtrl flight;

static float ax,ay,az,gx,gy,gz,mx,my,mz;
static float bme_t,bme_p,bme_h,bme_a,bme_tk,bme_ak;
static float aht_t,aht_h;
static float mad_roll,mad_pitch,mad_yaw;

static uint32_t lastEkf_us = 0;
static uint8_t last_flight_state = 255;

// ======================== I2C RECOVERY (Teensy-uyğun) ========================
static void i2c_recovery() {
    Serial.println(F("[I2C] RECOVERY: Wire yenidən başladılır..."));
    Wire.end();
    delay(20);
    Wire.begin();
    Wire.setClock(I2C_FREQ);
    // DÜZƏLİŞ: Teensy 4.1-də setWireTimeout yoxdur. setTimeout istifadə edirik (millis)
    Wire.setTimeout(3);  // 3 ms timeout (default 1000 ms-dən çox qısadır)
    delay(20);
    
    if (ok.bno055) {
        bno.begin();
        bno.setExtCrystalUse(false);
    }
    if (ok.bme280) {
        bme.begin(0x76, &Wire);
        bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                        Adafruit_BME280::SAMPLING_X2,
                        Adafruit_BME280::SAMPLING_X16,
                        Adafruit_BME280::SAMPLING_X1,
                        Adafruit_BME280::FILTER_X16,
                        Adafruit_BME280::STANDBY_MS_0_5);
    }
    if (ok.aht20) {
        aht.begin();
    }
}

// ======================== SETUP ========================
void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    esc_init();
    Serial.begin(115200);
    delay(200);
    Serial.println(F("\n=== TEENSY 4.1 " DEVICE_NAME " ==="));
    Serial.println(F("[REJIM] AVTOMATIK: 500m + Enish + +-5Derece -> ESC 1480us"));
    Serial.println(F("[RF] Binary Protocol (DeviceID=0xCC) | Paketler: ARM_ON, ARM_OFF"));
    Serial.println(F("[ALT] Avtomatik boot kalibrasiyasi (2.5s) - yandirildigi yer sifir"));
    Serial.println(F("[GPS] TinyGPS++ | [EKF] Dinamik dt + G-clipping (14G) + Simmetriya"));
    Serial.println(F("[I2C] 100 kHz + 3ms timeout | [MOTOR] Histerezis 5/15 + Touchdown"));

    rf_command_init();
    altvel.init();
    flight.init();
    RF_SERIAL.begin(RF_BAUD);

    Wire.begin();
    Wire.setClock(I2C_FREQ);
    // DÜZƏLİŞ: Teensy üçün setTimeout istifadə edirik
    Wire.setTimeout(3);  // 3 ms I2C timeout
    Serial.print(F("[I2C] ")); Serial.print(I2C_FREQ/1000); Serial.println(F(" kHz, timeout=3ms"));

    Serial.print(F("BNO055: "));
    if (!bno.begin()) {
        Serial.println(F("FAIL"));
        ok.bno055 = false;
    } else {
        ok.bno055 = true;
        bno.setExtCrystalUse(false);
        delay(100);
        uint8_t sys, gyro, accel, mag;
        bno.getCalibration(&sys, &gyro, &accel, &mag);
        Serial.print(F("OK (Cal: ")); Serial.print(sys); Serial.print('/');
        Serial.print(gyro); Serial.print('/'); Serial.print(accel); Serial.print('/');
        Serial.print(mag); Serial.println(F(")"));
    }

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
        kalmanTemp.init(0.001f, 0.5f, bme_t);
        kalmanAlt.init(0.01f, 2.0f, bme_a);
    }

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

    GPS_SERIAL.begin(GPS_BAUD);
    Serial.print(F("GPS:    Serial7 @ ")); Serial.print(GPS_BAUD);
    Serial.println(F(" baud - TinyGPS++"));

    ekf.init(0.0f, 0.0f, 9.80665f);
    Serial.println(F("[FILTER] Attitude EKF initialized"));
    Serial.print(F("[RF] Serial2 @ ")); Serial.print(RF_BAUD);
    Serial.println(F(" baud, Binary Protocol (0xCC)"));

    uint32_t now = millis();
    lastBno = lastBme = lastAht = lastGps = lastPrn = lastRf = lastGpsDbg = lastI2cCheck = now;
    lastFlight = now;
    lastEkf_us = micros();
    last_flight_state = 255;
    
    digitalWrite(LED_PIN, LOW);
}

// ======================== LOOP ========================
void loop() {
    uint32_t now = millis();
    rf_command_update();

    while (GPS_SERIAL.available() > 0) {
        tgps.encode(GPS_SERIAL.read());
    }

    if (now - lastI2cCheck >= I2C_CHECK_PERIOD) {
        lastI2cCheck = now;
        
        if (ok.bme280) {
            Wire.beginTransmission(0x76);
            uint8_t err = Wire.endTransmission();
            if (err != 0) {
                Serial.println(F("[I2C] BME280 cavab vermir - RECOVERY"));
                i2c_recovery();
            }
        }
        
        if (ok.bno055) {
            Wire.beginTransmission(0x28);
            uint8_t err = Wire.endTransmission();
            if (err != 0) {
                Serial.println(F("[I2C] BNO055 cavab vermir - RECOVERY"));
                i2c_recovery();
            }
        }
    }

    if (now - lastFlight >= FLIGHT_PERIOD) {
        lastFlight = now;
        float tilt = fmaxf(fabsf(mad_roll), fabsf(mad_pitch));
        bool descending = (altvel.vel < 0.0f);
        
        if (!ok.bno055) {
            tilt = 90.0f;
        }
        
        flight.update(rf_armed(), tilt, descending, altvel.rel_alt, altvel.vel, now);
        
        uint8_t cur_state = flight.state_code();
        if (cur_state != last_flight_state) {
            last_flight_state = cur_state;
            rf_send_status_event(cur_state, flight.throttle());
        }
    }

    if (now - lastBno >= BNO055_PERIOD) {
        lastBno = now;
        if (ok.bno055) {
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

            uint32_t now_us = micros();
            float dt_ekf = (float)(now_us - lastEkf_us) * 1.0e-6f;
            lastEkf_us = now_us;
            
            if (dt_ekf < 0.001f) dt_ekf = 0.001f;
            if (dt_ekf > 0.05f) dt_ekf = 0.05f;
            
            ekf.predict(gx, gy, gz, dt_ekf);
            ekf.update(ax, ay, az);
            ekf.getEulerDeg(mad_roll, mad_pitch, mad_yaw);
        }
    }

    if (now - lastBme >= BME280_PERIOD) {
        lastBme = now;
        if (ok.bme280) {
            bme_t = bme.readTemperature();
            bme_p = bme.readPressure() / 100.0f;
            bme_h = bme.readHumidity();
            bme_a = bme.readAltitude(SEA_LEVEL_HPA);

            bme_tk = kalmanTemp.update(bme_t);
            bme_ak = kalmanAlt.update(bme_a);

            altvel.update(bme_p, ekf.q, ax, ay, az, ok.bno055);
        }
    }

    if (ok.aht20 && now - lastAht >= AHT20_PERIOD) {
        lastAht = now;
        sensors_event_t humidity, temp;
        aht.getEvent(&humidity, &temp);
        aht_t = temp.temperature;
        aht_h = humidity.relative_humidity;
    }

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

    if (now - lastGpsDbg >= GPS_DEBUG_PERIOD) {
        lastGpsDbg = now;
        gps_debug_dump();
    }

    static bool led = false;
    if (now & 0x200) {
        if (!led) {
            digitalWrite(LED_PIN, HIGH);
            led = true;
        }
    } else {
        if (led) {
            digitalWrite(LED_PIN, LOW);
            led = false;
        }
    }

    if (now - lastPrn >= PRINT_PERIOD) {
        lastPrn = now;
        Serial.print(now); Serial.print(' ');
        if (ok.bno055) {
            Serial.print(F("A:")); Serial.print(ax,2); Serial.print(',');
            Serial.print(ay,2); Serial.print(','); Serial.print(az,2);
            Serial.print(F(" G:")); Serial.print(gx,3); Serial.print(',');
            Serial.print(gy,3); Serial.print(','); Serial.print(gz,3);
        } else {
            Serial.print(F("IMU:OFF"));
        }
        Serial.print(F(" | T:"));
        if (ok.bme280) {
            Serial.print(bme_tk,1); Serial.print('/'); Serial.print(bme_ak,1);
        } else {
            Serial.print(F("OFF"));
        }
        Serial.print(F(" | A:"));
        if (ok.aht20 && !isnan(aht_t)) {
            Serial.print(aht_t,1); Serial.print('/'); Serial.print(aht_h,1);
        } else {
            Serial.print(F("OFF"));
        }
        Serial.print(F(" | GPS:"));
        if (ok.gps_fix) {
            Serial.print(gps.lat,5); Serial.print(',');
            Serial.print(gps.lon,5);
            Serial.print(F(" sat:")); Serial.print(gps.satellites);
        } else {
            Serial.print(F("NO"));
        }
        Serial.print(F(" | FLT:"));
        Serial.print(rf_armed() ? F("ARM") : F("DISARM"));
        Serial.print('/'); Serial.print((int)flight.state_code());
        Serial.print(F(" tilt=")); Serial.print(fmaxf(fabsf(mad_roll), fabsf(mad_pitch)),1);
        Serial.print(F(" alt=")); Serial.print(altvel.rel_alt,2);
        Serial.print(F(" vel=")); Serial.print(altvel.vel,2);
        Serial.print(F(" pwm=")); Serial.print(flight.throttle());
        Serial.println();
    }

    if (now - lastRf >= RF_PERIOD) {
        lastRf = now;
        rf_send_binary_telemetry(
            gps, altvel, flight,
            mad_roll, mad_pitch, mad_yaw,
            ax, ay, az, gx, gy, gz, mx, my, mz,
            bme_tk, bme_p, bme_h, bme_ak,
            aht_t, aht_h,
            ok.bno055, ok.bme280, ok.aht20, ok.gps_fix
        );
    }
}