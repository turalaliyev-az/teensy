#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BME280.h>
#include <Adafruit_AHTX0.h>

// ======================== SENSOR OBYEKTLERI ========================
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BME280 bme;
Adafruit_AHTX0 aht;

// ======================== ESC (50 Hz PWM) ========================
#define ESC1_PIN      15      
#define ESC2_PIN      23      
#define ESC_PWM_FREQ  50.0f
#define ESC_US_MIN    1000
#define ESC_US_MAX    2000
#define ESC_US_OFF    1000    
#define ESC_US_TEST   1150    // TEST REJIMI UCUN YENI DEYER (Asagi suret)

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

// ======================== RF EMRLERİ ========================
static bool _armed = false;
static bool _last_ack = false;   

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
    void update(float pressure_hpa, float az_mps2, float ax_mps2, float ay_mps2);
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
#define CALIB_MS     2000UL
#define CALIB_MIN_N  10
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

void AltVel::update(float pressure_hpa, float az, float ax, float ay) {
    uint32_t now_us = micros();
    float dt = 0.0f;
    if (_last_us != 0) {
        dt = (float)(now_us - _last_us) * 1.0e-6f;
        if (dt < 0.0f) dt = 0.0f;
        if (dt > 0.5f) dt = 0.5f;
    }
    _last_us = now_us;

    float g_raw = sqrtf(ax*ax + ay*ay + az*az) / GRAVITY;
    if (_g_smooth <= 0.0f) {
        _g_smooth = g_raw;
    } else {
        _g_smooth += G_ALPHA * (g_raw - _g_smooth);
    }
    g_force = _g_smooth;

    if (!_have_prev) {
        _p_smooth = pressure_hpa; _prev_p = pressure_hpa; _have_prev = true;
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
        rel_alt = 0.0f; vel = 0.0f;
        if (millis() - _calib_start_ms >= CALIB_MS && _calib_count >= CALIB_MIN_N) {
            _p0 = _calib_sum / (float)_calib_count;
            _p_smooth = _p0; _prev_p = _p0; _calibrated = true;
        }
        return;
    }

    float z = 0.0f;
    if (_p0 > 1.0f) {
        z = 44330.0f * (1.0f - powf(_p_smooth / _p0, 0.1903f));
    }

    float a_vert = az - GRAVITY;
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

// ======================== UCUS NEZARETCISI (SINAQ REJIMI) ========================
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

void FlightCtrl::init() {
    state = FS_DISARMED;
    throttle_us = (float)ESC_US_OFF;
    _last_ms = 0;
}

void FlightCtrl::update(bool armed, bool level_ok, bool descending, float rel_alt, uint32_t now_ms) {
    _last_ms = now_ms;

    // DISARM her zaman ustundur: derhal 1000 us
    if (!armed) {
        state = FS_DISARMED;
        throttle_us = (float)ESC_US_OFF;
        esc_write_us((uint16_t)throttle_us);
        return;
    }

    // SINAQ REJIMI: Level ve descending sertleri helelik nezere alinmir.
    // RF-den '1' geldiyi an motorlar 1150 us alir.
    state = FS_MOTORS_ON;
    throttle_us = (float)ESC_US_TEST;
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
};

#define RAD2DEG 57.29577951308232f
#define GYRO_NOISE  0.02f    
#define BIAS_NOISE  0.0005f  
#define R_BASE      0.003f   
#define R_ADAPT     2.0f     
#define ACC_MIN     3.0f     
#define ACC_MAX     25.0f    

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
    for (int i=0;i<7;i++) {
        for (int j=0;j<7;j++) {
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
    for (int i=0;i<7;i++) {
        for (int j=0;j<7;j++) {
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
    for (int i=0;i<7;i++) {
        for (int j=0;j<7;j++) {
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
}

void AttitudeEKF::update(float ax, float ay, float az) {
    float amag = sqrtf(ax*ax + ay*ay + az*az);
    if (amag < ACC_MIN || amag > ACC_MAX) return;
    float n = 1.0f / amag; ax *= n; ay *= n; az *= n;
    float h0 = 2.0f * (q[1]*q[3] - q[0]*q[2]); 
    float h1 = 2.0f * (q[2]*q[3] + q[0]*q[1]); 
    float h2 = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
    float y0 = ax - h0; float y1 = ay - h1; float y2 = az - h2;
    
    float H[3][7]; 
    for (int j=0;j<7;j++) {
        H[0][j]=H[1][j]=H[2][j]=0.0f;
    }
    H[0][0]=-2.0f*q[2]; H[0][1]= 2.0f*q[3]; H[0][2]=-2.0f*q[0]; H[0][3]= 2.0f*q[1];
    H[1][0]= 2.0f*q[1]; H[1][1]= 2.0f*q[0]; H[1][2]= 2.0f*q[3]; H[1][3]= 2.0f*q[2];
    H[2][0]= 2.0f*q[0]; H[2][1]=-2.0f*q[1]; H[2][2]=-2.0f*q[2]; H[2][3]= 2.0f*q[3];
    
    float dev = fabsf(amag - 9.80665f) / 9.80665f; 
    float r = R_BASE + R_ADAPT * dev * dev;
    
    float PHt[7][3];
    for (int i=0;i<7;i++) {
        for (int j=0;j<3;j++) { 
            float sum=0.0f; 
            for (int k=0;k<7;k++) sum += P[i][k] * H[j][k]; 
            PHt[i][j] = sum; 
        }
    }
    float S[3][3];
    for (int i=0;i<3;i++) {
        for (int j=0;j<3;j++) { 
            float sum=0.0f; 
            for (int k=0;k<7;k++) sum += H[i][k] * PHt[k][j]; 
            S[i][j] = sum; 
        }
    }
    S[0][0]+=r; S[1][1]+=r; S[2][2]+=r;
    
    float det = S[0][0]*(S[1][1]*S[2][2]-S[1][2]*S[2][1]) - S[0][1]*(S[1][0]*S[2][2]-S[1][2]*S[2][0]) + S[0][2]*(S[1][0]*S[2][1]-S[1][1]*S[2][0]);
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
    for (int i=0;i<7;i++) {
        for (int j=0;j<3;j++) { 
            float sum=0.0f; 
            for (int k=0;k<3;k++) sum += PHt[i][k] * Si[k][j]; 
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
    for (int i=0;i<7;i++) {
        for (int j=0;j<7;j++) { 
            float sum=0.0f; 
            for (int k=0;k<3;k++) sum += K[i][k] * H[k][j]; 
            KH[i][j] = sum; 
        }
    }
    float Pnew[7][7];
    for (int i=0;i<7;i++) {
        for (int j=0;j<7;j++) { 
            float sum=0.0f; 
            for (int k=0;k<7;k++) sum += ((i==k?1.0f:0.0f) - KH[i][k]) * P[k][j]; 
            Pnew[i][j] = sum; 
        }
    }
    for (int i=0;i<7;i++) {
        for (int j=0;j<7;j++) {
            P[i][j] = Pnew[i][j];
        }
    }
}

void AttitudeEKF::getEulerDeg(float &roll, float &pitch, float &yaw) const {
    roll  = atan2f(2.0f*(q[0]*q[1] + q[2]*q[3]), 1.0f - 2.0f*(q[1]*q[1] + q[2]*q[2])) * RAD2DEG;
    pitch = asinf(2.0f*(q[0]*q[2] - q[3]*q[1])) * RAD2DEG;
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
#define I2C_FREQ        400000UL
#define BNO055_PERIOD   10    
#define BME280_PERIOD   40    
#define AHT20_PERIOD    1000  
#define GPS_PERIOD      200   
#define PRINT_PERIOD    200   
#define RF_PERIOD       66    
#define FLIGHT_PERIOD   10    
#define SEA_LEVEL_HPA   1013.25f

// ======================== GPS NMEA PARSER ========================
static float nmea_to_decimal(float ddmm) {
    int deg = (int)(ddmm / 100.0f); 
    float min = ddmm - (float)(deg * 100);
    return (float)deg + min / 60.0f;
}

struct GPSData {
    float lat, lon, altitude, speed, course; 
    uint8_t fix, satellites; 
    bool updated;
    GPSData() : lat(0),lon(0),altitude(0),speed(0),course(0),fix(0),satellites(0),updated(false) {}
};
static GPSData gps; 
static char gps_buf[128]; 
static uint8_t gps_idx = 0;

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
    p=strchr(p,',');if(!p)return;p++;
    if(*p!='A')return; 
    for(int i=0;i<4;i++){p=strchr(p,',');if(!p)return;p++;}
    float spd=strtof(p,&p);if(!p||*p!=',')return;p++; 
    float crs=strtof(p,&p);
    gps.speed=spd*0.514444f;gps.course=crs;
}

static void gps_read(){
    while(GPS_SERIAL.available()){
        char c=GPS_SERIAL.read();
        if(c=='$'){
            gps_idx=0;gps_buf[0]='$';gps_buf[1]=0;
        }
        else if(c=='\n'){
            gps_buf[gps_idx]=0;
            if(strncmp(gps_buf,"$GPGGA",6)==0)gps_parse_gpgga(gps_buf);
            else if(strncmp(gps_buf,"$GPRMC",6)==0)gps_parse_gprmc(gps_buf);
            gps_idx=0;
        }
        else if(gps_idx<127){
            gps_buf[gps_idx++]=c;gps_buf[gps_idx]=0;
        }
    }
}

// ======================== KALMAN VƏ STATUS ========================
struct Kalman1D {
    float Q,R,P,K,X;
    void init(float q,float r,float x0){Q=q;R=r;P=1;K=0;X=x0;}
    float update(float z){P+=Q;K=P/(P+R);X+=K*(z-X);P=(1-K)*P;return X;}
};

static struct{uint8_t bno055:1,bme280:1,aht20:1,gps_fix:1;} ok;
static AttitudeEKF ekf;
static Kalman1D kalmanTemp,kalmanAlt;
static uint32_t lastBno,lastBme,lastAht,lastGps,lastPrn,lastRf;
static uint32_t lastFlight;
static AltVel altvel;
static FlightCtrl flight;

static float ax,ay,az,gx,gy,gz,mx,my,mz;
static float bme_t,bme_p,bme_h,bme_a,bme_tk,bme_ak;
static float aht_t,aht_h;
static float mad_roll,mad_pitch,mad_yaw;

// ======================== SETUP ========================
void setup(){
    pinMode(LED_PIN,OUTPUT);
    digitalWrite(LED_PIN,HIGH);
    esc_init();   
    Serial.begin(115200);
    delay(200);
    Serial.println(F("\n=== TEENSY 4.1 " DEVICE_NAME " ==="));
    Serial.println(F("[REJIM] ESC SINAQ REJIMI AKTIV (ARM=1150us, DISARM=1000us)"));
    rf_command_init(); 
    altvel.init(); 
    flight.init();
    RF_SERIAL.begin(RF_BAUD);
    
    Wire.begin();
    Wire.setClock(I2C_FREQ);
    
    // BNO055 Init
    Serial.print(F("BNO055: "));
    if(!bno.begin()) {
        Serial.println(F("FAIL (Check Wiring)"));
        ok.bno055 = false;
    } else {
        ok.bno055 = true;
        bno.setExtCrystalUse(false); 
        delay(100);
        uint8_t sys, gyro, accel, mag;
        bno.getCalibration(&sys, &gyro, &accel, &mag);
        Serial.print(F("OK (Cal: ")); Serial.print(sys); Serial.print('/'); 
        Serial.print(gyro); Serial.print('/'); Serial.print(accel); Serial.print('/'); Serial.print(mag); Serial.println(F(")"));
    }

    // BME280 Init
    Serial.print(F("BME280: "));
    if (!bme.begin(0x76, &Wire)) {
        Serial.println(F("FAIL (Check Address/Wiring)"));
        ok.bme280 = false;
    } else {
        ok.bme280 = true;
        bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                        Adafruit_BME280::SAMPLING_X2,  // Temp
                        Adafruit_BME280::SAMPLING_X16, // Pressure
                        Adafruit_BME280::SAMPLING_X1,  // Humidity
                        Adafruit_BME280::FILTER_X16,
                        Adafruit_BME280::STANDBY_MS_0_5);
        Serial.println(F("25 Hz OK"));
        bme_t = bme.readTemperature();
        bme_a = bme.readAltitude(SEA_LEVEL_HPA);
        kalmanTemp.init(0.001f, 0.5f, bme_t);
        kalmanAlt.init(0.01f, 2.0f, bme_a);
    }

    // AHT20 Init
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

    // GPS Init
    GPS_SERIAL.begin(GPS_BAUD);
    Serial.print(F("GPS:    Serial7 @ ")); Serial.print(GPS_BAUD); Serial.println(F(" baud"));
    
    // EKF Init
    ekf.init(0.0f, 0.0f, 9.80665f);
    Serial.println(F("[FILTER] Attitude EKF initialized"));
    
    Serial.print(F("[RF] Serial2 @ ")); Serial.print(RF_BAUD); Serial.print(F(" baud, Header: ")); Serial.println(DEVICE_HEADER);

    uint32_t now=millis();
    lastBno=lastBme=lastAht=lastGps=lastPrn=lastRf=now; 
    lastFlight=now;
    digitalWrite(LED_PIN,LOW);
}

// ======================== LOOP ========================
void loop(){
    uint32_t now=millis();
    rf_command_update();

    // Ucus Nezaretcisi (100 Hz)
    if(now-lastFlight>=FLIGHT_PERIOD){
        lastFlight=now;
        static bool wasLevel=false;
        float tilt=fmaxf(fabsf(mad_roll),fabsf(mad_pitch));
        bool level = wasLevel ? (tilt<=8.0f) : (tilt<=5.0f);
        if(!ok.bno055){
            level=false;
            wasLevel=false;
        } else {
            wasLevel=level;
        }
        bool descending = (altvel.vel < 0.0f);
        
        // SINAQ REJIMI: level ve descending false olsa bele, flight.update() sadece armed statusunu yoxlayacaq
        flight.update(rf_armed(), level, descending, altvel.rel_alt, now);
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
    
    // Baro / BME280 (25 Hz)
    if(now-lastBme>=BME280_PERIOD){
        lastBme=now;
        if(ok.bme280){
            bme_t = bme.readTemperature();
            bme_p = bme.readPressure() / 100.0f;
            bme_h = bme.readHumidity();
            bme_a = bme.readAltitude(SEA_LEVEL_HPA);
            
            bme_tk = kalmanTemp.update(bme_t);
            bme_ak = kalmanAlt.update(bme_a); 
            
            altvel.update(bme_p, az, ax, ay);
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
    
    // GPS (Oxunma)
    gps_read();
    if(now-lastGps>=GPS_PERIOD){
        lastGps=now;
        ok.gps_fix=(gps.fix>0);
    }

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
        if(ok.bme280){ Serial.print(bme_tk,1); Serial.print('/'); Serial.print(bme_ak,1); } 
        else { Serial.print(F("OFF")); }
        
        Serial.print(F(" | A:"));
        if(ok.aht20 && !isnan(aht_t)){ Serial.print(aht_t,1); Serial.print('/'); Serial.print(aht_h,1); } 
        else { Serial.print(F("OFF")); }
        
        Serial.print(F(" | GPS:"));
        if(ok.gps_fix){ Serial.print(gps.lat,5); Serial.print(','); Serial.print(gps.lon,5); } 
        else { Serial.print(F("NO")); }
        
        Serial.print(F(" | FLT:"));
        if(rf_armed()) {
            Serial.print(F("[MOTOR-TEST]"));
        } else {
            Serial.print(F("DISARM"));
        }
        Serial.print('/'); Serial.print((int)flight.state_code());
        Serial.print(F(" alt=")); Serial.print(altvel.rel_alt,1); 
        Serial.print(F(" vel=")); Serial.print(altvel.vel,1);
        Serial.print(F(" g=")); Serial.print(altvel.g_force,2); 
        Serial.print(F(" pwm=")); Serial.print(flight.throttle()); 
        Serial.println();
    }

    // RF Modula Gonderme (15 Hz)
    if(now-lastRf>=RF_PERIOD){
        lastRf=now;
        RF_SERIAL.print(DEVICE_HEADER); RF_SERIAL.print(','); 
        RF_SERIAL.print(now); RF_SERIAL.print(',');
        
        if(ok.bno055){
            RF_SERIAL.print(ax,3); RF_SERIAL.print(','); RF_SERIAL.print(ay,3); RF_SERIAL.print(','); RF_SERIAL.print(az,3); RF_SERIAL.print(',');
            RF_SERIAL.print(gx,4); RF_SERIAL.print(','); RF_SERIAL.print(gy,4); RF_SERIAL.print(','); RF_SERIAL.print(gz,4); RF_SERIAL.print(',');
            RF_SERIAL.print(mx,2); RF_SERIAL.print(','); RF_SERIAL.print(my,2); RF_SERIAL.print(','); RF_SERIAL.print(mz,2);
        }else{
            RF_SERIAL.print(F("N,N,N,N,N,N,N,N,N"));
        }
        RF_SERIAL.print(',');
        
        if(ok.bme280){
            RF_SERIAL.print(bme_tk,1); RF_SERIAL.print(','); RF_SERIAL.print(bme_p,1); RF_SERIAL.print(','); 
            RF_SERIAL.print(bme_h,1); RF_SERIAL.print(','); RF_SERIAL.print(bme_ak,1);
        } else {
            RF_SERIAL.print(F("N,N,N,N"));
        }
        RF_SERIAL.print(',');
        
        if(ok.aht20 && !isnan(aht_t)){
            RF_SERIAL.print(aht_t,1); RF_SERIAL.print(','); RF_SERIAL.print(aht_h,1);
        }else{
            RF_SERIAL.print(F("N,N"));
        }
        RF_SERIAL.print(',');
        
        if(ok.gps_fix){
            RF_SERIAL.print(gps.lat,6); RF_SERIAL.print(','); RF_SERIAL.print(gps.lon,6); RF_SERIAL.print(',');
            RF_SERIAL.print(gps.altitude,1); RF_SERIAL.print(','); RF_SERIAL.print(gps.speed,2); RF_SERIAL.print(','); RF_SERIAL.print(gps.course,1);
        } else {
            RF_SERIAL.print(F("N,N,N,N,N"));
        }
        RF_SERIAL.print(',');
        
        RF_SERIAL.print(mad_roll,2); RF_SERIAL.print(','); RF_SERIAL.print(mad_pitch,2); RF_SERIAL.print(','); RF_SERIAL.print(mad_yaw,2);
        RF_SERIAL.print(','); RF_SERIAL.print(altvel.rel_alt,2); 
        RF_SERIAL.print(','); RF_SERIAL.print(altvel.vel,2);
        RF_SERIAL.print(','); RF_SERIAL.print(altvel.g_force,3); 
        RF_SERIAL.print(','); RF_SERIAL.print(altvel.dpdt,3);
        RF_SERIAL.print(','); RF_SERIAL.print(rf_armed()?1:0); 
        RF_SERIAL.print(','); RF_SERIAL.print((int)flight.state_code());
        RF_SERIAL.print(','); RF_SERIAL.print(flight.throttle()); 
        RF_SERIAL.println();
    }
}