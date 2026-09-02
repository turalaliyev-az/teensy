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

#include "esc.h"
#include "rf_command.h"
#include "alt_vel.h"
#include "flight_ctrl.h"
#include "attitude_ekf.h"

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
    w8(BNO055_ADDR,BNO055_SYS_TRIG,0x20);delay(650);  // RST_SYS (reset)
    w8(BNO055_ADDR,BNO055_PWR_MODE,0x00);delay(10);   // normal guc
    w8(BNO055_ADDR,BNO055_UNIT_SEL,0x00);             // m/s2, dps, deg, C
    w8(BNO055_ADDR,BNO055_SYS_TRIG,0x80);delay(50);   // CLK_SRC: xarici kristal (VACIBDİR!)
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
    // USB
    Serial.print(F("[I2C] "));Serial.print(n);Serial.print(F(" dev | BNO055="));Serial.print(bno?1:0);
    Serial.print(F(" BME280="));Serial.print(bme?1:0);Serial.print(F(" AHT20="));Serial.print(aht?1:0);
    Serial.print(F(" | "));
    for(uint8_t i=0;i<n;i++){Serial.print(F("0x"));Serial.print(addrs[i],HEX);Serial.print(' ');}
    Serial.println();
    // RF (RFD900X) diaqnostik sətir
    RF_SERIAL.print(F("DIAG,"));RF_SERIAL.print(millis());RF_SERIAL.print(',');
    RF_SERIAL.print(bno?1:0);RF_SERIAL.print(',');RF_SERIAL.print(bme?1:0);RF_SERIAL.print(',');RF_SERIAL.print(aht?1:0);
    RF_SERIAL.print(',');RF_SERIAL.print(n);
    for(uint8_t i=0;i<n;i++){RF_SERIAL.print(',');RF_SERIAL.print(F("0x"));RF_SERIAL.print(addrs[i],HEX);}
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