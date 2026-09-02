#ifndef ATTITUDE_EKF_H
#define ATTITUDE_EKF_H

#include <Arduino.h>

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

#endif
