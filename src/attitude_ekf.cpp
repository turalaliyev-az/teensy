#include "attitude_ekf.h"
#include <math.h>

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
