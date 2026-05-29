

// MahonyFilter.cpp 
// cpp

#include "MahonyFilter.h"
#include <math.h>
#pragma GCC optimize("O3") // Включение агрессивной оптимизации + FPU

MahonyFilter::MahonyFilter() : q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f),
                               twoKp(1.0f), twoKi(0.001f),
                               exInt(0.0f), eyInt(0.0f), ezInt(0.0f) {}

void MahonyFilter::begin(float sampleFreqHz) {
    twoKp = 1.0f * 2.0f; twoKi = 0.001f * 2.0f;
    q0 = 1.0f; q1 = q2 = q3 = 0.0f;
    exInt = eyInt = ezInt = 0.0f;
}

void MahonyFilter::setGains(float kp, float ki) {
    twoKp = kp * 2.0f; twoKi = ki * 2.0f;
}

void MahonyFilter::update(float dt, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
    float norm, vx, vy, vz, ex, ey, ez;

    norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 0.001f) return;
    norm = 1.0f / norm;
    ax *= norm; ay *= norm; az *= norm;

    vx = 2.0f*(q1*q3 - q0*q2); vy = 2.0f*(q0*q1 + q2*q3); vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    ex = ay*vz - az*vy; ey = az*vx - ax*vz; ez = ax*vy - ay*vx;

    if(twoKi > 0.0f) {
        exInt += ex*dt; eyInt += ey*dt; ezInt += ez*dt;
        gx += twoKi*exInt; gy += twoKi*eyInt; gz += twoKi*ezInt;
    }

    gx += twoKp*ex; gy += twoKp*ey; gz += twoKp*ez;

    float qa=q0, qb=q1, qc=q2, qd=q3;
    gx *= 0.5f*dt; gy *= 0.5f*dt; gz *= 0.5f*dt;
    q0 += -qb*gx - qc*gy - qd*gz;
    q1 +=  qa*gx + qc*gz - qd*gy;
    q2 +=  qa*gy - qb*gz + qd*gx;
    q3 +=  qa*gz + qb*gy - qc*gx;

    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm > 0.0001f) { norm = 1.0f/norm; q0*=norm; q1*=norm; q2*=norm; q3*=norm; }
}

void MahonyFilter::getQuaternion(float &w, float &x, float &y, float &z) const { w=q0; x=q1; y=q2; z=q3; }
void MahonyFilter::getEuler(float &roll, float &pitch, float &yaw) const {
    roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2));
    // pitch = asinf(constrainf(2.0f*(q0*q2 - q3*q1), -1.0f, 1.0f));
    pitch = asinf(constrain(2.0f*(q0*q2 - q3*q1), -1.0f, 1.0f));
    yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));
}
void MahonyFilter::setInitialState(float roll, float pitch, float yaw) {
    float cr = cosf(roll*0.5f), sr = sinf(roll*0.5f);
    float cp = cosf(pitch*0.5f), sp = sinf(pitch*0.5f);
    float cy = cosf(yaw*0.5f), sy = sinf(yaw*0.5f);
    q0 = cr*cp*cy + sr*sp*sy;
    q1 = sr*cp*cy - cr*sp*sy;
    q2 = cr*sp*cy + sr*cp*sy;
    q3 = cr*cp*sy - sr*sp*cy;
    exInt = eyInt = ezInt = 0.0f;
}


void MahonyFilter::setAdaptiveParams(float vibrationLevel) {
    if (!_adaptiveEnabled) {
        twoKp = _baseKp * 2.0f; 
        twoKi = _baseKi * 2.0f;
        return;
    }
    
    // Ограничиваем уровень вибраций [0.0 ... 1.0]
    float vib = fmaxf(0.0f, fminf(vibrationLevel, 1.0f));
    
    // Обратная зависимость: больше вибраций -> меньше Kp/Ki
    // Lerp: val = min + (max - min) * (1.0 - vib)
    float kp = _minKp + (_maxKp - _minKp) * (1.0f - vib);
    float ki = _minKi + (_maxKi - _minKi) * (1.0f - vib);
    
    twoKp = kp * 2.0f;
    twoKi = ki * 2.0f;
}
