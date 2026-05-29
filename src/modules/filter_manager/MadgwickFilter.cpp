

// MadgwickFilter.cpp 
// cpp
// 
#include "MadgwickFilter.h"
#include <math.h>
#pragma GCC optimize("O3")

MadgwickFilter::MadgwickFilter() : q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f), beta(0.02f) {}
void MadgwickFilter::begin(float sampleFreqHz) { beta = 0.02f; q0=1.0f; q1=q2=q3=0.0f; }
void MadgwickFilter::setBeta(float b) { beta = b; }

void MadgwickFilter::update(float dt, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
    float recipNorm;
    // float s0, s1, s2, s3, normS;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    // Строки 36,41,42,64,65: Инициализация и исправление области видимости
    float _2q0 = 0.0f, _2q1 = 0.0f, _2q2 = 0.0f, _2q3 = 0.0f; // ← Инициализация в начале функции
    float _8q1 = 0.0f, _8q2 = 0.0f, _4q0 = 0.0f, invS = 0.0f, normS = 0.0f; // ← Тоже
    // float _8q1 = 0.0f, _8q2 = 0.0f ; // ← Тоже

    // float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz,_2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    // float _8q1, _8q2, _4q0, invS ;
;

                            // Альтернатива : Добавить `(void)varname;` для подавления предупреждений, 
                            // если переменные нужны для будущей 9-осевой поддержки:
                            // Для временного подавления предупреждений:
                            //(void)invS; (void)_4q0; (void)_8q1; (void)_8q2; (void)normS;
                            //(void)s0; (void)s1; (void)s2; (void)s3; (void)hx; (void)hy;
                            //(void)_2q1mx; (void)_2q0q2; (void)_2q2q3; (void)q0q0; (void)q0q3;

    q0q0 = q0*q0; q0q1 = q0*q1; q0q2 = q0*q2; q0q3 = q0*q3;
    q1q1 = q1*q1; q1q2 = q1*q2; q1q3 = q1*q3; q2q2 = q2*q2; q2q3 = q2*q3; q3q3 = q3*q3;

    if(mx==0.0f && my==0.0f && mz==0.0f) {
                _2q0 = 2.0f*q0; _2q1 = 2.0f*q1; _2q2 = 2.0f*q2; _2q3 = 2.0f*q3;
                invS = 1.0f / (ax*ax + ay*ay + az*az);
                _4q0 = 4.0f*q0; _8q1 = 8.0f*q1; _8q2 = 8.0f*q2;
                normS = 1.0f / sqrtf(_2q1*q1 + _2q2*q2); // Исправлено под актуальные переменные
                // ... остальной код 6-осевого блока
        // 6-axis mode
        _2q0 = 2.0f*q0; _2q1 = 2.0f*q1; _2q2 = 2.0f*q2; _2q3 = 2.0f*q3;
        float invS = 1.0f / (ax*ax + ay*ay + az*az);
        float norm = sqrtf(ax*ax + ay*ay + az*az);
        if(norm < 0.001f) norm = 1.0f; else norm = 1.0f/norm;
        ax*=norm; ay*=norm; az*=norm;

        float _4q0 = 4.0f*q0, _4q1 = 4.0f*q1, _4q2 = 4.0f*q2, _8q1 = 8.0f*q1, _8q2 = 8.0f*q2;
        float normS = 1.0f / sqrtf(_4q1*q1 + _4q2*q2 - 1.0f); // Упрощённая норма для акселерометра

        // Градиентный спуск (упрощённая и быстрая версия для 6-осей)
        float v1 = 2.0f*(q1*q3 - q0*q2) - ax;
        float v2 = 2.0f*(q0*q1 + q2*q3) - ay;
        float v3 = 1.0f - 2.0f*(q1*q1 + q2*q2) - az;

        qDot1 = beta * ( 2.0f*v1*q2 - 2.0f*v2*q1 );
        qDot2 = beta * ( 2.0f*v1*q3 - 2.0f*v2*q0 - 4.0f*q1 + 4.0f*v3*q2 );
        qDot3 = beta * ( 2.0f*v1*q0 - 2.0f*v2*q3 + 4.0f*q2 + 4.0f*v3*q1 );
        qDot4 = beta * ( 2.0f*v1*q1 - 2.0f*v2*q2 );
    } else {
                _2q0 = 2.0f*q0; _2q1 = 2.0f*q1; _2q2 = 2.0f*q2; _2q3 = 2.0f*q3; // ← Заполняем перед использованием
                // ... остальной код 9-осевого блока
        // 9-axis mode (полная версия Мэджвика)
        _2q0mx = 2.0f*q0*mx; _2q0my = 2.0f*q0*my; _2q0mz = 2.0f*q0*mz;
        _2q1mx = 2.0f*q1*mx;
        _2bx = sqrtf(_2q0mx*_2q0mx + _2q0my*_2q0my + _2q0mz*_2q0mz);
        _2bz = sqrtf(_2q0mz*_2q0mz + _2q0my*_2q0my + _2q0mz*_2q0mz);        
            //  _2bx = sqrtf(_4q0mx*_4q0mx + _2q0my*_2q0my + _2q0mz*_2q0mz);
            //  _2bz = sqrtf(_4q0mz*_4q0mz + _2q0my*_2q0my + _2q0mz*_2q0mz);
        
        _4bx = 2.0f*_2bx; _4bz = 2.0f*_2bz;

        float s0 = -_2q2*(2.0f*q1q3 - _2q0*q2 - ax) + _2q1*(2.0f*q0*q1 + _2q2*q3 - ay) - _2bz*q2*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0*q2) - mx) + (-_2bx*q3 + _2bz*q1)*(_2bx*(q1q2 - q0*q3) + _2bz*(q0q1 + q2q3) - my) + _2bx*q2*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);
        float s1 =  _2q3*(2.0f*q1q3 - _2q0*q2 - ax) + _2q0*(2.0f*q0*q1 + _2q2*q3 - ay) - 4.0f*q1*(1 - 2.0f*q1q1 - 2.0f*q2q2 - az) + _2bz*q3*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0*q2) - mx) + (_2bx*q2 + _2bz*q0)*(_2bx*(q1q2 - q0*q3) + _2bz*(q0q1 + q2q3) - my) + (_2bx*q3 - _4bz*q1)*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);
        float s2 = -_2q0*(2.0f*q1q3 - _2q0*q2 - ax) + _2q3*(2.0f*q0*q1 + _2q2*q3 - ay) - 4.0f*q2*(1 - 2.0f*q1q1 - 2.0f*q2q2 - az) + (-_4bx*q2 - _2bz*q0)*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0*q2) - mx) + (_2bx*q1 + _2bz*q3)*(_2bx*(q1q2 - q0*q3) + _2bz*(q0q1 + q2q3) - my) + (_2bx*q0 - _4bz*q2)*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);
        float s3 =  _2q1*(2.0f*q1q3 - _2q0*q2 - ax) + _2q2*(2.0f*q0*q1 + _2q2*q3 - ay) + (-_4bx*q3 + _2bz*q1)*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0*q2) - mx) + (-_2bx*q0 + _2bz*q2)*(_2bx*(q1q2 - q0*q3) + _2bz*(q0q1 + q2q3) - my) + _2bx*q1*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);

        recipNorm = 1.0f / sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        qDot1 = beta * s0 * recipNorm; qDot2 = beta * s1 * recipNorm;
        qDot3 = beta * s2 * recipNorm; qDot4 = beta * s3 * recipNorm;
    }

    // Интегрирование гироскопа + коррекция градиентом
    q0 += (-q1*gx - q2*gy - q3*gz - qDot1) * dt;
    q1 += ( q0*gx + q2*gz - q3*gy - qDot2) * dt;
    q2 += ( q0*gy - q1*gz + q3*gx - qDot3) * dt;
    q3 += ( q0*gz + q1*gy - q2*gx - qDot4) * dt;

    float norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm > 0.0001f) { norm = 1.0f/norm; q0*=norm; q1*=norm; q2*=norm; q3*=norm; }
}

void MadgwickFilter::getQuaternion(float &w, float &x, float &y, float &z) const { w=q0; x=q1; y=q2; z=q3; }
void MadgwickFilter::getEuler(float &roll, float &pitch, float &yaw) const {
    roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2));
          //  pitch = asinf(constrainf(2.0f*(q0*q2 - q3*q1), -1.0f, 1.0f));
    pitch = asinf(constrain(2.0f*(q0*q2 - q3*q1), -1.0f, 1.0f));
    yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));
}
void MadgwickFilter::setInitialState(float roll, float pitch, float yaw) {
    float cr = cosf(roll*0.5f), sr = sinf(roll*0.5f);
    float cp = cosf(pitch*0.5f), sp = sinf(pitch*0.5f);
    float cy = cosf(yaw*0.5f), sy = sinf(yaw*0.5f);
    q0 = cr*cp*cy + sr*sp*sy;
    q1 = sr*cp*cy - cr*sp*sy;
    q2 = cr*sp*cy + sr*cp*sy;
    q3 = cr*cp*sy - sr*sp*cy;
}


void MadgwickFilter::setAdaptiveParams(float vibrationLevel) {
    if (!_adaptiveEnabled) {
        beta = _baseBeta;
        return;
    }
    
    float vib = fmaxf(0.0f, fminf(vibrationLevel, 1.0f));
    
    // Обратная зависимость: больше вибраций -> меньше beta
    beta = _minBeta + (_maxBeta - _minBeta) * (1.0f - vib);
}
