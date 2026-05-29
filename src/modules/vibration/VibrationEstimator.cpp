

// **`VibrationEstimator.cpp`**
// 
// 
#include "modules/vibration/VibrationEstimator.h"
// #include "VibrationEstimator.h"
#include <math.h>
#pragma GCC optimize("O3") // Агрессивная оптимизация + FPU

VibrationEstimator::VibrationEstimator() 
    : _filteredMag(1.0f), _vibration(0.0f), 
      _alphaMag(0.15f), _alphaVib(0.05f), _normG(1.5f) {}

void VibrationEstimator::configure(float alphaMag, float alphaVib, float normG) {
    _alphaMag = fmaxf(0.01f, fminf(alphaMag, 0.99f));
    _alphaVib = fmaxf(0.005f, fminf(alphaVib, 0.5f));
    _normG = fmaxf(0.5f, normG);
}

float VibrationEstimator::compute(float ax, float ay, float az) {
    // 1. Модуль вектора ускорения (FPU: sqrtf)
    float mag = sqrtf(ax*ax + ay*ay + az*az);
    
    // 2. НЧ-фильтр модуля (отсекает вибрации >~20Hz)
    _filteredMag = _filteredMag * (1.0f - _alphaMag) + mag * _alphaMag;
    
    // 3. "Дрожание" = отклонение от сглаженного значения
    float jitter = fabsf(mag - _filteredMag);
    
    // 4. НЧ-фильтр вибрации (интегрирует энергию вибраций ~5Hz)
    _vibration = _vibration * (1.0f - _alphaVib) + jitter * _alphaVib;
    
    // 5. Нормализация в [0.0 ... 1.0+]
    return fminf(_vibration / _normG, 1.0f);
}
