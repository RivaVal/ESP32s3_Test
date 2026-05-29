// ### 📁 2. Фильтр Махони `MahonyFilter.h` / `.cpp`
// Классический алгоритм с ПИ-коррекцией дрейфа. 
// Быстрый, стабильный.
// **Логика:** При вибрациях уменьшаем `Kp` 
// (меньше реакции на шумный акселерометр) и обнуляем 
// `Ki` (защита от насыщения интегратора).
// MahonyFilter.h 

// cpp
// 
#pragma once
#include "ISensorFilter.h"

class MahonyFilter : public ISensorFilter {
public:
    MahonyFilter();
    void begin(float sampleFreqHz) override;
    void update(float dt, float ax, float ay, float az, float gx, float gy, float gz, 
                float mx = 0.0f, float my = 0.0f, float mz = 0.0f) override;
    void getQuaternion(float &w, float &x, float &y, float &z) const override;
    void getEuler(float &roll, float &pitch, float &yaw) const override;
    void setInitialState(float roll, float pitch, float yaw) override;
    const char* getName() const override { return "Mahony"; }
    void setGains(float kp, float ki);

    void setAdaptiveParams(float vibrationLevel) override;
    bool isAdaptiveEnabled() const override { return _adaptiveEnabled; }
    void setAdaptiveEnabled(bool enable) { _adaptiveEnabled = enable; }

private:
    float q0, q1, q2, q3;
    float twoKp, twoKi;
    float exInt, eyInt, ezInt;

    bool _adaptiveEnabled = false;
    float _baseKp = 2.0f, _baseKi = 0.002f; // Базовые коэффициенты
    float _minKp = 0.5f, _maxKp = 3.0f;
    float _minKi = 0.0f, _maxKi = 0.005f;

};
