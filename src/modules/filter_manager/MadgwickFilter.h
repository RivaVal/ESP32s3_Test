

// ### 📁 3. Фильтр Мэджвика `MadgwickFilter.h` / `.cpp`
// Алгоритм градиентного спуска. Меньше дрейф, 
// чуть выше нагрузка на CPU (но ESP32 справится легко).
//  MadgwickFilter.h 
// ### 📄 4. `MadgwickFilter.h` / `.cpp` (Адаптивный)
// Логика:  При вибрациях уменьшаем `beta` (шаг градиентного 
// спуска). Фильтр начинает больше доверять гироскопу и 
// игнорировать зашумлённый акселерометр.

// cpp
// 
#pragma once
#include "modules/filter_manager/ISensorFilter.h"
//#include "moISensorFilter.h"

class MadgwickFilter : public ISensorFilter {
public:
    MadgwickFilter();
    void begin(float sampleFreqHz) override;
    void update(float dt, float ax, float ay, float az, float gx, float gy, float gz, 
                float mx = 0.0f, float my = 0.0f, float mz = 0.0f) override;
    void getQuaternion(float &w, float &x, float &y, float &z) const override;
    void getEuler(float &roll, float &pitch, float &yaw) const override;
    void setInitialState(float roll, float pitch, float yaw) override;
    const char* getName() const override { return "Madgwick"; }
    void setBeta(float beta);

    void setAdaptiveParams(float vibrationLevel) override;
    bool isAdaptiveEnabled() const override { return _adaptiveEnabled; }
    void setAdaptiveEnabled(bool enable) { _adaptiveEnabled = enable; }

private:
    float q0, q1, q2, q3;
    float beta;

   
    // 🔑 АДАПТИВНОСТЬ (добавлено для устранения ошибок компиляции)
    bool _adaptiveEnabled = false;
    float _baseBeta = 0.04f;
    float _minBeta = 0.008f;
    float _maxBeta = 0.12f;    
};
