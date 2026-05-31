

// ### 📄 2. `ISensorFilter.h` (Расширение интерфейса)
// cpp
// 

#pragma once
#include <Arduino.h>

class ISensorFilter { 
public:
    virtual ~ISensorFilter() = default;
    virtual void begin(float sampleFreqHz) = 0;
    virtual void update(float dt, float ax, float ay, float az, 
                        float gx, float gy, float gz, 
                        float mx = 0.0f, float my = 0.0f, float mz = 0.0f) = 0;
    virtual void getQuaternion(float &w, float &x, float &y, float &z) const = 0;
    virtual void getEuler(float &roll, float &pitch, float &yaw) const = 0;
    virtual void setInitialState(float roll, float pitch, float yaw) = 0;
    virtual const char* getName() const = 0;
    
    // 🔑 НОВЫЙ МЕТОД: Адаптивная подстройка (по умолчанию ничего не делает)
    virtual void setAdaptiveParams(float vibrationLevel) { (void)vibrationLevel; }
    virtual bool isAdaptiveEnabled() const { return false; }
};
