
// ### 📄 1. `VibrationEstimator.h` / `.cpp` (Оценка вибраций)
// **Принцип:** Вибрация проявляется как высокочастотные отклонения модуля 
//  вектора ускорения от `1g`. Используем **экспоненциальное скользящее 
// среднее (EMA)** для O(1) вычислений без буферов.
// 
// **`VibrationEstimator.h`**

#pragma once
#include <Arduino.h>

class VibrationEstimator {
public:
    VibrationEstimator();
    
    // Расчёт нормализованного уровня вибраций [0.0 - 1.0+]
    // 0.0 = идеально гладко, 1.0 = сильные вибрации
    float compute(float ax, float ay, float az);
    
    // Настройка чувствительности (вызывать 1 раз в setup)
    void configure(float alphaMag = 0.15f, float alphaVib = 0.05f, float normG = 1.5f);
    
    // Получить сырое значение ускорения (для отладки)
    float getRawMagnitude() const { return _filteredMag; }
    float getVibrationLevel() const { return _vibration; }

private:
    float _filteredMag;   // Сглаженный модуль ускорения
    float _vibration;     // Сглаженный уровень "дрожания"
    float _alphaMag;      // Коэф. сглаживания модуля (быстрее)
    float _alphaVib;      // Коэф. сглаживания вибрации (медленнее)
    float _normG;         // Нормализующий коэффициент (макс. допустимая вибрация)
};
