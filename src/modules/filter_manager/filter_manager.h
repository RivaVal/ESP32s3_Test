

/**
 * @file filter_manager.h
 * @brief Менеджер переключения фильтров ориентации (Махони/Мэджвик)
 * @details
 * - Адаптивные параметры на основе уровня вибраций
 * - Безопасное переключение без скачков ориентации
 * - Поддержка 6-осевого (аксел+гироскоп) и 9-осевого (с магнитометром) режимов
 */
#pragma once
#ifndef FILTER_MANAGER_H
#define FILTER_MANAGER_H

#include "common/types.h"
#include "modules/filter/mahony_filter.h"
#include "modules/filter/madgwick_filter.h"
#include "modules/vibration/vibration_estimator.h"

enum class FilterType { MAHONY, MADGWICK };

class FilterManager {
public:
    FilterManager();

    bool begin(float sampleFreqHz = 250.0f);  // 250 Гц для ESP32-S3

    // 🔑 Адаптивность одной командой
    void enableAdaptive(bool enable);
    bool isAdaptiveEnabled() const { return _adaptiveEnabled; }

    // 🔑 Основной метод обновления (вызывать 250 раз/сек)
    void update(float dt, float ax, float ay, float az,
                float gx, float gy, float gz,
                float mx = 0.0f, float my = 0.0f, float mz = 0.0f);

    // Получение ориентации
    void getQuaternion(float &w, float &x, float &y, float &z) const;
    void getEuler(float &roll, float &pitch, float &yaw) const;

    // Переключение фильтра (безопасно в полёте)
    void requestSwitch(FilterType type);
    FilterType getActiveType() const { return _currentType; }

    // Диагностика
    const char* getActiveFilterName() const;
    float getCurrentVibrationLevel() const { return _vibEstimator.getVibrationLevel(); }

private:
    MahonyFilter _mahony;
    MadgwickFilter _madgwick;
    VibrationEstimator _vibEstimator;

    ISensorFilter* _activeFilter;
    FilterType _currentType;

    // Безопасное переключение
    volatile bool _pendingSwitch;
    FilterType _pendingType;

    bool _adaptiveEnabled = false;

    static const char* TAG;
};

#endif // FILTER_MANAGER_H
