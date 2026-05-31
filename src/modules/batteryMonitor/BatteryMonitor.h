



/**
 * @file BatteryMonitor.h
 * @brief Мониторинг напряжения аккумулятора (3S-12S LiPo) — КЛАССИЧЕСКИЙ ADC API
 * @version 1.1.0 — Исправлены типы для ESP-IDF 5.0+
 * @date 2026
 * 
 * @details
 * - Использует driver/adc.h + esp_adc_cal.h (совместимо с Arduino Framework)
 * - Явное приведение adc1_channel_t для совместимости с ESP32-S3
 * - Поддержка конфигураций 3S-12S из CommonTypes.h
 * - Калибровка через esp_adc_cal_characterize()
 * - Усреднение 16 выборок, защита от выбросов
 * 
 * @note GPIO7 (ADC1_CH6) рекомендован для ESP32-S3 — поддерживает калибровку!
 */
// === src/modules/batteryMonitor/BatteryMonitor.h ===
#pragma once

#include "common/CommonTypes.h"   // ← BatteryStatus_t, BatteryConfig_t

#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <Arduino.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <esp_log.h>
#include "config/pins.h"
#include "common/types.h"  

class BatteryMonitor {
public:
    BatteryMonitor();
    ~BatteryMonitor();
    
    bool begin(uint8_t cellCount = 4);
    BatteryStatus_t checkVoltage();
    void update();
    
    // Геттеры
    BatteryStatus_t getStatus() const { return _lastStatus; }
    bool isVoltageOk() const { return _lastStatus.isOk; }
    bool isLow() const { return _lastStatus.isLow; }
    bool isCritical() const { return _lastStatus.isCritical; }
    float getVoltagePerCell() const { return _lastStatus.voltagePerCell; }
    
    bool setCellCount(uint8_t cellCount);
    uint8_t getCellCount() const { return _currentConfig.cellCount; }
    
    void printStatus(bool verbose = false) const;
    void resetStats();

private:
    BatteryStatus_t _lastStatus;
    BatteryConfig_t _currentConfig;
    bool _initialized = false;
    bool _calibrated = false;
    
    // 🔑 Классический ADC контекст
    adc_unit_t _adcUnit;
    adc1_channel_t _adcChannel;  // ← Явный тип для ESP-IDF 5.0+
    esp_adc_cal_characteristics_t _adcChars;
    
    uint32_t _checkCount = 0;
    uint32_t _criticalCount = 0;
    uint32_t _lastCheckTime = 0;
    
    static const char* TAG;
    
    // Приватные методы
    bool _initADC();
    bool _readADC(int& outValue);
    float _adcToVoltage(int adcRaw) const;
    float _calcPercentage(float voltage) const;
    void _updateStatus(float voltage);
    const BatteryConfig_t* _findConfig(uint8_t cells) const;
    
    static adc_unit_t _gpioToAdcUnit(uint8_t gpio);
    static adc1_channel_t _gpioToAdc1Channel(uint8_t gpio);
};

#endif // BATTERY_MONITOR_H