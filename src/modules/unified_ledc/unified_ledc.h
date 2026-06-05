
/**
 * @file unified_ledc.h
 * @brief Контроллер управления силовыми моторами через аппаратный ШИМ LEDC ESP32-S3
 * @details
 * - Использует аппаратный ШИМ LEDC для генерации PWM 1 кГц
 * - Поддерживает 2 независимых мотора (GPIO 16, 21)
 * - Разрешение 10 бит (0-1023 градации мощности)
 * - Встроенная защита: ограничение тестовой мощности, проверка диапазонов, graceful degradation
 *
 * @environment ESP32-S3 | PlatformIO | Arduino Framework | ESP-IDF 5.0+
 * @version 2.0.0 (ШАГ 5: Миграция LEDC)
 * @date 2026-05-28
 */
/**
 * @file unified_ledc.h
 * @brief Контроллер управления силовыми моторами через аппаратный ШИМ LEDC ESP32-S3
 * @version 2.1.0 (Рефакторинг: пины перенесены в config/pins.h)
 */
#pragma once
#include <Arduino.h>
#include <driver/ledc.h>
#include "esp_err.h"
#include "common/types.h"
#include "config/pins.h"  // 🔑 ЗАВИСИМОСТЬ ОТ ГЛОБАЛЬНОГО КОНФИГА

class UnifiedLEDCController {
private:
    bool _initialized = false;
    const char* _lastError = "No error";

        //                // 🔑 ПАРАМЕТРЫ ШИМ (Частота и разрешение)
    static constexpr uint32_t _freqHz         = 1000;  // 1 кГц (стандарт для ESC/моторов)
    static constexpr uint32_t _dutyResolution = 10;    // 10 бит (0-1023)
    static constexpr ledc_mode_t _speedMode   = LEDC_LOW_SPEED_MODE;

    esp_err_t setupTimer();
    esp_err_t setupChannel(uint8_t index);

public:
    UnifiedLEDCController();
    bool begin();
    bool setMotorPower(uint8_t motorIndex, float powerPercent);
    void stopAllMotors();
    bool runStartupDiagnostic(float maxPowerPercent = 30.0f, uint32_t durationMs = 2000);
    
    bool isInitialized() const { return _initialized; }
    const char* getLastError() const { return _lastError; }
};
