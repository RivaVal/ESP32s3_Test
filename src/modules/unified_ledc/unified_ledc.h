
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
#pragma once

#include <Arduino.h>
#include <driver/ledc.h>
#include "esp_err.h"
#include "common/types.h"

class UnifiedLEDCController {
private:
    bool _initialized = false;
    const char* _lastError = "No error";

    // 🔑 КОНФИГУРАЦИЯ ПИНОВ И КАНАЛОВ (ESP32-S3)
    static constexpr uint8_t _motorPins[2]   = {16, 21};                 // GPIO16, GPIO21
    static constexpr ledc_channel_t _channels[2] = {LEDC_CHANNEL_1, LEDC_CHANNEL_2};
    static constexpr uint32_t _freqHz        = 1000;                     // Частота PWM для моторов
    static constexpr uint32_t _dutyResolution = 10;                      // 10 бит (0-1023)

    esp_err_t setupTimer();
    esp_err_t setupChannel(uint8_t index);

public:
    UnifiedLEDCController();

    /** @brief Инициализация LEDC таймера и каналов */
    bool begin();

    /** @brief Установка мощности мотора (0.0 - 100.0%) */
    bool setMotorPower(uint8_t motorIndex, float powerPercent);

    /** @brief Мгновенная остановка всех моторов */
    void stopAllMotors();

    /** @brief Безопасный предстартовый тест с ограничением мощности */
    bool runStartupDiagnostic(float maxPowerPercent = 30.0f, uint32_t durationMs = 2000);

    /** @brief Проверка статуса инициализации */
    bool isInitialized() const { return _initialized; }

    /** @brief Получение текста последней ошибки */
    const char* getLastError() const { return _lastError; }
};
