
/**
 * @file battery_monitor.h
 * @brief Мониторинг напряжения аккумуляторной батареи (3S-12S LiPo) для ESP32-S3
 * @details
 * - Поддержка конфигураций АКБ от 3S до 12S LiPo/Li-Ion
 * - Новый ADC драйвер esp_adc/adc_oneshot.h (ESP-IDF 5.0+)
 * - Автоматический расчёт порогов на основе количества ячеек
 * - Периодическая проверка в update() без блокировки
 * - Аварийные действия при критическом разряде
 * - Подробная отладка через ESP_LOG (DEBUG/INFO/WARN/ERROR)
 *
 * @note Требуется внешний делитель напряжения для подключения к GPIO7 (ADC1_CH6)!
 *       На ESP32-S3: GPIO1-10 → ADC_UNIT_1 (с калибровкой), GPIO11-20 → ADC_UNIT_2 (без калибровки)
 *
 * @environment ESP32-S3 | PlatformIO | Arduino Framework | ESP-IDF 5.0+
 * @version 2.0.0 (ШАГ 6: Миграция BatteryMonitor)
 * @date 2026-05-28
 */
#pragma once

#include <Arduino.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_log.h>
#include "common/types.h"  // BatteryStatus_t, BatteryConfig_t

class BatteryMonitor {
public:
    BatteryMonitor();
    ~BatteryMonitor();

    /** @brief Инициализация монитора батареи с заданной конфигурацией АКБ */
    bool begin(uint8_t cellCount = 4);  // 4S по умолчанию

    /** @brief Мгновенная проверка напряжения (немедленное чтение + расчёт) */
    BatteryStatus_t checkVoltage();

    /** @brief Периодическая проверка (вызывать в loop(), неблокирующая) */
    void update();

    /** @brief Получение последнего статуса батареи */
    BatteryStatus_t getStatus() const { return _lastStatus; }

    /** @brief Проверка: напряжение в норме? */
    bool isVoltageOk() const { return _lastStatus.isOk; }

    /** @brief Проверка: критически низкое напряжение? */
    bool isCritical() const { return _lastStatus.isCritical; }

    /** @brief Проверка: низкое напряжение (<20%)? */
    bool isLow() const { return _lastStatus.isLow; }

    /** @brief Установка конфигурации АКБ (3-12 ячеек) */
    bool setCellCount(uint8_t cellCount);

    /** @brief Получение текущей конфигурации */
    uint8_t getCellCount() const { return _currentConfig.cellCount; }

    /** @brief Печать статуса в лог (verbose = подробный вывод) */
    void printStatus(bool verbose = false) const;

    /** @brief Получение напряжения на одну ячейку */
    float getVoltagePerCell() const { return _lastStatus.voltagePerCell; }

    /** @brief Сброс счётчиков и статистики */
    void reset();

    /** @brief Проверка успешной инициализации */
    bool isInitialized() const { return _initialized; }

private:
    // 🔑 Приватные методы для работы с новым ADC API
    bool initADC();
    bool readADC(int &adcValue);
    float adcToBatteryVoltage(int adcValue) const;
    float calculatePercentage(float voltage) const;
    const BatteryConfig_t* findConfig(uint8_t cellCount) const;
    void updateStatus(float voltage);

    // 🔑 Кроссплатформенные хелперы для маппинга пинов (ESP32 / ESP32-S3)
    static adc_unit_t _gpioToAdcUnit(uint8_t gpio);
    static adc_channel_t _gpioToAdcChannel(uint8_t gpio);

    // 🔑 Состояние и конфигурация
    BatteryStatus_t _lastStatus;
    BatteryConfig_t _currentConfig;
    bool _initialized = false;
    uint32_t _checkCount = 0;
    uint32_t _criticalCount = 0;

    // 🔑 Контекст ADC (ESP-IDF 5.0+)
    adc_oneshot_unit_handle_t _adcHandle = nullptr;
    adc_cali_handle_t _adcCaliHandle = nullptr;
    bool _adcCaliValid = false;

    // 🔑 Тег для ESP_LOG
    static const char* const TAG;
};
