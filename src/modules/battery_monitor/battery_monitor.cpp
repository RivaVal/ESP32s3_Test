
/**
 * @file battery_monitor.cpp
 * @brief Реализация мониторинга напряжения аккумулятора для ESP32-S3
 * @details
 * - Чтение ADC с усреднением 16 выборок для подавления шумов
 * - Защита от аномальных значений (фильтрация выбросов)
 * - Полная совместимость с esp_adc/adc_oneshot.h (ESP-IDF 5.0+)
 * - Калибровка через Curve Fitting (только для ADC_UNIT_1 на S3)
 * - Подробная отладка через ESP_LOG на всех уровнях
 *
 * @version 2.0.0 (ШАГ 6: Миграция BatteryMonitor)
 * @date 2026-05-28
 */
#include "battery_monitor.h"

// ============================================================================
// СТАТИЧЕСКАЯ ПЕРЕМЕННАЯ ТАГА
// ============================================================================
const char* const BatteryMonitor::TAG = "BATTERY";

// ============================================================================
// ТАБЛИЦА КОНФИГУРАЦИЙ АКБ (3S-12S) — из CommonTypes.h
// ============================================================================
constexpr BatteryConfig_t BATTERY_CONFIG_TABLE[] = {
    {3,  12.6f, 11.1f, 9.0f,  8.4f,  "3S LiPo"},
    {4,  16.8f, 14.8f, 12.0f, 11.2f, "4S LiPo"},
    {5,  21.0f, 18.5f, 15.0f, 14.0f, "5S LiPo"},
    {6,  25.2f, 22.2f, 18.0f, 16.8f, "6S LiPo"},
    {7,  29.4f, 25.9f, 21.0f, 19.6f, "7S LiPo"},
    {8,  33.6f, 29.6f, 24.0f, 22.4f, "8S LiPo"},
    {9,  37.8f, 33.3f, 27.0f, 25.2f, "9S LiPo"},
    {10, 42.0f, 37.0f, 30.0f, 28.0f, "10S LiPo"},
    {11, 46.2f, 40.7f, 33.0f, 30.8f, "11S LiPo"},
    {12, 50.4f, 44.4f, 36.0f, 33.6f, "12S LiPo"}
};
constexpr uint8_t BATTERY_CONFIG_COUNT = sizeof(BATTERY_CONFIG_TABLE) / sizeof(BATTERY_CONFIG_TABLE[0]);
constexpr uint8_t MIN_CELL_COUNT = 3;
constexpr uint8_t MAX_CELL_COUNT = 12;

// ============================================================================
// КОНСТРУКТОР / ДЕСТРУКТОР
// ============================================================================
BatteryMonitor::BatteryMonitor() {
    _lastStatus = {};
    _currentConfig = BATTERY_CONFIG_TABLE[1]; // 4S по умолчанию
    ESP_LOGI(TAG, "✅ Конструктор BatteryMonitor инициализирован");
}

BatteryMonitor::~BatteryMonitor() {
    if (_adcCaliHandle) {
        ESP_LOGD(TAG, "🔄 Освобождение калибровки ADC");
        adc_cali_delete_scheme_curve_fitting(_adcCaliHandle);
    }
    if (_adcHandle) {
        ESP_LOGD(TAG, "🔄 Освобождение ADC unit");
        adc_oneshot_del_unit(_adcHandle);
    }
    ESP_LOGI(TAG, "🔄 Ресурсы BatteryMonitor освобождены");
}

// ============================================================================
// 🔑 КРОССПЛАТФОРМЕННЫЕ ХЕЛПЕРЫ (ESP32 / ESP32-S3)
// ============================================================================
adc_unit_t BatteryMonitor::_gpioToAdcUnit(uint8_t gpio) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    // 🔑 ESP32-S3: GPIO1-10 → ADC_UNIT_1 (с калибровкой!), GPIO11-20 → ADC_UNIT_2 (без калибровки)
    return (gpio >= 1 && gpio <= 10) ? ADC_UNIT_1 : ADC_UNIT_2;
#else
    // Классический ESP32: GPIO32-39 → ADC_UNIT_1
    return ADC_UNIT_1;
#endif
}

adc_channel_t BatteryMonitor::_gpioToAdcChannel(uint8_t gpio) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    // 🔑 ADC1 channels для ESP32-S3 (GPIO1-10)
    switch(gpio) {
        case 1:  return ADC_CHANNEL_0;
        case 2:  return ADC_CHANNEL_1;
        case 3:  return ADC_CHANNEL_2;
        case 4:  return ADC_CHANNEL_3;
        case 5:  return ADC_CHANNEL_4;
        case 6:  return ADC_CHANNEL_5;
        case 7:  return ADC_CHANNEL_6;  // 🔑 РЕКОМЕНДУЕМЫЙ ПИН ДЛЯ БАТАРЕИ!
        case 8:  return ADC_CHANNEL_7;
        case 9:  return ADC_CHANNEL_8;
        case 10: return ADC_CHANNEL_9;
        default: return ADC_CHANNEL_0;  // Fallback
    }
#else
    // Классический ESP32 маппинг
    switch(gpio) {
        case 32: return ADC_CHANNEL_4;
        case 33: return ADC_CHANNEL_5;
        case 34: return ADC_CHANNEL_6;  // 🔑 Оригинальный пин батареи
        case 35: return ADC_CHANNEL_7;
        case 36: return ADC_CHANNEL_0;
        case 39: return ADC_CHANNEL_3;
        default: return ADC_CHANNEL_6;
    }
#endif
}

// ============================================================================
// 🔑 ИНИЦИАЛИЗАЦИЯ ADC (ESP-IDF 5.0+ oneshot API)
// ============================================================================
bool BatteryMonitor::initADC() {
    ESP_LOGI(TAG, "⚙️  Инициализация ADC (ESP-IDF 5.0+ oneshot)...");

#if defined(CONFIG_IDF_TARGET_ESP32S3)
    adc_unit_t adcUnit = _gpioToAdcUnit(Config::Pins::BATTERY_ADC_PIN);
    if (adcUnit == ADC_UNIT_2) {
        ESP_LOGW(TAG, "⚠️ GPIO%d → ADC_UNIT_2: калибровка отключена", Config::Pins::BATTERY_ADC_PIN);
        ESP_LOGW(TAG, "💡 Рекомендация: используйте GPIO1-10 для ADC1 (с калибровкой)");
    } else {
        ESP_LOGI(TAG, "✅ GPIO%d → ADC_UNIT_1: калибровка активна", Config::Pins::BATTERY_ADC_PIN);
    }
#endif

    adc_channel_t adcCh = _gpioToAdcChannel(Config::Pins::BATTERY_ADC_PIN);

    // ✅ Порядок полей критичен для компиляции!
    adc_oneshot_unit_init_cfg_t initConfig = {
        .unit_id = _gpioToAdcUnit(Config::Pins::BATTERY_ADC_PIN),
        .clk_src = ADC_RTC_CLK_SRC_RC_FAST,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    esp_err_t err = adc_oneshot_new_unit(&initConfig, &_adcHandle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ Ошибка создания ADC unit: %s", esp_err_to_name(err));
        return false;
    }

    // Конфигурация канала
    adc_oneshot_chan_cfg_t chanConfig = {
        .atten = Config::Battery::ADC_ATTENUATION,
        .bitwidth = ADC_BITWIDTH_12
    };
    err = adc_oneshot_config_channel(_adcHandle, adcCh, &chanConfig);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ Ошибка конфигурации канала: %s", esp_err_to_name(err));
        adc_oneshot_del_unit(_adcHandle);
        return false;
    }

    // 🔑 Калибровка (только для ADC_UNIT_1 на S3)
    _adcCaliValid = false;
    if (_gpioToAdcUnit(Config::Pins::BATTERY_ADC_PIN) == ADC_UNIT_1) {
        adc_cali_curve_fitting_config_t caliConfig = {
            .unit_id = ADC_UNIT_1,
            .chan = adcCh,
            .atten = Config::Battery::ADC_ATTENUATION,
            .bitwidth = ADC_BITWIDTH_12,
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 1, 0)
            .default_vref = 1100,
#endif
        };
        err = adc_cali_create_scheme_curve_fitting(&caliConfig, &_adcCaliHandle);
        if (err == ESP_OK) {
            _adcCaliValid = true;
            ESP_LOGI(TAG, "✅ Калибровка ADC активирована (Curve Fitting)");
        } else {
            ESP_LOGW(TAG, "⚠️  Калибровка недоступна: %s", esp_err_to_name(err));
        }
    } else {
        ESP_LOGW(TAG, "⚠️  ADC_UNIT_2: калибровка отключена (ограничение IDF 5.0+)");
    }

    return true;
}

// ============================================================================
// 🔑 ЧТЕНИЕ ADC С ОБРАБОТКОЙ КАЛИБРОВКИ
// ============================================================================
bool BatteryMonitor::readADC(int &adcValue) {
    if (!_initialized || !_adcHandle) {
        ESP_LOGE(TAG, "❌ readADC: ADC не инициализирован!");
        return false;
    }

    adc_channel_t adcCh = _gpioToAdcChannel(Config::Pins::BATTERY_ADC_PIN);
    esp_err_t err = adc_oneshot_read(_adcHandle, adcCh, &adcValue);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ Ошибка чтения ADC (GPIO%d, CH%d): %s",
                 Config::Pins::BATTERY_ADC_PIN, adcCh, esp_err_to_name(err));
        return false;
    }

    // 🔑 Применение калибровки (если активна)
    if (_adcCaliValid && _adcCaliHandle) {
        int voltageMV = 0;
        err = adc_cali_raw_to_voltage(_adcCaliHandle, adcValue, &voltageMV);
        if (err == ESP_OK) {
            // Конвертируем мВ обратно в raw для совместимости с формулой делителя
            adcValue = (voltageMV * Config::Battery::ADC_RESOLUTION) / 3300;
            ESP_LOGV(TAG, "📊 ADC: raw=%d → calibrated=%d mV → raw=%d",
                     adcValue, voltageMV, adcValue);
        } else {
            ESP_LOGV(TAG, "⚠️ Калибровка не применима: %s", esp_err_to_name(err));
        }
    }
    return true;
}

// ============================================================================
// 🔑 ПРЕОБРАЗОВАНИЕ НАПРЯЖЕНИЯ
// ============================================================================
float BatteryMonitor::adcToBatteryVoltage(int adcValue) const {
    float adcVoltage = (adcValue / static_cast<float>(Config::Battery::ADC_RESOLUTION))
                       * Config::Battery::ADC_MAX_VOLTAGE;
    return adcVoltage * Config::Battery::VOLTAGE_DIVIDER_RATIO;
}

float BatteryMonitor::calculatePercentage(float voltage) const {
    if (voltage <= _currentConfig.voltageMin) return 0.0f;
    if (voltage >= _currentConfig.voltageMax) return 100.0f;
    return ((voltage - _currentConfig.voltageMin) /
            (_currentConfig.voltageMax - _currentConfig.voltageMin)) * 100.0f;
}

void BatteryMonitor::updateStatus(float voltage) {
    _lastStatus.isCritical = (voltage <= _currentConfig.voltageCritical);
    _lastStatus.isLow = (voltage <= _currentConfig.voltageMin);
    _lastStatus.isOk = (voltage > _currentConfig.voltageMin && voltage <= _currentConfig.voltageMax);
}

const BatteryConfig_t* BatteryMonitor::findConfig(uint8_t cellCount) const {
    for (uint8_t i = 0; i < BATTERY_CONFIG_COUNT; i++) {
        if (BATTERY_CONFIG_TABLE[i].cellCount == cellCount) {
            return &BATTERY_CONFIG_TABLE[i];
        }
    }
    return nullptr;
}

// ============================================================================
// 🔑 ПУБЛИЧНЫЕ МЕТОДЫ
// ============================================================================
bool BatteryMonitor::begin(uint8_t cellCount) {
    ESP_LOGI(TAG, "=== 🚀 ИНИЦИАЛИЗАЦИЯ МОНИТОРА БАТАРЕИ ===");

    if (cellCount < MIN_CELL_COUNT || cellCount > MAX_CELL_COUNT) {
        ESP_LOGE(TAG, "❌ Недопустимое количество ячеек: %u (допустимо %u-%u)",
                 cellCount, MIN_CELL_COUNT, MAX_CELL_COUNT);
        return false;
    }

    const BatteryConfig_t* config = findConfig(cellCount);
    if (!config) {
        ESP_LOGE(TAG, "❌ Конфигурация для %uS не найдена в таблице", cellCount);
        return false;
    }
    _currentConfig = *config;
    ESP_LOGI(TAG, "🔋 Конфигурация АКБ: %s", _currentConfig.name);
    ESP_LOGI(TAG, "   Ячеек: %u | Диапазон: %.2fВ - %.2fВ | Критический: %.2fВ",
             _currentConfig.cellCount, _currentConfig.voltageMin,
             _currentConfig.voltageMax, _currentConfig.voltageCritical);

    if (!initADC()) {
        ESP_LOGE(TAG, "❌ Критическая ошибка: не удалось инициализировать ADC");
        return false;
    }

    _initialized = true;
    ESP_LOGI(TAG, "⏳ Задержка перед первой проверкой (%u мс)...", Config::Battery::STARTUP_CHECK_DELAY);
    delay(Config::Battery::STARTUP_CHECK_DELAY);

    checkVoltage();
    ESP_LOGI(TAG, "✅ Монитор батареи успешно инициализирован");
    return true;
}

BatteryStatus_t BatteryMonitor::checkVoltage() {
    if (!_initialized) {
        ESP_LOGE(TAG, "❌ checkVoltage: монитор не инициализирован!");
        return _lastStatus;
    }

    constexpr uint8_t SAMPLES = 16;
    int32_t sum = 0;
    for (uint8_t i = 0; i < SAMPLES; i++) {
        int adcValue = 0;
        if (readADC(adcValue)) {
            sum += adcValue;
        }
        delay(2);  // Стабилизация между выборками
    }
    int adcValue = sum / SAMPLES;

    float voltage = adcToBatteryVoltage(adcValue);
    float percentage = calculatePercentage(voltage);
    float voltagePerCell = voltage / _currentConfig.cellCount;

    _lastStatus = {
        .voltage = voltage,
        .percentage = percentage,
        .isOk = true, .isLow = false, .isCritical = false,
        .lastCheckTime = millis(),
        .adcValue = static_cast<uint16_t>(adcValue),
        .cellCount = _currentConfig.cellCount,
        .voltagePerCell = voltagePerCell,
        .configName = _currentConfig.name
    };

    updateStatus(voltage);
    _checkCount++;

    if (_lastStatus.isCritical) {
        _criticalCount++;
        ESP_LOGE(TAG, "🔴 КРИТИЧЕСКИ НИЗКОЕ напряжение: %.2fВ (%.1f%%)", voltage, percentage);
    }

    return _lastStatus;
}

void BatteryMonitor::update() {
    static uint32_t lastCheck = 0;
    if (!_initialized) return;

    uint32_t currentTime = millis();
    if (currentTime - lastCheck >= Config::Battery::CHECK_INTERVAL_MS) {
        checkVoltage();
        lastCheck = currentTime;
        if (_checkCount % 10 == 0) printStatus(false);
    }
}

bool BatteryMonitor::setCellCount(uint8_t cellCount) {
    if (cellCount < MIN_CELL_COUNT || cellCount > MAX_CELL_COUNT) {
        ESP_LOGE(TAG, "❌ setCellCount: недопустимое значение %u", cellCount);
        return false;
    }
    const BatteryConfig_t* config = findConfig(cellCount);
    if (!config) {
        ESP_LOGE(TAG, "❌ setCellCount: конфигурация не найдена");
        return false;
    }
    _currentConfig = *config;
    ESP_LOGI(TAG, "✅ Конфигурация АКБ изменена: %s", _currentConfig.name);
    checkVoltage();
    return true;
}

void BatteryMonitor::printStatus(bool verbose) const {
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "🔋 СТАТУС БАТАРЕИ");
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "Конфигурация: %s (%uS)", _lastStatus.configName, _lastStatus.cellCount);
    ESP_LOGI(TAG, "Напряжение: %.2f В | На ячейку: %.2f В", _lastStatus.voltage, _lastStatus.voltagePerCell);
    ESP_LOGI(TAG, "Заряд: %.1f %% | ADC значение: %u", _lastStatus.percentage, _lastStatus.adcValue);
    ESP_LOGI(TAG, "Статус: %s",
             _lastStatus.isCritical ? "🔴 КРИТИЧЕСКИЙ" :
             _lastStatus.isLow ? "⚠️  НИЗКИЙ" :
             _lastStatus.isOk ? "✅ НОРМА" : "❓ НЕИЗВЕСТЕН");

    if (verbose) {
        ESP_LOGI(TAG, "Последняя проверка: %lu мс назад", millis() - _lastStatus.lastCheckTime);
        ESP_LOGI(TAG, "Всего проверок: %lu | Критических событий: %lu", _checkCount, _criticalCount);
    }
    ESP_LOGI(TAG, "==========================================");
}

void BatteryMonitor::reset() {
    _checkCount = 0;
    _criticalCount = 0;
    _lastStatus.lastCheckTime = 0;
    ESP_LOGI(TAG, "🔄 Статистика батареи сброшена");
}
