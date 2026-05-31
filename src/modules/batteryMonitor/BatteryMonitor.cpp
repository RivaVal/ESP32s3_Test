/**
 * @file BatteryMonitor.cpp
 * @brief Реализация мониторинга АКБ — КЛАССИЧЕСКИЙ ADC API
 * @version 1.1.0 — Исправлены типы для совместимости с ESP-IDF 5.0+
 */
#include "modules/batteryMonitor/BatteryMonitor.h"
#include <cmath>

const char* BatteryMonitor::TAG = "BATTERY";

// ============================================================================
// Конструктор / Деструктор
// ============================================================================
BatteryMonitor::BatteryMonitor() {
    _lastStatus = {};
    _currentConfig = BATTERY_CONFIG_TABLE[1];  // 4S по умолчанию
    ESP_LOGI(TAG, "✅ BatteryMonitor: конструктор завершён");
}

BatteryMonitor::~BatteryMonitor() {
    // Классический драйвер не требует явного освобождения
    ESP_LOGI(TAG, "🔄 BatteryMonitor: ресурсы освобождены");
}

// ============================================================================
// 🔑 Маппинг GPIO → ADC_UNIT (ESP32 / ESP32-S3)
// ============================================================================
adc_unit_t BatteryMonitor::_gpioToAdcUnit(uint8_t gpio) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    // ESP32-S3: GPIO1-10 → ADC_UNIT_1 (с калибровкой)
    return (gpio >= 1 && gpio <= 10) ? ADC_UNIT_1 : ADC_UNIT_2;
#else
    // Классический ESP32: GPIO32-39 → ADC_UNIT_1
    return (gpio >= 32 && gpio <= 39) ? ADC_UNIT_1 : ADC_UNIT_2;
#endif
}

// 🔑 Возвращает adc1_channel_t (явный тип для ESP-IDF 5.0+)
adc1_channel_t BatteryMonitor::_gpioToAdc1Channel(uint8_t gpio) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    // ESP32-S3: GPIO1-10 → ADC1_CH0-9
    switch(gpio) {
        case 1:  return ADC1_CHANNEL_0;
        case 2:  return ADC1_CHANNEL_1;
        case 3:  return ADC1_CHANNEL_2;
        case 4:  return ADC1_CHANNEL_3;
        case 5:  return ADC1_CHANNEL_4;
        case 6:  return ADC1_CHANNEL_5;
        case 7:  return ADC1_CHANNEL_6;  // 🔑 Наш пин!
        case 8:  return ADC1_CHANNEL_7;
        case 9:  return ADC1_CHANNEL_8;
        case 10: return ADC1_CHANNEL_9;
        default: return ADC1_CHANNEL_0;
    }
#else
    // Классический ESP32
    switch(gpio) {
        case 32: return ADC1_CHANNEL_4;
        case 33: return ADC1_CHANNEL_5;
        case 34: return ADC1_CHANNEL_6;  // Оригинальный пин
        case 35: return ADC1_CHANNEL_7;
        case 36: return ADC1_CHANNEL_0;
        case 39: return ADC1_CHANNEL_3;
        default: return ADC1_CHANNEL_6;
    }
#endif
}

// ============================================================================
// 🔧 Инициализация ADC
// ============================================================================
bool BatteryMonitor::_initADC() {
    ESP_LOGI(TAG, "⚙️ Инициализация ADC (классический драйвер)...");
    
    const uint8_t pin = Config::Pins::BATTERY_ADC_PIN;
    _adcUnit = _gpioToAdcUnit(pin);
    _adcChannel = _gpioToAdc1Channel(pin);  // ← adc1_channel_t
    
    ESP_LOGI(TAG, "   📍 GPIO%d → ADC_UNIT_%d, CH%d", 
             pin, (_adcUnit == ADC_UNIT_1) ? 1 : 2, static_cast<int>(_adcChannel));
    
    // 🔑 Настройка пинов
    if (_adcUnit == ADC_UNIT_1) {
        ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
        // 🔑 Используем ADC_ATTEN_DB_12 (не DB_11 — депрекейтед)
        ESP_ERROR_CHECK(adc1_config_channel_atten(_adcChannel, ADC_ATTEN_DB_12));
    } else {
        // ADC2: без калибровки, используем с осторожностью
        ESP_LOGW(TAG, "⚠️ ADC2 не поддерживает esp_adc_cal! Точность ~±5%");
        ESP_ERROR_CHECK(adc2_config_channel_atten(
            static_cast<adc2_channel_t>(_adcChannel), ADC_ATTEN_DB_12));
    }
    
    // 🔑 Калибровка (только для ADC_UNIT_1)
    if (_adcUnit == ADC_UNIT_1) {
        esp_adc_cal_value_t val = esp_adc_cal_characterize(
            _adcUnit,
            ADC_ATTEN_DB_12,  // ← Тот же аттенюатор
            ADC_WIDTH_BIT_12,
            Config::Battery::ADC_VREF_MV,  // 1100 мВ из config
            &_adcChars
        );
        
        if (val == ESP_ADC_CAL_VAL_EFUSE_TP) {
            ESP_LOGI(TAG, "✅ Калибровка: две точки (наиболее точная)");
            _calibrated = true;
        } else if (val == ESP_ADC_CAL_VAL_EFUSE_VREF) {
            ESP_LOGI(TAG, "✅ Калибровка: одна точка (eFuse Vref)");
            _calibrated = true;
        } else {
            ESP_LOGW(TAG, "⚠️ Калибровка: дефолтные значения");
            _calibrated = false;
        }
    }
    
    return true;
}

// ============================================================================
// 🔧 Чтение ADC с усреднением
// ============================================================================
bool BatteryMonitor::_readADC(int& outValue) {
    if (!_initialized) return false;
    
    constexpr uint8_t SAMPLES = 16;
    int32_t sum = 0;
    
    for (uint8_t i = 0; i < SAMPLES; i++) {
        int raw = 0;
        
        if (_adcUnit == ADC_UNIT_1) {
            raw = adc1_get_raw(_adcChannel);  // ← adc1_channel_t ✓
        } else {
            // ADC2: блокирующий вызов
            adc2_get_raw(static_cast<adc2_channel_t>(_adcChannel), 
                        ADC_WIDTH_BIT_12, &raw);
        }
        
        // Фильтрация выбросов
        if (raw > 0 && raw < 4095) {
            sum += raw;
        }
        delay(2);
    }
    
    outValue = sum / SAMPLES;
    return true;
}

// ============================================================================
// 🔧 Преобразование ADC → напряжение батареи
// ============================================================================
float BatteryMonitor::_adcToVoltage(int adcRaw) const {
    // 🔑 Применяем калибровку, если доступна
    uint32_t voltageMV = 0;
    if (_calibrated && _adcUnit == ADC_UNIT_1) {
        voltageMV = esp_adc_cal_raw_to_voltage(adcRaw, &_adcChars);
    } else {
        // Дефолтное преобразование: 3.3В референс, 12 бит
        voltageMV = (adcRaw * 3300) >> 12;
    }
    
    // Переводим в напряжение на пине (В) и учитываем делитель
    float pinVoltage = voltageMV / 1000.0f;
    return pinVoltage * Config::Battery::BATTERY_VOLTAGE_DIVIDER_RATIO;
}

// ============================================================================
// 🔧 Остальные приватные методы (без изменений)
// ============================================================================
float BatteryMonitor::_calcPercentage(float voltage) const {
    if (voltage <= _currentConfig.voltageMin) return 0.0f;
    if (voltage >= _currentConfig.voltageMax) return 100.0f;
    return ((voltage - _currentConfig.voltageMin) / 
            (_currentConfig.voltageMax - _currentConfig.voltageMin)) * 100.0f;
}

void BatteryMonitor::_updateStatus(float voltage) {
    _lastStatus.isCritical = (voltage <= _currentConfig.voltageCritical);
    _lastStatus.isLow = (voltage <= _currentConfig.voltageMin);
    _lastStatus.isOk = (voltage > _currentConfig.voltageMin && 
                        voltage <= _currentConfig.voltageMax);
}

const BatteryConfig_t* BatteryMonitor::_findConfig(uint8_t cells) const {
    for (uint8_t i = 0; i < BATTERY_CONFIG_COUNT; i++) {
        if (BATTERY_CONFIG_TABLE[i].cellCount == cells) {
            return &BATTERY_CONFIG_TABLE[i];
        }
    }
    return nullptr;
}

// ============================================================================
// 🚀 ПУБЛИЧНЫЕ МЕТОДЫ
// ============================================================================
bool BatteryMonitor::begin(uint8_t cellCount) {
    ESP_LOGI(TAG, "=== 🚀 Инициализация BatteryMonitor ===");
    
    if (cellCount < MIN_CELL_COUNT || cellCount > MAX_CELL_COUNT) {
        ESP_LOGE(TAG, "❌ Недопустимое количество ячеек: %u", cellCount);
        return false;
    }
    
    const BatteryConfig_t* cfg = _findConfig(cellCount);
    if (!cfg) {
        ESP_LOGE(TAG, "❌ Конфигурация для %uS не найдена", cellCount);
        return false;
    }
    _currentConfig = *cfg;
    
    ESP_LOGI(TAG, "🔋 Конфигурация: %s | Диапазон: %.2fВ - %.2fВ",
             _currentConfig.name, _currentConfig.voltageMin, _currentConfig.voltageMax);
    
    if (!_initADC()) {
        ESP_LOGE(TAG, "❌ Ошибка инициализации ADC");
        return false;
    }
    
    _initialized = true;
    delay(100);
    checkVoltage();
    
    ESP_LOGI(TAG, "✅ BatteryMonitor готов");
    return true;
}

BatteryStatus_t BatteryMonitor::checkVoltage() {
    if (!_initialized) return _lastStatus;
    
    int adcValue = 0;
    if (!_readADC(adcValue)) return _lastStatus;
    
    float voltage = _adcToVoltage(adcValue);
    float percentage = _calcPercentage(voltage);
    float perCell = voltage / _currentConfig.cellCount;
    
    _lastStatus.voltage = voltage;
    _lastStatus.percentage = percentage;
    _lastStatus.voltagePerCell = perCell;
    _lastStatus.adcValue = static_cast<uint16_t>(adcValue);
    _lastStatus.cellCount = _currentConfig.cellCount;
    _lastStatus.configName = _currentConfig.name;
    _lastStatus.lastCheckTime = millis();
    
    _updateStatus(voltage);
    _checkCount++;
    
    if (_lastStatus.isCritical) {
        _criticalCount++;
        ESP_LOGE(TAG, "🔴 КРИТИЧЕСКИ НИЗКО: %.2fВ (%.1f%%)", voltage, percentage);
    }
    
    return _lastStatus;
}

void BatteryMonitor::update() {
    if (!_initialized) return;
    if (millis() - _lastCheckTime >= Config::Battery::CHECK_INTERVAL_MS) {
        checkVoltage();
        _lastCheckTime = millis();
    }
}

// ... остальные публичные методы (setCellCount, printStatus, resetStats) 
// ... оставляем без изменений, как в предыдущей версии

//====================

bool BatteryMonitor::setCellCount(uint8_t cellCount) {
    if (cellCount < MIN_CELL_COUNT || cellCount > MAX_CELL_COUNT) {
        ESP_LOGE(TAG, "❌ setCellCount: недопустимое значение %u", cellCount);
        return false;
    }
    
    const BatteryConfig_t* cfg = _findConfig(cellCount);
    if (!cfg) return false;
    
    _currentConfig = *cfg;
    ESP_LOGI(TAG, "✅ Конфигурация изменена: %s", _currentConfig.name);
    
    // Немедленная перепроверка
    checkVoltage();
    return true;
}

void BatteryMonitor::printStatus(bool verbose) const {
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "🔋 СТАТУС БАТАРЕИ");
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "Конфигурация: %s (%uS)", _lastStatus.configName, _lastStatus.cellCount);
    ESP_LOGI(TAG, "Напряжение: %.2f В | На ячейку: %.2f В", 
             _lastStatus.voltage, _lastStatus.voltagePerCell);
    ESP_LOGI(TAG, "Заряд: %.1f %% | ADC: %u", 
             _lastStatus.percentage, _lastStatus.adcValue);
    
    const char* statusStr = _lastStatus.isCritical ? "🔴 КРИТИЧЕСКИЙ" :
                           _lastStatus.isLow ? "⚠️  НИЗКИЙ" :
                           _lastStatus.isOk ? "✅ НОРМА" : "❓ НЕИЗВЕСТЕН";
    ESP_LOGI(TAG, "Статус: %s", statusStr);
    
    if (verbose) {
        ESP_LOGI(TAG, "Проверок: %lu | Критических: %lu | Последняя: %lu мс назад",
                 _checkCount, _criticalCount, millis() - _lastStatus.lastCheckTime);
    }
    ESP_LOGI(TAG, "==========================================");
}

void BatteryMonitor::resetStats() {
    _checkCount = 0;
    _criticalCount = 0;
    _lastCheckTime = 0;
    ESP_LOGI(TAG, "🔄 Статистика сброшена");
}
