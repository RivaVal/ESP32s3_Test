
//### 📁 2. `modules/unified_ledc/unified_ledc.cpp`
// Реализация с пошаговой диагностикой `ESP_LOG`, обработкой ошибок ESP-IDF и защитой от переполнения.
//

/**
 * @file unified_ledc.cpp
 * @brief Реализация контроллера моторов через LEDC ESP32-S3
 * @details Содержит логику инициализации аппаратного ШИМ, расчёта duty-cycle и безопасных тестов.
 */
#include "unified_ledc.h"

static const char* TAG = "LEDC_CTRL";

UnifiedLEDCController::UnifiedLEDCController() {
    ESP_LOGD(TAG, "🔧 Конструктор UnifiedLEDCController создан");
}

bool UnifiedLEDCController::begin() {
    if (_initialized) {
        ESP_LOGW(TAG, "⚠️ LEDC уже инициализирован. Повторный вызов пропущен.");
        return true;
    }

    ESP_LOGI(TAG, "⚙️ Инициализация LEDC для 2 моторов...");
    ESP_LOGI(TAG, "   📍 Пины: M0→GPIO%d, M1→GPIO%d", _motorPins[0], _motorPins[1]);
    ESP_LOGI(TAG, "   ⚡ Частота: %lu Гц | Разрешение: %lu бит", _freqHz, _dutyResolution);

    // 1️⃣ Настройка таймера
    esp_err_t err = setupTimer();
    if (err != ESP_OK) {
        _lastError = esp_err_to_name(err);
        ESP_LOGE(TAG, "❌ Настройка таймера LEDC не удалась: %s", _lastError);
        return false;
    }
    ESP_LOGD(TAG, "✅ Таймер LEDC настроен");

    // 2️⃣ Настройка каналов
    for (int i = 0; i < 2; i++) {
        err = setupChannel(i);
        if (err != ESP_OK) {
            _lastError = esp_err_to_name(err);
            ESP_LOGE(TAG, "❌ Настройка канала %d не удалась: %s", i, _lastError);
            stopAllMotors(); // Сброс в безопасное состояние
            return false;
        }
    }

    _initialized = true;
    ESP_LOGI(TAG, "✅ LEDC успешно инициализирован. Моторы в режиме STOP (0%)");
    return true;
}

bool UnifiedLEDCController::setMotorPower(uint8_t motorIndex, float powerPercent) {
    if (!_initialized) {
        ESP_LOGE(TAG, "❌ setMotorPower: контроллер не инициализирован!");
        return false;
    }
    if (motorIndex >= 2) {
        ESP_LOGE(TAG, "❌ setMotorPower: неверный индекс %u (допустимо 0-1)", motorIndex);
        return false;
    }

    // 🔒 Ограничение диапазона 0-100%
    powerPercent = constrain(powerPercent, 0.0f, 100.0f);

    // 📐 Преобразование % → 10-битное значение (0-1023)
    uint32_t duty = static_cast<uint32_t>((powerPercent * 1023.0f) / 100.0f);

    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, _channels[motorIndex], duty);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ ledc_set_duty для мотора %u: %s", motorIndex, esp_err_to_name(err));
        return false;
    }

    err = ledc_update_duty(LEDC_LOW_SPEED_MODE, _channels[motorIndex]);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ ledc_update_duty для мотора %u: %s", motorIndex, esp_err_to_name(err));
        return false;
    }

    ESP_LOGD(TAG, "🔋 Motor #%u: %.1f%% → duty=%lu/1023", motorIndex, powerPercent, duty);
    return true;
}

void UnifiedLEDCController::stopAllMotors() {
    if (!_initialized) return;
    ESP_LOGW(TAG, "🛑 Аварийная остановка всех моторов!");
    setMotorPower(0, 0.0f);
    setMotorPower(1, 0.0f);
}

bool UnifiedLEDCController::runStartupDiagnostic(float maxPowerPercent, uint32_t durationMs) {
    if (!_initialized) return false;

    // 🔒 Аппаратный ограничитель безопасности (не более 35%)
    maxPowerPercent = constrain(maxPowerPercent, 0.0f, 35.0f);

    ESP_LOGI(TAG, "🧪 Запуск предстартового теста моторов (MAX: %.1f%%)", maxPowerPercent);
    ESP_LOGW(TAG, "⚠️ Убедитесь, что БПЛА закреплён и винты свободны!");
    delay(2000); // Задержка для осознания оператора

    auto rampMotor = [this](uint8_t idx, float limit) {
        ESP_LOGD(TAG, "   Разгон M%d: 0%% → %.1f%%", idx, limit);
        for (int p = 0; p <= (int)limit; p += 5) { setMotorPower(idx, p); delay(40); }
        ESP_LOGD(TAG, "   Торможение M%d: %.1f%% → 0%%", idx, limit);
        for (int p = (int)limit; p >= 0; p -= 5) { setMotorPower(idx, p); delay(40); }
    };

    rampMotor(0, maxPowerPercent);
    rampMotor(1, maxPowerPercent);

    ESP_LOGI(TAG, "✅ Тест моторов завершён. Все моторы остановлены.");
    stopAllMotors();
    return true;
}

// 🔧 Внутренние методы настройки LEDC
esp_err_t UnifiedLEDCController::setupTimer() {
    ledc_timer_config_t timer_conf = {};
    timer_conf.speed_mode      = LEDC_LOW_SPEED_MODE;
    timer_conf.timer_num       = LEDC_TIMER_0;
    timer_conf.duty_resolution = static_cast<ledc_timer_bit_t>(_dutyResolution);
    timer_conf.freq_hz         = _freqHz;
    timer_conf.clk_cfg         = LEDC_AUTO_CLK;
    return ledc_timer_config(&timer_conf);
}

esp_err_t UnifiedLEDCController::setupChannel(uint8_t index) {
    ledc_channel_config_t ch_conf = {};
    ch_conf.gpio_num     = _motorPins[index];
    ch_conf.speed_mode   = LEDC_LOW_SPEED_MODE;
    ch_conf.channel      = _channels[index];
    ch_conf.timer_sel    = LEDC_TIMER_0;
    ch_conf.duty         = 0; // Старт с 0%
    ch_conf.hpoint       = 0;
    esp_err_t err = ledc_channel_config(&ch_conf);
    if (err == ESP_OK) ledc_set_duty(LEDC_LOW_SPEED_MODE, _channels[index], 0);
    return err;
}
