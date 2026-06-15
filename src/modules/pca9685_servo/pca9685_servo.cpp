/**
 * @file pca9685_servo.cpp
 * @brief Реализация контроллера сервоприводов через PCA9685
 * @details
 * - Адаптирован под ESP32-S3 и нативный I2C драйвер
 * - Все операции с подробной отладкой через ESP_LOG
 * - Предстартовый тест: 3 цикла, все каналы одновременно
 * - Углы безопасности и калибровка каждого канала
 *
 * @version 1.0 (миграция из Arduino IDE)
 * @date 2026-05-26
 */

#include "pca9685_servo.h"
#include <cstring>
#include <cmath>

// ============================================================================
// Глобальная последовательность теста (определена здесь)
// ============================================================================
const int16_t SERVO_TEST_SEQUENCE[] = {0, 30, 60, 0, -30, -60, 0};
const uint8_t SERVO_TEST_SEQUENCE_SIZE = 7;

// ============================================================================
// Статический тег для отладки
// ============================================================================
const char* PCA9685ServoController::TAG = "PCA9685_SERVO";

// ============================================================================
// Конструктор
// ============================================================================
PCA9685ServoController::PCA9685ServoController(uint8_t i2c_address)
    : _i2c_address(i2c_address) {

    // Инициализация каналов по умолчанию (0-6)
    for (uint8_t i = 0; i < 7; i++) {
        _servo_channels[i] = i;
    }

    // Инициализация калибровки
    for (uint8_t i = 0; i < 7; i++) {
        _servo_trim[i] = 0;
    }

    _initialized = false;
    _chip_connected = false;

    ESP_LOGI(TAG, "✅ Конструктор: адрес=0x%02X, каналы=0-6", _i2c_address);
    ESP_LOGI(TAG, "   Параметры серво: min=%u мкс, neutral=%u мкс, max=%u мкс",
             _servo_params.min_pulse_us,
             _servo_params.neutral_pulse_us,
             _servo_params.max_pulse_us);
}

// ============================================================================
// Инициализация (Adafruit-style sequence)
// ============================================================================
bool PCA9685ServoController::begin(I2CMasterController& i2c_manager) {
    ESP_LOGI(TAG, "=== 🚀 ИНИЦИАЛИЗАЦИЯ PCA9685 (7 серво) ===");
    ESP_LOGI(TAG, "   I2C адрес: 0x%02X, Частота: %u Гц",
             _i2c_address, _servo_params.frequency);
    ESP_LOGI(TAG, "   🔑 Диапазон импульсов: %u-%u мкс",
             _servo_params.min_pulse_us, _servo_params.max_pulse_us);

    _i2c_manager = &i2c_manager;

    // 1. Проверка подключения
    if (!_i2c_manager->isDeviceConnected(_i2c_address)) {
        ESP_LOGE(TAG, "❌ PCA9685 не отвечает на 0x%02X", _i2c_address);
        return false;
    }
    _chip_connected = true;
    ESP_LOGI(TAG, "✅ PCA9685 обнаружен на 0x%02X", _i2c_address);

    // 2. Чтение MODE1 до инициализации
    uint8_t mode1;
    if (!_readRegister(PCA9685_MODE1, &mode1, 1)) {
        ESP_LOGE(TAG, "❌ Не удалось прочитать MODE1");
        return false;
    }
    ESP_LOGI(TAG, "   MODE1 до инициализации: 0x%02X (SLEEP=%d)",
             mode1, (mode1 & PCA9685_MODE1_SLEEP) ? 1 : 0);

    // 3. Переход в SLEEP режим
    uint8_t sleep_mode = (mode1 & 0x7F) | PCA9685_MODE1_SLEEP;
    if (!_writeRegister(PCA9685_MODE1, &sleep_mode, 1)) {
        ESP_LOGE(TAG, "❌ Не удалось установить SLEEP режим");
        return false;
    }
    ESP_LOGI(TAG, "✅ SLEEP режим установлен");
    delay(5);

    // 4. Установка prescaler для 50 Гц
    // prescale = round(osc_clock / (4096 * freq)) - 1
    const float osc_clock = 25000000.0f;
    const float target_freq = 50.0f;
    uint8_t prescale = static_cast<uint8_t>(
        (osc_clock / (4096.0f * target_freq)) - 1.0f + 0.5f
    );
    ESP_LOGI(TAG, "   Prescaler: %u для %.1f Гц", prescale, target_freq);

    if (!_writeRegister(PCA9685_PRESCALE, &prescale, 1)) {
        ESP_LOGE(TAG, "❌ Не удалось установить prescaler");
        return false;
    }

    // 5. Выход из SLEEP + AUTO_INCREMENT
    uint8_t wake_mode = PCA9685_MODE1_AUTO_INC | PCA9685_MODE1_RESTART;
    ESP_LOGI(TAG, "   🔑 Запись MODE1=0x%02X (AUTO_INCREMENT=1, SLEEP=0)", wake_mode);
    if (!_writeRegister(PCA9685_MODE1, &wake_mode, 1)) {
        ESP_LOGE(TAG, "❌ Не удалось выйти из SLEEP режима");
        return false;
    }
    ESP_LOGI(TAG, "✅ Выход из SLEEP (MODE1=0x%02X)", wake_mode);

    // 6. Настройка MODE2: totem-pole output (ОБЯЗАТЕЛЬНО для серво!)
    ESP_LOGI(TAG, "⚙️  Настройка MODE2 (режим выхода для серво)...");
    uint8_t mode2 = PCA9685_MODE2_OUTDRV;  // 0b00000100 = OUTDRV=1

    bool mode2_ok = false;
    for (uint8_t retry = 0; retry < 3; retry++) {
        if (_writeRegister(PCA9685_MODE2, &mode2, 1)) {
            delay(5);
            uint8_t mode2_verify;
            if (_readRegister(PCA9685_MODE2, &mode2_verify, 1)) {
                if ((mode2_verify & PCA9685_MODE2_OUTDRV) == PCA9685_MODE2_OUTDRV) {
                    mode2_ok = true;
                    ESP_LOGI(TAG, "   ✅ MODE2 записан успешно (0x%02X)", mode2_verify);
                    break;
                }
            }
        }
        delay(10);
    }

    if (!mode2_ok) {
        ESP_LOGE(TAG, "❌ КРИТИЧЕСКАЯ ОШИБКА: Не удалось установить MODE2!");
        ESP_LOGE(TAG, "   Сервоприводы могут вращаться непрерывно!");
        return false;
    }

    // 7. Задержка стабилизации осциллятора
    ESP_LOGI(TAG, "⏳ Стабилизация осциллятора (1500 мс)...");
    delay(1500);

    // 8. Проверка запуска
    if (!_readRegister(PCA9685_MODE1, &mode1, 1)) {
        ESP_LOGE(TAG, "❌ Не удалось прочитать MODE1 для проверки");
        return false;
    }
    if (mode1 & PCA9685_MODE1_SLEEP) {
        ESP_LOGE(TAG, "❌ Осциллятор НЕ запустился! (SLEEP=1)");
        return false;
    }
    ESP_LOGI(TAG, "✅ Осциллятор стабилизирован");

    // 9. Финализация
    _initialized = true;
    resetToNeutral();  // Сброс в нейтраль для безопасности

    ESP_LOGI(TAG, "✅ PCA9685 успешно инициализирован!");
    return true;
}

// ============================================================================
// Преобразования и низкоуровневые методы
// ============================================================================

uint16_t PCA9685ServoController::_pulseUsToTicks(uint16_t pulse_us) const {
    // ticks = pulse_us * 4096 / 20000 (период при 50 Гц = 20000 мкс)
    uint32_t ticks = (static_cast<uint32_t>(pulse_us) * 4096UL) / 20000UL;

    // Ограничение 12 битами
    if (ticks > 4095) ticks = 4095;
    if (ticks < 1) ticks = 1;  // 0 = Full Off

    ESP_LOGV(TAG, "_pulseUsToTicks: %u мкс → %u тиков", pulse_us, ticks);
    return static_cast<uint16_t>(ticks);
}

uint16_t PCA9685ServoController::_logicalAngleToPulseUs(int16_t logical_angle) const {
    // Ограничение входного угла
    if (logical_angle < -90) logical_angle = -90;
    if (logical_angle > 90) logical_angle = 90;

    // Линейная интерполяция: pulse = neutral + angle * (max - min) / 180
    int32_t pulse_width = _servo_params.neutral_pulse_us +
        (static_cast<int32_t>(logical_angle) *
         (_servo_params.max_pulse_us - _servo_params.min_pulse_us)) / 180;

    // Ограничение физическими пределами
    if (pulse_width < static_cast<int32_t>(_servo_params.min_pulse_us)) {
        pulse_width = _servo_params.min_pulse_us;
    }
    if (pulse_width > static_cast<int32_t>(_servo_params.max_pulse_us)) {
        pulse_width = _servo_params.max_pulse_us;
    }

    ESP_LOGV(TAG, "_logicalAngleToPulseUs: %+d° → %ld мкс",
             logical_angle, pulse_width);
    return static_cast<uint16_t>(pulse_width);
}

int16_t PCA9685ServoController::_validateSafetyAngle(int16_t angle) const {
    // Ограничение максимального угла
    if (angle > SERVO_SAFE_MAX_ANGLE) {
        ESP_LOGV(TAG, "⚠️ Угол %+d° > +%d°, ограничено", angle, SERVO_SAFE_MAX_ANGLE);
        angle = SERVO_SAFE_MAX_ANGLE;
    }
    if (angle < -SERVO_SAFE_MAX_ANGLE) {
        ESP_LOGV(TAG, "⚠️ Угол %+d° < -%d°, ограничено", angle, -SERVO_SAFE_MAX_ANGLE);
        angle = -SERVO_SAFE_MAX_ANGLE;
    }

    // Буферная зона вокруг нейтрали
    if (angle > 0 && angle < SERVO_NEUTRAL_BUFFER) {
        angle = SERVO_NEUTRAL_BUFFER;
    }
    if (angle < 0 && angle > -SERVO_NEUTRAL_BUFFER) {
        angle = -SERVO_NEUTRAL_BUFFER;
    }

    return angle;
}

bool PCA9685ServoController::_setPWM(uint8_t channel, uint16_t on_tick, uint16_t off_tick) {
    if (!_initialized || channel > 15) {
        ESP_LOGE(TAG, "❌ _setPWM: невалидные параметры (ch=%u)", channel);
        return false;
    }

    uint8_t reg_base = PCA9685_LED0_ON_L + 4 * channel;
    uint8_t data[4];
    data[0] = on_tick & 0xFF;
    data[1] = (on_tick >> 8) & 0x0F;   // Mask bits 4-7
    data[2] = off_tick & 0xFF;
    data[3] = (off_tick >> 8) & 0x0F;  // Mask bits 4-7

    if (!_i2c_manager->writeRegister(_i2c_address, reg_base, data, 4)) {
        ESP_LOGE(TAG, "❌ Ошибка записи PWM канал %u", channel);
        return false;
    }

    ESP_LOGV(TAG, "_setPWM[%u]: on=%u, off=%u", channel, on_tick, off_tick);
    return true;
}

bool PCA9685ServoController::_readRegister(uint8_t reg_addr, uint8_t* data, size_t len) {
    return _i2c_manager->readRegister(_i2c_address, reg_addr, data, len);
}

bool PCA9685ServoController::_writeRegister(uint8_t reg_addr, const uint8_t* data, size_t len) {
    return _i2c_manager->writeRegister(_i2c_address, reg_addr, data, len);
}

// ============================================================================
// Публичные методы управления
// ============================================================================
/*
bool PCA9685ServoController::setLogicalAngle(uint8_t servo_index, int16_t angle) {
    if (!_initialized || servo_index >= 7) {
        ESP_LOGE(TAG, "❌ setLogicalAngle: невалидные параметры");
        return false;
    }

    int16_t safe_angle = _validateSafetyAngle(angle);
    uint16_t pulse_us = _logicalAngleToPulseUs(safe_angle);
    pulse_us += _servo_trim[servo_index];

    // Ограничение физическими пределами
    if (pulse_us < _servo_params.min_pulse_us) pulse_us = _servo_params.min_pulse_us;
    if (pulse_us > _servo_params.max_pulse_us) pulse_us = _servo_params.max_pulse_us;

    uint16_t ticks = _pulseUsToTicks(pulse_us);

    // 🔑 Отладочная печать
    _printPWMDebug(servo_index, safe_angle, pulse_us, ticks);

    ESP_LOGV(TAG, "setLogicalAngle[%u]: %+d° → %u мкс → %u тиков",
             servo_index, safe_angle, pulse_us, ticks);

    return _setPWM(_servo_channels[servo_index], 0, ticks);
}
*/

bool PCA9685ServoController::setLogicalAngle(uint8_t servo_index, int16_t angle, bool enable_debug_log) {
    if (!_initialized || servo_index >= 7) {
        ESP_LOGE(TAG, "❌ setLogicalAngle: невалидные параметры");
        return false;
    }

    int16_t safe_angle = _validateSafetyAngle(angle);
    uint16_t pulse_us = _logicalAngleToPulseUs(safe_angle);
    pulse_us += _servo_trim[servo_index];

    // Ограничение физическими пределами
    if (pulse_us < _servo_params.min_pulse_us) pulse_us = _servo_params.min_pulse_us;
    if (pulse_us > _servo_params.max_pulse_us) pulse_us = _servo_params.max_pulse_us;

    uint16_t ticks = _pulseUsToTicks(pulse_us);

    // 🔑 ИЗМЕНЕНО: Выводим отладку только если разрешено
    if (enable_debug_log) {
        _printPWMDebug(servo_index, safe_angle, pulse_us, ticks);
    }

    ESP_LOGV(TAG, "setLogicalAngle[%u]: %+d° → %u мкс → %u тиков",
             servo_index, safe_angle, pulse_us, ticks);

    return _setPWM(_servo_channels[servo_index], 0, ticks);
}

bool PCA9685ServoController::setPhysicalAngle(uint8_t servo_index, uint16_t physical_angle) {
    if (!_initialized || servo_index >= 7) return false;
    if (physical_angle > 180) physical_angle = 180;

    int16_t logical_angle = static_cast<int16_t>(physical_angle) - 90;
    return setLogicalAngle(servo_index, logical_angle);
}

bool PCA9685ServoController::setPulseWidthUs(uint8_t servo_index, uint16_t pulse_us) {
    if (!_initialized || servo_index >= 7) return false;

    if (pulse_us < _servo_params.min_pulse_us) pulse_us = _servo_params.min_pulse_us;
    if (pulse_us > _servo_params.max_pulse_us) pulse_us = _servo_params.max_pulse_us;

    uint16_t ticks = _pulseUsToTicks(pulse_us);
    return _setPWM(servo_index, 0, ticks);
}

void PCA9685ServoController::setServoTrim(uint8_t servo_index, int16_t trim_us) {
    if (servo_index < 7) {
        _servo_trim[servo_index] = trim_us;
        ESP_LOGI(TAG, "✅ Серво #%u: калибровка = %+d мкс", servo_index, trim_us);
    }
}

void PCA9685ServoController::resetToNeutral() {
    if (!_initialized) {
        ESP_LOGW(TAG, "⚠️ resetToNeutral: контроллер не инициализирован");
        return;
    }

    ESP_LOGI(TAG, "🔄 Сброс всех сервоприводов в нейтраль (1500 мкс)");
    for (uint8_t i = 0; i < 7; i++) {
        setLogicalAngle(i, 0);
        delay(50);  // 🔑 50 мс между каналами (снижает пиковый ток!)
    }
    ESP_LOGI(TAG, "✅ Все сервоприводы в нейтрали");
}

// ============================================================================
// 🔑 Предстартовый тест сервоприводов
// ============================================================================
bool PCA9685ServoController::runServoTest() {
    if (!_initialized) {
        ESP_LOGE(TAG, "❌ runServoTest: контроллер не инициализирован!");
        return false;
    }

    // ========================================================================
    // ШАГ 1: Вывод параметров теста
    // ========================================================================
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "⚙️  ПРЕДСТАРТОВЫЙ ТЕСТ СЕРВОПРИВОДОВ");
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "📊 ПАРАМЕТРЫ:");
    ESP_LOGI(TAG, "   • PCA9685 адрес: 0x%02X", _i2c_address);
    ESP_LOGI(TAG, "   • Каналы: 0-6 (7 сервоприводов)");
    ESP_LOGI(TAG, "   • Частота PWM: %u Гц", _servo_params.frequency);
    ESP_LOGI(TAG, "   • Импульсы: %u-%u мкс",
             _servo_params.min_pulse_us, _servo_params.max_pulse_us);
    ESP_LOGI(TAG, "   • Циклов теста: %d", SERVO_TEST_CYCLES);
    ESP_LOGI(TAG, "   • Задержка шага: %d мс", SERVO_TEST_STEP_DELAY_MS);
    
    delay(100); // 🔑 ДОБАВЛЕНО: Задержка для USB CDC

    // Расчётные значения для ключевых углов
    ESP_LOGI(TAG, "📋 РАСЧЁТНЫЕ ЗНАЧЕНИЯ PWM:");
    ESP_LOGI(TAG, "   Угол -90°: %u мкс → %u тиков",
             _servo_params.min_pulse_us, _pulseUsToTicks(_servo_params.min_pulse_us));
    ESP_LOGI(TAG, "   Угол   0°: %u мкс → %u тиков",
             _servo_params.neutral_pulse_us, _pulseUsToTicks(_servo_params.neutral_pulse_us));
    ESP_LOGI(TAG, "   Угол +90°: %u мкс → %u тиков",
             _servo_params.max_pulse_us, _pulseUsToTicks(_servo_params.max_pulse_us));
    ESP_LOGI(TAG, "==========================================");

    // ========================================================================
    // ШАГ 2: Основной цикл теста
    // ========================================================================
    for (uint8_t cycle = 0; cycle < SERVO_TEST_CYCLES; cycle++) {
        ESP_LOGI(TAG, "🔄 ЦИКЛ ТЕСТА №%d из %d", cycle + 1, SERVO_TEST_CYCLES);

        for (uint8_t step = 0; step < SERVO_TEST_SEQUENCE_SIZE; step++) {
            int16_t target_angle = SERVO_TEST_SEQUENCE[step];

            // Все 7 сервоприводов двигаются ОДНОВРЕМЕННО!
            for (uint8_t servo_index = 0; servo_index < 7; servo_index++) {
                //setLogicalAngle(servo_index, target_angle);

                // 🔑 ИЗМЕНЕНО: Передаём только первый серво для детального лога
                if (servo_index == 0) {
                    setLogicalAngle(servo_index, target_angle, true);  // Подробный лог
                } else {
                    setLogicalAngle(servo_index, target_angle, false); // Без лога
                }
             }

                         // 🔑 ИЗМЕНЕНО: Выводим только один раз для шага
                    //        const char* label;
                    //        if (target_angle == 0) {
                    //            label = "НЕЙТРАЛЬ";
                    //        } else if (target_angle > 0) {
                    //            label = "МАКС+";
                   //         } else {
                    //            label = "МАКС-";
                    //        }
                    //        ESP_LOGI(TAG, "   📍 Шаг %u: %+3d° [%s] - все 7 серво",
                    //                step, target_angle, label);
                            
                    //        delay(50); // 🔑 ДОБАВЛЕНО: Задержка для USB CDC
                    //        delay(SERVO_TEST_STEP_DELAY_MS);

            // Отладочный вывод
            const char* label;
            if (target_angle == 0) {
                label = "НЕЙТРАЛЬ";
            } else if (target_angle > 0) {
                label = "МАКС+";
            } else {
                label = "МАКС-";
            }
            ESP_LOGI(TAG, "   📍 Шаг %u: %+3d° [%s] - все 7 серво",
                     step, target_angle, label);

            delay(50); // 🔑 ДОБАВЛЕНО: Задержка для USB CDC
             delay(SERVO_TEST_STEP_DELAY_MS);
        }

        if (cycle < SERVO_TEST_CYCLES - 1) {
            ESP_LOGI(TAG, "⏳ Пауза между циклами: %d мс", SERVO_TEST_PAUSE_MS);
            delay(50); // 🔑 ДОБАВЛЕНО: Задержка для USB CDC
            delay(SERVO_TEST_PAUSE_MS);
        }
    }

    // ========================================================================
    // ШАГ 3: Финальный сброс
    // ========================================================================
    ESP_LOGI(TAG, "🔄 Финальный сброс в нейтраль (0°)...");
    resetToNeutral();
    delay(500);

    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "✅ ТЕСТ СЕРВОПРИВОДОВ ЗАВЕРШЁН УСПЕШНО!");
    ESP_LOGI(TAG, "==========================================");
    return true;
}

// ============================================================================
// Обработка команд пульта
// ============================================================================
void PCA9685ServoController::processFlightCommands(uint8_t com_up, uint8_t com_left) {
    if (!_initialized) {
        ESP_LOGW(TAG, "⚠️ processFlightCommands: контроллер не инициализирован");
        return;
    }

    // Преобразование 0-180 → логический диапазон -90°...+90°
    int16_t pitch_angle = map(com_up, 0, 180, 90, -90);
    int16_t yaw_angle   = map(com_left, 0, 180, 90, -90);

    ESP_LOGD(TAG, "📡 RC Commands: Up=%3d→P=%+3d°, Left=%3d→Y=%+3d°",
             com_up, pitch_angle, com_left, yaw_angle);

    // 🎯 Каналы 0,1: Рули высоты (синхронно)
    setLogicalAngle(0, pitch_angle);
    setLogicalAngle(1, pitch_angle);

    // 🎯 Каналы 2,3: Рули направления (инверсно)
    setLogicalAngle(2, yaw_angle);
    setLogicalAngle(3, -yaw_angle);

    // 🎯 Каналы 4,5,6: Вспомогательные серво (парашют, крышки)
    int16_t hatch_angle = map(com_up, 0, 180, -90, 90);
    setLogicalAngle(4, hatch_angle);
    setLogicalAngle(5, hatch_angle);
    setLogicalAngle(6, hatch_angle);
}

// ============================================================================
// Отладочная печать
// ============================================================================
void PCA9685ServoController::_printPWMDebug(uint8_t servo_index, int16_t angle,
                                           uint16_t pulse_us, uint16_t ticks) {
    ESP_LOGD(TAG, "   🔍 PWM DEBUG: Серво #%u | Угол: %+3d° | Импульс: %u мкс | Тики: %u",
             servo_index, angle, pulse_us, ticks);

    // Расчёт фактической длительности
    float actual_us = (ticks * 20000.0f) / 4096.0f;
    ESP_LOGD(TAG, "   📊 Фактическая длительность: %.1f мкс", actual_us);

    // Проверка расхождения
    float diff = static_cast<float>(pulse_us) - actual_us;
    if (fabsf(diff) > 5.0f) {
        ESP_LOGW(TAG, "   ⚠️ Расхождение: задано %u мкс, фактически %.1f мкс (Δ=%.1f мкс)",
                 pulse_us, actual_us, diff);
    }

    // Проверка диапазона
    if (pulse_us < _servo_params.min_pulse_us ||
        pulse_us > _servo_params.max_pulse_us) {
        ESP_LOGW(TAG, "   ⚠️ ВНИМАНИЕ: Импульс вне диапазона %u-%u мкс!",
                 _servo_params.min_pulse_us, _servo_params.max_pulse_us);
    }
}
