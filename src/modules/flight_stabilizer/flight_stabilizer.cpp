

/**
 * @file flight_stabilizer.cpp
 * @brief Реализация ПИД-стабилизации с адаптивными параметрами
 * @version 2.0.0 (ESP32-S3 адаптация)
 */
// **Файл:** `src/modules/flight_stabilizer/flight_stabilizer.cpp`
//
#include "modules/flight_stabilizer/FlightStabilizer.h"
#include <math.h>

const char* FlightStabilizer::TAG = "FLIGHT_STAB";

// 🔑 ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ДЛЯ ПРИЕМА КОМАНД ИЗ loop()
// Они обновляются в main.cpp при получении пакета по LoRa
extern volatile float g_comUp;
extern volatile float g_comLeft;

// ... (Конструктор, деструктор остаются без изменений) ...

FlightStabilizer::FlightStabilizer() {
    // Инициализация ПИД по умолчанию (значения требуют тюнинга!)
    _rollPID = {2.0f, 0.0f, 0.5f, 10.0f, 30.0f, 0.2f};
    _pitchPID = {2.0f, 0.0f, 0.5f, 10.0f, 30.0f, 0.2f};
    _yawPID = {1.5f, 0.0f, 0.3f, 10.0f, 45.0f, 0.2f};

    // Сохраняем базовые значения для адаптивности
    _baseRollPID = _rollPID;
    _basePitchPID = _pitchPID;

    resetPIDStates();

    ESP_LOGI(TAG, "✅ FlightStabilizer: конструктор завершён");
    ESP_LOGD(TAG, "   Roll PID: Kp=%.2f Ki=%.2f Kd=%.2f",
             _rollPID.kp, _rollPID.ki, _rollPID.kd);
}

FlightStabilizer::~FlightStabilizer() {
    if (_taskHandle) {
        vTaskDelete(_taskHandle);
        _taskHandle = nullptr;
    }
    ESP_LOGI(TAG, "🔄 Ресурсы FlightStabilizer освобождены");
}


// 🔑 ЗАМЕНА: Сигнатура метода изменена на GY91Handler*
bool FlightStabilizer::begin(GY91Handler* imuHandler,
                             PCA9685ServoController* servoController) {
    ESP_LOGI(TAG, "=== 🚀 Инициализация FlightStabilizer ===");
    if (!imuHandler || !servoController) {
        ESP_LOGE(TAG, "❌ Null pointer в begin(): imu=%p servo=%p", imuHandler, servoController);
        return false;
    }
    _imuHandler = imuHandler;
    _servoController = servoController;

    if (!_imuHandler->isInitialized()) {
        ESP_LOGW(TAG, "⚠️ IMU не инициализирован — калибровка в процессе");
    }
    
    _initialized = true;
    _enabled = false;  // По умолчанию выключено, включим в setup()
    _lastUpdateMicros = micros();
    
    ESP_LOGI(TAG, "✅ FlightStabilizer готов | Режим: %s", modeToString(_mode));
    return true;
}

/**
 * @brief Задача стабилизации на ядре 1 (ESP32-S3)
 * @details Выполняет update() с фиксированным шагом 250 Гц (4000 мкс)
 */
void FlightStabilizer::stabilizationTask(void* parameter) {
    FlightStabilizer* self = static_cast<FlightStabilizer*>(parameter);
    
    while (true) {
        if (self->_enabled && self->_initialized) {
            uint32_t now = micros();
            int32_t elapsed = now - self->_lastUpdateMicros;
            
            // 🔑 Фиксированный шаг: 4000 мкс = 250 Гц
            if (elapsed >= 4000) {  
                self->_dt = 0.004f;
                self->_lastUpdateMicros = now;
                
                // 🔑 ИСПРАВЛЕНИЕ: Читаем актуальные команды из глобальных volatile переменных
                self->update(g_comUp, g_comLeft);
            }
        }
        // Отдаём управление планировщику
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void FlightStabilizer::startStabilizationTask() {
    if (!_initialized) {
        ESP_LOGE(TAG, "❌ Нельзя запустить задачу: не инициализирован");
        return;
    }

    ESP_LOGI(TAG, "🚀 Создание задачи стабилизации на CORE 1...");

    xTaskCreatePinnedToCore(
        stabilizationTask,    // Функция задачи
        "StabilizerTask",     // Имя
        8192,                 // Стек (8 KB для ESP_LOG внутри)
        this,                 // Параметр (this)
        2,                    // Приоритет (выше loop())
        &_taskHandle,         // Хендл
        1                     // 🔑 Ядро 1
    );

    if (_taskHandle) {
        ESP_LOGI(TAG, "✅ Задача запущена | Heap: %u B", ESP.getFreeHeap());
    } else {
        ESP_LOGE(TAG, "❌ Ошибка создания задачи!");
    }
}

// ... (остальные методы update, calculatePID, applyRollCorrection и т.д. остаются без изменений) ...

/* 
#include "modules/flight_stabilizer/FlightStabilizer.h"
#include <math.h>

// Тег для логирования
const char* FlightStabilizer::TAG = "FLIGHT_STAB";

FlightStabilizer::FlightStabilizer() {
    // Инициализация ПИД по умолчанию (значения требуют тюнинга!)
    _rollPID = {2.0f, 0.0f, 0.5f, 10.0f, 30.0f, 0.2f};
    _pitchPID = {2.0f, 0.0f, 0.5f, 10.0f, 30.0f, 0.2f};
    _yawPID = {1.5f, 0.0f, 0.3f, 10.0f, 45.0f, 0.2f};

    // Сохраняем базовые значения для адаптивности
    _baseRollPID = _rollPID;
    _basePitchPID = _pitchPID;

    resetPIDStates();

    ESP_LOGI(TAG, "✅ FlightStabilizer: конструктор завершён");
    ESP_LOGD(TAG, "   Roll PID: Kp=%.2f Ki=%.2f Kd=%.2f",
             _rollPID.kp, _rollPID.ki, _rollPID.kd);
}

FlightStabilizer::~FlightStabilizer() {
    if (_taskHandle) {
        vTaskDelete(_taskHandle);
        _taskHandle = nullptr;
    }
    ESP_LOGI(TAG, "🔄 Ресурсы FlightStabilizer освобождены");
}

// bool FlightStabilizer::begin(MPU9250Handler* imuHandler,
//                             PCA9685ServoController* servoController) {

bool FlightStabilizer::begin(GY91Handler* imuHandler, 
                            PCA9685ServoController* servoController) {
    ESP_LOGI(TAG, "=== 🚀 Инициализация FlightStabilizer ===");

    // 🔑 Проверка указателей
    if (!imuHandler || !servoController) {
        ESP_LOGE(TAG, "❌ Null pointer в begin(): imu=%p servo=%p",
                 imuHandler, servoController);
        return false;
    }

    _imuHandler = imuHandler;
    _servoController = servoController;

    // Проверка инициализации IMU
    if (!_imuHandler->isInitialized()) {
        ESP_LOGW(TAG, "⚠️ IMU не инициализирован — калибровка в процессе");
    }

    _initialized = true;
    _enabled = false;  // По умолчанию выключено
    _lastUpdateMicros = micros();

    ESP_LOGI(TAG, "✅ FlightStabilizer готов | Режим: %s",
             modeToString(_mode));
    return true;
}

//   /   *   *
// * @brief Задача стабилизации на ядре 1 (ESP32-S3)
//  * @details Выполняет update() с фиксированным шагом 250 Гц
//  *  /

void Flight_Stabilizer::stabilization_Task(void* parameter) {
    FlightStabilizer* self = static_cast<FlightStabilizer*>(parameter);

    // Локальные переменные для команд (обновляются из основного loop)
    // float comUp = 127.0f, comLeft = 127.0f;
    float comUp, comLeft;

    while (true) {
        if (self->_enabled && self->_initialized) {
            // 🔑 Фиксированный шаг: блокируем до следующего тика
            uint32_t now = micros();
            int32_t elapsed = now - self->_lastUpdateMicros;

            if (elapsed >= 4000) {  // 4000 мкс = 250 Гц
                self->_dt = 0.004f;
                self->_lastUpdateMicros = now;

                // 🔧 Получаем актуальные команды (через атомарные переменные или очередь)
                // В реальном проекте: xQueueReceive() из основного loop
                // self->update(comUp, comLeft);
                self->update(g_comUp, g_comLeft);
            }
        }

        // 🔑 Отдаём управление планировщику (минимальная задержка)
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void FlightStabilizer::startStabilizationTask() {
    if (!_initialized) {
        ESP_LOGE(TAG, "❌ Нельзя запустить задачу: не инициализирован");
        return;
    }

    ESP_LOGI(TAG, "🚀 Создание задачи стабилизации на CORE 1...");

    xTaskCreatePinnedToCore(
        stabilizationTask,    // Функция задачи
        "StabilizerTask",     // Имя
        8192,                 // Стек (8 KB для ESP_LOG внутри)
        this,                 // Параметр (this)
        2,                    // Приоритет (выше loop())
        &_taskHandle,         // Хендл
        1                     // 🔑 Ядро 1
    );

    if (_taskHandle) {
        ESP_LOGI(TAG, "✅ Задача запущена | Heap: %u B", ESP.getFreeHeap());
    } else {
        ESP_LOGE(TAG, "❌ Ошибка создания задачи!");
    }
}

*/

/**
 * @brief Основной метод обновления ПИД-логики
 * @note Вызывается из задачи стабилизации с фиксированным шагом
 */
bool FlightStabilizer::update(float comUp, float comLeft) {
    // 🔑 Быстрые проверки
    if (!_enabled || !_initialized || !_imuHandler || !_servoController) {
        return false;
    }

    // 🔑 Проверка валидности данных IMU
    if (!_imuHandler->isDataValid()) {
        _stats.invalidDataEvents++;
        if (_stats.invalidDataEvents % 100 == 0) {
            ESP_LOGW(TAG, "⚠️ Данные IMU недоступны (событий: %lu)",
                     _stats.invalidDataEvents);
        }
        return false;
    }

    // 🔹 Получение текущих углов
    const SensorData& imu = _imuHandler->getData();
    float currentRoll = imu.roll;
    float currentPitch = imu.pitch;
    float currentYaw = imu.yaw;

    // 🔹 Валидация углов (защита от выбросов)
    validateAngles(currentRoll, currentPitch, currentYaw);

    // 🔹 Преобразование команд пульта в целевые углы (-45°...+45°)
    _targetPitch = map(comUp, 0, 255, -45.0f, 45.0f);
    _targetRoll = map(comLeft, 0, 255, -45.0f, 45.0f);

    // 🔹 Расчёт ПИД в зависимости от режима
    switch (_mode) {
        case StabilizationMode::ROLL_PITCH:{
            // Крен
            float rollError = currentRoll - _targetRoll;
            float rollCorrection = calculatePID(_rollState, _rollPID, rollError, 0);
            applyRollCorrection(rollCorrection);

            // Тангаж
            float pitchError = currentPitch - _targetPitch;
            float pitchCorrection = calculatePID(_pitchState, _pitchPID, pitchError, 1);
            applyPitchCorrection(pitchCorrection);

            _stats.corrections += 2;
            break;
        }
        case StabilizationMode::FULL: {
            // Крен + Тангаж (как выше) + Рыскание
            float yawError = currentYaw - _targetYaw;
            float yawCorrection = calculatePID(_yawState, _yawPID, yawError, 2);
            applyYawCorrection(yawCorrection);

            _stats.corrections += 3;
            break;
        }
        case StabilizationMode::MANUAL:{ 
        }
        default:
            return false;  // Ручной режим — без стабилизации
        
    }

    _stats.stabilizations++;

    // 🔹 Периодическая отладка (каждые 50 итераций)
    #if CONFIG_LOG_DEFAULT_LEVEL >= ESP_LOG_DEBUG
    if (_stats.stabilizations % 50 == 0)
        ESP_LOGD(TAG, "STAB| R:%+5.1f°→%+5.1f°| P:%+5.1f°→%+5.1f°| Y:%5.1f°→%5.1f°",
                 currentRoll, _targetRoll, currentPitch, _targetPitch,
                 currentYaw, _targetYaw);
    #endif    
        //                if (_stats.stabilizations % 50 == 0 && ESP_LOG_LEVEL >= ESP_LOG_DEBUG) {
        //                    ESP_LOGD(TAG, "STAB| R:%+5.1f°→%+5.1f°| P:%+5.1f°→%+5.1f°| Y:%5.1f°→%5.1f°",
        //                            currentRoll, _targetRoll, currentPitch, _targetPitch,
        //                            currentYaw, _targetYaw);
        //                }

    return true;
}

/**
 * @brief Расчёт ПИД-регулятора с фильтром производной и Anti-Windup
 * @param axisIndex 0=Roll, 1=Pitch, 2=Yaw (для отладки и адаптивности)
 */
float FlightStabilizer::calculatePID(PIDState& state, const PIDConfig& config,
                                    float error, int axisIndex) {
    // 🔹 Пропорциональная составляющая
    float P = config.kp * error;

    // 🔹 Интегральная составляющая с Anti-Windup
    bool outputSaturated = false;
    float tempOutput = P + config.ki * state.integral +
                      config.kd * _prevDerivative[axisIndex];

    if (fabsf(tempOutput) >= config.maxOutput) {
        outputSaturated = true;
    }

    // Интегрируем ТОЛЬКО если выход не насыщен
    if (!outputSaturated) {
        state.integral += error * _dt;
    }
    state.integral = constrain(state.integral, -config.maxIntegral, config.maxIntegral);
    float I = config.ki * state.integral;

    // 🔹 Дифференциальная составляющая с НЧ-фильтром
    float D_raw = (error - state.lastError) / _dt;
    float D = config.kd * (
        config.dFilterAlpha * D_raw +
        (1.0f - config.dFilterAlpha) * _prevDerivative[axisIndex]
    );
    _prevDerivative[axisIndex] = D_raw;  // Сохраняем для следующего шага

    state.lastError = error;

    // 🔹 Формирование выхода
    float output = P + I + D;
    output = constrain(output, -config.maxOutput, config.maxOutput);

    // 🔹 Отладка компонентов ПИД (только при DEBUG_VERBOSE)
    #if CONFIG_LOG_DEFAULT_LEVEL_DEBUG
    if (ESP_LOG_LEVEL >= ESP_LOG_VERBOSE) {
        const char* axisName = (axisIndex == 0) ? "ROLL" :
                              (axisIndex == 1) ? "PITCH" : "YAW";
        ESP_LOGV(TAG, "[%s] P=%.2f I=%.2f D=%.2f | OUT=%.2f",
                 axisName, P, I, D, output);
    }
    #endif

    return output;
}

/**
 * @brief Применение коррекции крена (дифференциальное управление элеронами)
 */
void FlightStabilizer::applyRollCorrection(float correction) {
    float leftAileron = _targetRoll - correction;
    float rightAileron = _targetRoll + correction;

    // Ограничение диапазона серво
    leftAileron = constrain(leftAileron, -45.0f, 45.0f);
    rightAileron = constrain(rightAileron, -45.0f, 45.0f);

    // Установка углов (каналы 0 и 1 для элеронов)
    _servoController->setLogicalAngle(0, static_cast<int16_t>(leftAileron));
    _servoController->setLogicalAngle(1, static_cast<int16_t>(rightAileron));
}

/**
 * @brief Применение коррекции тангажа (синхронное управление рулями высоты)
 */
void FlightStabilizer::applyPitchCorrection(float correction) {
    float elevatorAngle = _targetPitch + correction;
    elevatorAngle = constrain(elevatorAngle, -45.0f, 45.0f);

    // Каналы 2 и 3 для рулей высоты
    _servoController->setLogicalAngle(2, static_cast<int16_t>(elevatorAngle));
    _servoController->setLogicalAngle(3, static_cast<int16_t>(elevatorAngle));
}

/**
 * @brief Применение коррекции рыскания (руль направления)
 */
void FlightStabilizer::applyYawCorrection(float correction) {
    float rudderAngle = _targetYaw + correction;
    rudderAngle = constrain(rudderAngle, -45.0f, 45.0f);

    // Канал 4 для руля направления
    _servoController->setLogicalAngle(4, static_cast<int16_t>(rudderAngle));
}

/**
 * @brief Адаптивное обновление ПИД-коэффициентов на основе вибраций
 * @param vibrationLevel Уровень вибраций [0.0...1.0] от VibrationEstimator
 */
void FlightStabilizer::updateAdaptiveParams(float vibrationLevel) {
    if (!_adaptiveEnabled) {
        // Восстанавливаем базовые значения
        _rollPID = _baseRollPID;
        _pitchPID = _basePitchPID;
        return;
    }

    // Нормализация [0.0...1.0]
    float vib = constrain(vibrationLevel, 0.0f, 1.0f);

    // 🔹 Алгоритм адаптивности:
    // Чем выше вибрация → тем МЕНЬШЕ Kp (меньше реакция на шум)
    // и МЕНЬШЕ Kd (производная усиливает высокочастотный шум)
    // Ki меняем минимально для сохранения удержания позиции

    float factor = 1.0f - (vib * 0.7f);  // При max вибрации оставляем 30% мощности

    _rollPID.kp = _baseRollPID.kp * factor;
    _rollPID.kd = _baseRollPID.kd * factor * 0.5f;  // Kd снижаем агрессивнее
    _rollPID.ki = _baseRollPID.ki * (1.0f - vib * 0.3f);  // Ki почти не меняем

    _pitchPID.kp = _basePitchPID.kp * factor;
    _pitchPID.kd = _basePitchPID.kd * factor * 0.5f;
    _pitchPID.ki = _basePitchPID.ki * (1.0f - vib * 0.3f);

    // 🔹 Отладка адаптивности
    #if CONFIG_LOG_DEFAULT_LEVEL_DEBUG
    if (ESP_LOG_LEVEL >= ESP_LOG_DEBUG && _stats.stabilizations % 100 == 0) {
        ESP_LOGD(TAG, "ADAPT| Vib=%.2f | Roll Kp=%.2f→%.2f Kd=%.2f→%.2f",
                 vib, _baseRollPID.kp, _rollPID.kp,
                 _baseRollPID.kd, _rollPID.kd);
    }
    #endif
}

//   ===*********************************************************
// 🔧 Остальные методы (validateAngles, printStatus, isStable, resetPIDStates, modeToString)
// реализуются аналогично исходному коду с добавлением ESP_LOG


/**
* @brief Валидация углов от IMU
*
* @param roll Крен (градусы) - передаётся по ссылке
* @param pitch Тангаж (градусы) - передаётся по ссылке
* @param yaw Рыскание (градусы) - передаётся по ссылке
*
* @note Защита от аномальных значений датчика
*/
void FlightStabilizer::validateAngles(float& roll, float& pitch, float& yaw) {
    // Проверка крена
    if (isnan(roll) || isinf(roll) || abs(roll) > 120.0f) {
        roll = 0.0f;
        ESP_LOGW(TAG, "⚠️ Аномальный крен: сброшено в 0°");
    }

    // Проверка тангажа
    if (isnan(pitch) || isinf(pitch) || abs(pitch) > 120.0f) {
        pitch = 0.0f;
        ESP_LOGW(TAG, "⚠️ Аномальный тангаж: сброшено в 0°");
    }

    // Проверка рыскания
    if (isnan(yaw) || isinf(yaw) || yaw < 0.0f || yaw > 360.0f) {
        yaw = 0.0f;
        ESP_LOGW(TAG, "⚠️ Аномальное рыскание: сброшено в 0°");
    }
}

// ============================================================================
// ДИАГНОСТИКА
// ============================================================================

/**
* @brief Вывод статуса системы в лог
*/
void FlightStabilizer::printStatus() const {
    ESP_LOGI(TAG, "=== СТАТУС СИСТЕМЫ СТАБИЛИЗАЦИИ ===");
    ESP_LOGI(TAG, "Режим: %s", modeToString(_mode));
    ESP_LOGI(TAG, "Состояние: %s", _enabled ? "ВКЛЮЧЕНА" : "ВЫКЛЮЧЕНА");
    ESP_LOGI(TAG, "Инициализирована: %s", _initialized ? "ДА" : "НЕТ");

    if (_imuHandler && !_imuHandler->isDataValid()) {
        ESP_LOGW(TAG, "⚠️ Данные гироскопа недоступны");
    }

    ESP_LOGI(TAG, "Статистика:");
    ESP_LOGI(TAG, "  Обновлений: %lu", _stats.stabilizations);
    ESP_LOGI(TAG, "  Коррекций: %lu", _stats.corrections);
    ESP_LOGI(TAG, "  Насыщений: %lu", _stats.saturationEvents);
    ESP_LOGI(TAG, "  Ошибок данных: %lu", _stats.invalidDataEvents);
    ESP_LOGI(TAG, "===================================");
}

/**
* @brief Преобразование режима стабилизации в строку
* @param mode Режим для преобразования
* @return Строковое описание режима
*/
const char* FlightStabilizer::modeToString(StabilizationMode mode) {
    switch (mode) {
        case StabilizationMode::MANUAL:      return "MANUAL (ручной)";
        case StabilizationMode::ROLL_PITCH:  return "ROLL_PITCH (автогоризонт)";
        case StabilizationMode::FULL:        return "FULL (полная)";
        case StabilizationMode::HOVER:       return "HOVER (удержание)";
        default:                             return "UNKNOWN";
    }
}

/**
* @brief Проверка стабильности положения
* @return true если положение стабильно
*
* @note Стабильным считается положение с малыми углами и угловыми скоростями
*/
bool FlightStabilizer::isStable() const {
    if (!_imuHandler || !_imuHandler->isDataValid()) {
        return false;
    }

    const SensorData& data = _imuHandler->getData();

    // Проверка углов и угловых скоростей
    bool anglesStable = (abs(data.roll) < 5.0f && abs(data.pitch) < 5.0f);
    bool ratesStable = (abs(data.gyro.x) < 0.17f && abs(data.gyro.y) < 0.17f); // ~10°/с в рад/с

    return anglesStable && ratesStable;
}

void FlightStabilizer::resetPIDStates() {
    // Полное обнуление состояний PID-регуляторов
    _rollState  = {};
    _pitchState = {};
    _yawState   = {};
    
    // Сброс фильтров производной (anti-derivative kick)
    _prevDerivative[0] = 0.0f;
    _prevDerivative[1] = 0.0f;
    _prevDerivative[2] = 0.0f;
    
    ESP_LOGD(TAG, "🔄 PID states reset");
}

        //            // Реализация адаптивности
        //            void FlightStabilizer::updateAdaptiveParams(float vibrationLevel) {
        //                if (!_adaptivePIDEnabled) {
        //                    // Если адаптивность выключена — возвращаем базовые
        //                    _rollPID = _baseRollPID;
        //                    _pitchPID = _basePitchPID;
        //                    return;
        //                }
//
        //                // Нормализуем вибрацию [0.0 ... 1.0]
        //                float vib = constrain(vibrationLevel, 0.0f, 1.0f);
//
        //                // 📐 АЛГОРИТМ:
        //                // Чем выше вибрация, тем МЕНЬШЕ Kp (чтобы сервы не дергались от шума)
        //                // и МЕНЬШЕ Kd (производная усиливает высокочастотный шум).
        //                // Ki оставляем почти без изменений для удержания позиции.
//
        //                // Формула: New = Base * (1.0 - vib * dampingFactor)
        //                // dampingFactor = 0.7 (при макс вибрации оставляем 30% мощности)
//
        //                float factor = 1.0f - (vib * 0.7f);
//
        //                _rollPID.kp = _baseRollPID.kp * factor;
        //                _rollPID.kd = _baseRollPID.kd * factor * 0.5f; // Kd снижаем агрессивнее
//
        //                _pitchPID.kp = _basePitchPID.kp * factor;
        //                _pitchPID.kd = _basePitchPID.kd * factor * 0.5f;
//
        //                // Ki (интегратор) снижаем незначительно, чтобы не потерять ориентацию
        //                _rollPID.ki = _baseRollPID.ki * (1.0f - vib * 0.3f);
        //                _pitchPID.ki = _basePitchPID.ki * (1.0f - vib * 0.3f);
        //            }

// #endif // FLIGHT_STABILIZER_H
