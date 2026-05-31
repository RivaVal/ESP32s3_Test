

/**
 * @file flight_stabilizer.h
 * @brief Система ПИД-стабилизации полёта БПЛА
 * @version 2.0.0 (ESP32-S3 адаптация)
 * @details
 * - Фиксированный шаг дискретизации 250 Гц (4000 мкс)
 * - Выполнение на ядре 1 для разгрузки ядра 0 (LoRa/I2C/UART)
 * - Адаптивные ПИД-коэффициенты на основе оценки вибраций
 * - Полная документация и отладка через ESP_LOG
 */
#pragma once
#ifndef FLIGHT_STABILIZER_H
#define FLIGHT_STABILIZER_H

#include <Arduino.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Локальные заголовки проекта
#include "common/types.h"
#include "config/pins.h"
// #include "modules/imu_handler/mpu9250_handler.h"
#include "modules/imu_handler/MPU9250_Handler.h"  // ← Обратите внимание на регистр!
#include "modules/pca9685_servo/pca9685_servo.h"



/**
 * @brief Конфигурация ПИД-регулятора с адаптивными параметрами
 */
struct PIDConfig {
    float kp = 2.0f;
    float ki = 0.0f;
    float kd = 0.5f;
    float maxIntegral = 10.0f;
    float maxOutput = 30.0f;
    float dFilterAlpha = 0.2f;

    // ✅ ДОБАВИТЬ конструктор по умолчанию:
    PIDConfig() = default;

    // Конструктор с параметрами (опционально)
    PIDConfig(float p, float i, float d, float mi, float mo, float df)
        : kp(p), ki(i), kd(d), maxIntegral(mi), maxOutput(mo), dFilterAlpha(df) {}
};

        //        struct PIDConfig {
        //            float kp = 2.0f;              ///< Пропорциональный коэффициент
        //            float ki = 0.0f;              ///< Интегральный коэффициент
        //            float kd = 0.5f;              ///< Дифференциальный коэффициент
         ///           float maxIntegral = 10.0f;    ///< Ограничение интеграла (anti-windup)
        //            float maxOutput = 30.0f;      ///< Макс. выход (градусы)
        //            float dFilterAlpha = 0.2f;    ///< Коэф. фильтра производной (0.0-1.0)
//
        //            // 🔑 Адаптивные параметры (заполняются автоматически)
        //            float baseKp = 2.0f;          ///< Базовое Kp для восстановления
        //            float baseKi = 0.0f;          ///< Базовое Ki
        //            float baseKd = 0.5f;          ///< Базовое Kd

        //            PIDConfig(float p, float i, float d, float mi, float mo, float df) 
        //                : kp(p), ki(i), kd(d), maxIntegral(mi), maxOutput(mo), dFilterAlpha(df) {}

        //        };

        //        struct PIDConfig {
        //            float kp = 2.0f, ki = 0.0f, kd = 0.5f;
        //            float maxIntegral = 10.0f, maxOutput = 30.0f, dFilterAlpha = 0.2f;
                    
        //            // 🔧 Добавьте конструктор:
        //            PIDConfig(float p, float i, float d, float mi, float mo, float df) 
        //                : kp(p), ki(i), kd(d), maxIntegral(mi), maxOutput(mo), dFilterAlpha(df) {}
        //        };
/**
 * @brief Состояние ПИД-регулятора
 */
struct PIDState {
    float error = 0.0f;
    float integral = 0.0f;
    float derivative = 0.0f;
    float lastError = 0.0f;
    float output = 0.0f;
};

/**
 * @brief Режимы стабилизации
 */
enum class StabilizationMode : uint8_t {
    MANUAL      = 0,    ///< Ручное управление (без стабилизации)
    ROLL_PITCH  = 1,    ///< Автогоризонт (крен + тангаж)
    FULL        = 2,    ///< Полная стабилизация (вкл. рыскание)
    HOVER       = 3     ///< Удержание позиции (требует GPS/барометр)
};

/**
 * @class FlightStabilizer
 * @brief Основной класс системы стабилизации полёта
 *
 * @note Архитектура:
 * - Все вычисления выполняются на ядре 1 (ESP32-S3)
 * - Фиксированный шаг 250 Гц устраняет джиттер ПИД
 * - Адаптивность снижает Kp/Kd при вибрациях для стабильности
 */
class FlightStabilizer {
public:
    FlightStabilizer();
    ~FlightStabilizer();

    /**
     * @brief Инициализация системы
     * @param imuHandler Указатель на обработчик IMU
     * @param servoController Указатель на контроллер серво
     * @return true при успехе
     */
    bool begin(MPU9250Handler* imuHandler, PCA9685ServoController* servoController);

    /**
     * @brief Запуск задачи стабилизации на ядре 1
     * @note Вызывать после begin() в setup()
     */
    void startStabilizationTask();

    /**
     * @brief Основной метод обновления (вызывается из задачи)
     * @param comUp Команда тангажа 0-255
     * @param comLeft Команда крена 0-255
     * @return true если стабилизация активна
     */
    bool update(float comUp, float comLeft);

    // Управление
    void enable();
    void disable();
    void setMode(StabilizationMode mode);
    StabilizationMode getMode() const { return _mode; }
    bool isEnabled() const { return _enabled; }

    // Адаптивность
    void setAdaptivePID(bool enable) { _adaptiveEnabled = enable; }
    bool isAdaptivePIDEnabled() const { return _adaptiveEnabled; }
    void updateAdaptiveParams(float vibrationLevel);

    // Диагностика
    void printStatus() const;
    bool isStable() const;
    void resetPIDStates();

    // Геттеры конфигурации (для тюнинга)
    PIDConfig getRollPID() const { return _rollPID; }
    PIDConfig getPitchPID() const { return _pitchPID; }
    PIDConfig getYawPID() const { return _yawPID; }

    // 🔧 Статический метод для преобразования режима в строку
    static const char* modeToString(StabilizationMode mode);  

    // static const char* TAG;

private:
    // Указатели на внешние объекты
    MPU9250Handler* _imuHandler = nullptr;
    PCA9685ServoController* _servoController = nullptr;

    // Флаги состояния
    bool _initialized = false;
    bool _enabled = false;
    bool _adaptiveEnabled = false;
    StabilizationMode _mode = StabilizationMode::MANUAL;

    // Временные параметры
    float _dt = 0.004f;  // Фиксированный шаг: 4000 мкс = 250 Гц
    uint32_t _lastUpdateMicros = 0;

    // Целевые углы (из команд пульта)
    float _targetRoll = 0.0f;
    float _targetPitch = 0.0f;
    float _targetYaw = 0.0f;

    // ПИД-конфигурации и состояния по осям
    PIDConfig _rollPID, _pitchPID, _yawPID;
    PIDState _rollState, _pitchState, _yawState;

    // 🔑 Фильтр производной (отдельно по осям)
    float _prevDerivative[3] = {0.0f, 0.0f, 0.0f};  // Roll, Pitch, Yaw

    // Базовые коэффициенты для адаптивности
    PIDConfig _baseRollPID, _basePitchPID;

    // Статистика
    struct {
        uint32_t stabilizations = 0;
        uint32_t corrections = 0;
        uint32_t saturationEvents = 0;
        uint32_t invalidDataEvents = 0;
    } _stats;

    // 🔧 Приватные методы
    float calculatePID(PIDState& state, const PIDConfig& config,
                      float error, int axisIndex);
    void applyRollCorrection(float correction);
    void applyPitchCorrection(float correction);
    void applyYawCorrection(float correction);
    void validateAngles(float& roll, float& pitch, float& yaw);

    // 🔧 Задача FreeRTOS для ядра 1
    static void stabilizationTask(void* parameter);
    TaskHandle_t _taskHandle = nullptr;

    // Тег для ESP_LOG
    static const char* TAG;
};

#endif // FLIGHT_STABILIZER_H
