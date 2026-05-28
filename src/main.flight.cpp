

/**
 * @file main.cpp
 * @brief Точка входа проекта (ШАГ 7: Интеграция FlightStabilizer)
 * @version 1.7.0
 */
#include <Arduino.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>

// 🔹 Существующие модули (сохраняем!)
#include "modules/lora_communicator/lora_communicator.h"
#include "modules/i2c_master/i2c_master.h"
#include "modules/pca9685_servo/pca9685_servo.h"
#include "modules/ledc_motor/ledc_motor.h"
#include "modules/battery_monitor/battery_monitor.h"

// 🔹 НОВЫЕ модули ШАГА 7
#include "modules/flight_stabilizer/flight_stabilizer.h"
#include "modules/imu_handler/mpu9250_handler.h"
#include "modules/filter_manager/filter_manager.h"
#include "modules/telemetry_uart/telemetry_uart_bridge.h"

// Конфигурация
#include "config/pins.h"
#include "common/types.h"

static const char* TAG = "MAIN";

// 🔹 Глобальные объекты
static I2CMasterController i2c_manager;
static PCA9685ServoController servo_controller;
static LEDCMotorController motor_controller;
static BatteryMonitor battery_monitor;
static MPU9250Handler imu_handler;
static FilterManager filter_manager;
static FlightStabilizer flight_stabilizer;
static TelemetryUARTBridge telemetry_bridge;

// 🔹 Переменные для передачи команд в задачу стабилизации
static volatile float g_comUp = 127.0f;
static volatile float g_comLeft = 127.0f;
static portMUX_TYPE command_mux = portMUX_INITIALIZER_UNLOCKED;

void setup() {
    // 🔹 1. USB CDC + базовая инициализация
    Serial.begin(115200);
    uint32_t start = millis();
    while (!Serial && millis() - start < 3000) { delay(10); yield(); }
    delay(500);

    esp_log_level_set("*", ESP_LOG_INFO);
    ESP_LOGI(TAG, "🚀 UAV_ESP32S3_Core v1.7.0 | Heap: %u B", ESP.getFreeHeap());

    // 🔹 2. PSRAM диагностика
    if (ESP.getPsramSize() > 0) {
        ESP_LOGI(TAG, "✅ PSRAM: %u B available", ESP.getPsramSize());
    }

    // 🔹 3. Инициализация зависимостей (в порядке зависимости)

    // 3.1. I2C (база для IMU и PCA9685)
    if (!i2c_manager.begin()) {
        ESP_LOGE(TAG, "❌ I2C initialization failed");
    }

    // 3.2. IMU (требует I2C)
    if (!imu_handler.begin(i2c_manager)) {
        ESP_LOGE(TAG, "❌ IMU initialization failed");
    } else {
        ESP_LOGI(TAG, "✅ IMU ready | Calibrating gyro...");
        delay(2000);  // Калибровка гироскопа
    }

    // 3.3. PCA9685 (требует I2C)
    if (!servo_controller.begin(i2c_manager)) {
        ESP_LOGE(TAG, "❌ PCA9685 initialization failed");
    }

    // 3.4. LEDC моторы
    if (!motor_controller.begin()) {
        ESP_LOGE(TAG, "❌ LEDC motor initialization failed");
    }

    // 3.5. BatteryMonitor
    if (!battery_monitor.begin(4)) {  // 4S LiPo
        ESP_LOGW(TAG, "⚠️ Battery monitor not initialized");
    }

    // 3.6. FilterManager (для FlightStabilizer)
    filter_manager.begin(250.0f);  // 250 Гц
    filter_manager.enableAdaptive(true);  // 🔑 Включаем адаптивность

    // 3.7. FlightStabilizer (требует IMU и PCA9685)
    if (flight_stabilizer.begin(&imu_handler, &servo_controller)) {
        flight_stabilizer.setMode(StabilizationMode::ROLL_PITCH);
        flight_stabilizer.setAdaptivePID(true);  // 🔑 Адаптивные ПИД
        flight_stabilizer.startStabilizationTask();  // 🔑 Запуск на ядре 1
        ESP_LOGI(TAG, "✅ FlightStabilizer active");
    }

    // 3.8. LoRa (уже работает с Шага 2)
    if (!lora_init()) {
        ESP_LOGE(TAG, "❌ CRITICAL: LoRa failed");
        // Индикация ошибки...
    }

    // 3.9. TelemetryUARTBridge (UART2 → RPi)
    if (!telemetry_bridge.begin(921600,
                               Config::Pins::UART_TX,
                               Config::Pins::UART_RX,
                               Config::Pins::UART_RTS,
                               Config::Pins::UART_CTS)) {
        ESP_LOGW(TAG, "⚠️ Telemetry bridge not initialized");
    }

    // 🔹 4. Предстартовые тесты
    ESP_LOGI(TAG, "🧪 Running startup tests...");
    if (servo_controller.isInitialized()) {
        servo_controller.runServoTest();  // Безопасный тест ±30°
    }

    ESP_LOGI(TAG, "🟢 Setup complete. Entering loop...");
}

void loop() {
    static uint32_t last_telem = 0;
    static uint32_t last_stats = 0;

    // 🔹 1. Обработка LoRa-команд
    if (lora_packet_available()) {
        DataComSet_t packet;
        if (lora_read_packet(&packet) > 0) {
            // 🔑 Атомарное обновление команд для задачи стабилизации
            portENTER_CRITICAL(&command_mux);
            g_comUp = static_cast<float>(packet.comUp);
            g_comLeft = static_cast<float>(packet.comLeft);
            portEXIT_CRITICAL(&command_mux);

            // Обработка флагов (парашют и т.д.)
            if (packet.comSetAll & PARASHUT_FLAG) {
                ESP_LOGI(TAG, "🪂 Parachute command received");
                servo_controller.setPhysicalAngle(4, 180);
            }
        }
    }

    // 🔹 2. Обновление IMU и фильтра (на ядре 0)
    if (imu_handler.updateSensors()) {
        const SensorData& imu = imu_handler.getData();

        // 🔹 Обновление фильтра ориентации
        filter_manager.update(0.004f,  // dt = 4 мс
                             imu.accel.x, imu.accel.y, imu.accel.z,
                             imu.gyro.x, imu.gyro.y, imu.gyro.z,
                             imu.mag.x, imu.mag.y, imu.mag.z);

        // 🔹 Обновление адаптивных параметров ПИД
        float vibLevel = filter_manager.getCurrentVibrationLevel();
        flight_stabilizer.updateAdaptiveParams(vibLevel);
    }

    // 🔹 3. Телеметрия на RPi (каждые 20-50 мс)
    if (millis() - last_telem >= 30 && telemetry_bridge.isInitialized()) {
        TelemetryPacket_t pkt = {};

        // Заполнение пакета (упрощённо)
        pkt.roll = imu_handler.getData().roll;
        pkt.pitch = imu_handler.getData().pitch;
        pkt.yaw = imu_handler.getData().yaw;
        pkt.battery_voltage = battery_monitor.getStatus().voltage;
        // ... остальные поля

        if (!telemetry_bridge.sendTelemetry(pkt)) {
            ESP_LOGV("TELEM", "⏳ Telemetry buffer busy");
        }
        last_telem = millis();
    }

    // 🔹 4. Периодическая статистика
    if (millis() - last_stats >= 5000) {
        flight_stabilizer.printStatus();
        battery_monitor.printStatus(false);
        last_stats = millis();
    }

    // 🔹 5. Отдаём управление (критично для ESP-IDF)
    delay(10);  // Автоматически вызывает yield()
}
