
/**
 * @file main.Battery.cpp
 * @brief Точка входа для UAV_ESP32S3_Core (ШАГ 6: Интеграция BatteryMonitor)
 * @details
 * - Инициализация: USB CDC, PSRAM, LoRa, I2C, PCA9685, LEDC, BatteryMonitor
 * - Предстартовые тесты серво, моторов и мониторинга АКБ
 * - Основной цикл: приём команд → управление → телеметрия + мониторинг батареи
 *
 * @environment ESP32-S3-N16R8 | PlatformIO | Arduino Framework | ESP-IDF 5.0+
 * @version 1.3 (ШАГ 6: Интеграция BatteryMonitor)
 * @date 2026-05-28
 */
#include <Arduino.h>
#include <esp_log.h>

// ============================================================================
// Подключения модулей
// ============================================================================
#include "modules/lora_communicator/lora_communicator.h"
#include "modules/i2c_master/i2c_master.h"
#include "modules/pca9685_servo/pca9685_servo.h"
#include "modules/unified_ledc/unified_ledc.h"
#include "modules/battery_monitor/battery_monitor.h"  // 🆕 ШАГ 6
#include "config/pins.h"
#include "common/types.h"

static const char* TAG = "MAIN";

// ============================================================================
// Глобальные объекты модулей
// ============================================================================
static I2CMasterController i2c_manager;
static PCA9685ServoController servo_controller;
static UnifiedLEDCController motor_controller;
static BatteryMonitor battery_monitor;  // 🆕 ШАГ 6

// ============================================================================
// setup(): Инициализация системы
// ============================================================================
void setup() {
    Serial.begin(115200);
    uint32_t start = millis();
    while (!Serial && millis() - start < 3000) { delay(10); yield(); }

    esp_log_level_set("*", static_cast<esp_log_level_t>(CORE_DEBUG_LEVEL));
    ESP_LOGI(TAG, "🚀 UAV_ESP32S3_Core STARTUP | Core: %s Rev%d", ESP.getChipModel(), ESP.getChipRevision());
    ESP_LOGI(TAG, "💾 Heap: %u B free | PSRAM: %u B", ESP.getFreeHeap(), ESP.getPsramSize());

    // 1️⃣ LoRa (ШАГ 2)
    ESP_LOGI(TAG, "📡 Initializing LoRa communicator...");
    if (!lora_init()) {
        ESP_LOGE(TAG, "❌ CRITICAL: LoRa init failed!");
        pinMode(LED_BUILTIN, OUTPUT);
        while (true) { digitalWrite(LED_BUILTIN, HIGH); delay(100); digitalWrite(LED_BUILTIN, LOW); delay(400); }
    }
    ESP_LOGI(TAG, "✅ LoRa module ready");

    // 2️⃣ I2C + PCA9685 (ШАГ 4)
    ESP_LOGI(TAG, "🔌 Initializing I2C Master...");
    if (i2c_manager.begin()) {
        i2c_manager.scanDevices();
        ESP_LOGI(TAG, "🤖 Initializing PCA9685...");
        if (servo_controller.begin(i2c_manager)) {
            ESP_LOGI(TAG, "🧪 Running servo test...");
            if (!servo_controller.runServoTest()) {
                ESP_LOGE(TAG, "❌ Servo test FAILED!");
                while (true) { digitalWrite(LED_BUILTIN, HIGH); delay(100); digitalWrite(LED_BUILTIN, LOW); delay(100); }
            }
        } else {
            ESP_LOGW(TAG, "⚠️ PCA9685 not found. Servo control disabled.");
        }
    } else {
        ESP_LOGW(TAG, "⚠️ I2C failed. Servo control disabled.");
    }

    // 3️⃣ LEDC (ШАГ 5)
    ESP_LOGI(TAG, "⚡ Initializing LEDC Motor Controller...");
    if (!motor_controller.begin()) {
        ESP_LOGE(TAG, "❌ CRITICAL: LEDC init failed! Error: %s", motor_controller.getLastError());
        while (true) { digitalWrite(LED_BUILTIN, HIGH); delay(200); digitalWrite(LED_BUILTIN, LOW); delay(50); }
    }
    ESP_LOGI(TAG, "🧪 Running motor diagnostic (SAFE MODE)...");
    if (!motor_controller.runStartupDiagnostic(25.0f, 1500)) {
        ESP_LOGE(TAG, "❌ Motor diagnostic FAILED!");
        while (true) { digitalWrite(LED_BUILTIN, HIGH); delay(200); digitalWrite(LED_BUILTIN, LOW); delay(50); }
    }

    // 4️⃣ 🆕 BatteryMonitor (ШАГ 6)
    ESP_LOGI(TAG, "🔋 Initializing Battery Monitor...");
    if (!battery_monitor.begin(Config::Battery::DEFAULT_CELL_COUNT)) {
        ESP_LOGE(TAG, "❌ CRITICAL: BatteryMonitor init failed!");
        while (true) { digitalWrite(LED_BUILTIN, HIGH); delay(100); digitalWrite(LED_BUILTIN, LOW); delay(300); }
    }
    ESP_LOGI(TAG, "✅ BatteryMonitor ready | Config: %s", battery_monitor.getStatus().configName);
    battery_monitor.printStatus(false);  // Первичный вывод статуса

    // 🔒 Проверка напряжения перед запуском
    if (battery_monitor.isCritical()) {
        ESP_LOGE(TAG, "🔴 КРИТИЧЕСКИ НИЗКОЕ напряжение! Запуск запрещён!");
        while (true) { digitalWrite(LED_BUILTIN, HIGH); delay(50); digitalWrite(LED_BUILTIN, LOW); delay(450); }
    } else if (battery_monitor.isLow()) {
        ESP_LOGW(TAG, "⚠️  Низкое напряжение батареи. Будьте осторожны!");
    }

    ESP_LOGI(TAG, "🟢 Setup complete. Entering main loop...");
}

// ============================================================================
// loop(): Основной цикл
// ============================================================================
void loop() {
    static uint32_t last_stats_print = 0;
    static uint32_t tick = 0;

    // 🔹 1. Обработка LoRa
    lora_loop();
    if (lora_packet_available()) {
        DataComSet_t packet;
        size_t len = lora_read_packet(&packet);
        if (len > 0) {
            ESP_LOGI(TAG, "📥 Command received | ID:%u | Thr:%u", packet.packet_id, packet.comThrottle);

            // Серво (ШАГ 4)
            if (servo_controller.isInitialized()) {
                servo_controller.processFlightCommands(packet.comUp, packet.comLeft);
                if (packet.comSetAll & PARASHUT_FLAG) {
                    ESP_LOGI(TAG, "🪂 Parachute activated!");
                    servo_controller.setPhysicalAngle(4, 180);
                }
            }

            // Моторы (ШАГ 5)
            if (motor_controller.isInitialized()) {
                float throttlePct = map(packet.comThrottle, 1000, 2000, 0, 100);
                throttlePct = constrain(throttlePct, 0.0f, 100.0f);
                motor_controller.setMotorPower(0, throttlePct);
                motor_controller.setMotorPower(1, throttlePct);
            }
        }
    }

    // 🔹 2. 🆕 Мониторинг батареи (ШАГ 6)
    if (battery_monitor.isInitialized()) {
        battery_monitor.update();  // Неблокирующая периодическая проверка

        // Аварийное действие при критическом разряде
        if (battery_monitor.isCritical()) {
            ESP_LOGE(TAG, "🔴 BATTERY CRITICAL! Emergency stop!");
            motor_controller.stopAllMotors();
            if (servo_controller.isInitialized()) {
                servo_controller.resetToNeutral();  // Безопасное положение серво
            }
        }
    }

    // 🔹 3. Статистика
    if (millis() - last_stats_print >= 10000) {
        LoRaStats_t lora_stats = lora_get_stats();
        ESP_LOGI(TAG, "📊 Stats | LoRa RX: %lu/%lu | CRC: %lu | ACK: %lu/%lu | RSSI: %d dBm",
                 lora_stats.packets_success, lora_stats.packets_received, lora_stats.crc_errors,
                 lora_stats.acks_success, lora_stats.acks_sent, lora_stats.last_rssi);

        ESP_LOGI(TAG, "🤖 Servo: %s | ⚡ Motors: %s | 🔋 Battery: %.2fV (%.1f%%)",
                 servo_controller.isInitialized() ? "OK" : "OFF",
                 motor_controller.isInitialized() ? "OK" : "OFF",
                 battery_monitor.getStatus().voltage,
                 battery_monitor.getStatus().percentage);

        last_stats_print = millis();
    }

    // 🔹 4. Heartbeat & RTOS yield
    ESP_LOGD(TAG, "⏱️ Tick #%lu | Heap: %u B", tick++, ESP.getFreeHeap());
    delay(100);
}
