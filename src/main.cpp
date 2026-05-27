
/**
 * @file main.cpp
 * @brief Точка входа проекта UAV_ESP32S3_Core (ШАГ 1: LoRa модуль)
 * @details 
 * - Инициализация USB CDC, PSRAM, LoRa
 * - Демонстрация приёма пакетов и отправки ACK
 * - Готово для последовательного добавления других модулей
 */
/**
 * @file main.cpp
 * @brief Точка входа для проекта UAV_ESP32S3_Core (ШАГ 4: Интеграция PCA9685)
 * @details
 * - Инициализация: USB CDC, PSRAM, LoRa, I2C, PCA9685
 * - Предстартовый тест сервоприводов
 * - Основной цикл с обработкой команд и отправкой телеметрии
 *
 * @environment ESP32-S3-N16R8 | PlatformIO | Arduino Framework
 * @version 1.1 (ШАГ 4: Интеграция PCA9685)
 * @date 2026-05-26
 */

#include <Arduino.h>
#include <esp_log.h>

// ============================================================================
// Подключения модулей
// ============================================================================
// ✅ ШАГ 2: Модуль связи (уже работает)
#include "modules/lora_communicator/lora_communicator.h"

// 🆕 ШАГ 4: Новые модули
#include "modules/i2c_master/i2c_master.h"
#include "modules/pca9685_servo/pca9685_servo.h"

#include "config/pins.h"
#include "common/types.h"

// ============================================================================
// Конфигурация отладки
// ============================================================================
static const char* TAG = "MAIN";

// ============================================================================
// Глобальные объекты модулей
// ============================================================================
static I2CMasterController i2c_manager;                    ///< Контроллер I2C
static PCA9685ServoController servo_controller;            ///< Контроллер серво

// ============================================================================
// setup(): Инициализация системы
// ============================================================================
void setup() {
    // ========================================================================
    // 1. Инициализация Native USB CDC (ESP32-S3)
    // ========================================================================
    Serial.begin(115200);

    // 🔑 Таймаут для USB CDC (критично для ESP32-S3)
    uint32_t start = millis();
    while (!Serial && millis() - start < 3000) {
        delay(10);
        yield();
    }
    delay(500);

    Serial.println("=== 🚀 UAV_ESP32S3_Core STARTUP ===");
    Serial.println("Chip: " + String(ESP.getChipModel()) + " Rev" + String(ESP.getChipRevision()));
    Serial.println("Heap: " + String(ESP.getFreeHeap()) + " B free");
    Serial.flush();

    // Настройка уровня логирования
    esp_log_level_set("*", static_cast<esp_log_level_t>(CORE_DEBUG_LEVEL));
    ESP_LOGI(TAG, "🚀 System startup | Core: %s Rev%d",
             ESP.getChipModel(), ESP.getChipRevision());

    // ========================================================================
    // 2. Диагностика памяти
    // ========================================================================
    ESP_LOGI(TAG, "💾 Heap: %u B free | PSRAM: %u B",
             ESP.getFreeHeap(), ESP.getPsramSize());

    if (ESP.getPsramSize() == 0) {
        ESP_LOGW(TAG, "⚠️ PSRAM not detected! Check board settings");
    }

    // ========================================================================
    // 3. ✅ ШАГ 2: Инициализация модуля связи (уже работает)
    // ========================================================================
    ESP_LOGI(TAG, "📡 Initializing LoRa communicator...");
    if (!lora_init()) {
        ESP_LOGE(TAG, "❌ CRITICAL: LoRa initialization failed!");
        pinMode(LED_BUILTIN, OUTPUT);
        while (true) {
            digitalWrite(LED_BUILTIN, HIGH); delay(100);
            digitalWrite(LED_BUILTIN, LOW);  delay(400);
        }
    }
    ESP_LOGI(TAG, "✅ LoRa module ready");

    // ========================================================================
    // 4. 🆕 ШАГ 4: Инициализация I2C + PCA9685
    // ========================================================================
    ESP_LOGI(TAG, "🔌 Initializing I2C Master...");
    if (!i2c_manager.begin()) {
        ESP_LOGE(TAG, "❌ CRITICAL: I2C initialization failed!");
        // Не останавливаем систему — серво опциональны
    } else {
        // Сканирование шины для отладки
        i2c_manager.scanDevices();

        // Инициализация контроллера серво
        ESP_LOGI(TAG, "🤖 Initializing PCA9685 Servo Controller...");
        if (!servo_controller.begin(i2c_manager)) {
            ESP_LOGE(TAG, "❌ CRITICAL: PCA9685 initialization failed!");
            // Не останавливаем систему — серво опциональны
        } else {
            ESP_LOGI(TAG, "✅ PCA9685 ready | 7 servos on channels 0-6");

            // 🔑 ПРЕДСТАРТОВЫЙ ТЕСТ СЕРВОПРИВОДОВ
            ESP_LOGI(TAG, "🧪 Running servo startup test...");
            if (!servo_controller.runServoTest()) {
                ESP_LOGE(TAG, "❌ Servo test FAILED!");
                // Индикация ошибки
                pinMode(LED_BUILTIN, OUTPUT);
                while (true) {
                    digitalWrite(LED_BUILTIN, HIGH); delay(100);
                    digitalWrite(LED_BUILTIN, LOW);  delay(100);
                }
            }
            ESP_LOGI(TAG, "✅ Servo test completed successfully");
        }
    }

    // ========================================================================
    // 5. Финальный статус
    // ========================================================================
    ESP_LOGI(TAG, "🟢 Setup complete. Entering main loop...");
    Serial.println("✅ System ready. Entering loop...");
    Serial.flush();
}

// ============================================================================
// loop(): Основной цикл
// ============================================================================
void loop() {
    static uint32_t last_stats_print = 0;
    static uint32_t tick = 0;

    // ========================================================================
    // 🔹 1. Обработка LoRa-событий (ШАГ 2 — уже работает)
    // ========================================================================
    lora_loop();

    if (lora_packet_available()) {
        DataComSet_t packet;
        size_t len = lora_read_packet(&packet);

        if (len > 0) {
            ESP_LOGI(TAG, "📥 Command received | ID: %u | Up: %d | Left: %d",
                     packet.packet_id, packet.comUp, packet.comLeft);

            // ====================================================================
            // 🔹 Обработка команд сервоприводам (ШАГ 4)
            // ====================================================================
            if (servo_controller.isInitialized()) {
                servo_controller.processFlightCommands(packet.comUp, packet.comLeft);

                // Обработка флага парашюта
                if (packet.comSetAll & PARASHUT_FLAG) {
                    ESP_LOGI(TAG, "🪂 Parachute flag detected!");
                    servo_controller.setPhysicalAngle(4, 180);  // Канал 4 = парашют
                }
            }
        }
    }

    // ========================================================================
    // 🔹 2. Периодическая печать статистики (каждые 10 сек)
    // ========================================================================
    if (millis() - last_stats_print >= 10000) {
        LoRaStats_t stats = lora_get_stats();
        ESP_LOGI(TAG, "📊 Stats | RX: %lu/%lu | CRC: %lu | ACK: %lu/%lu | RSSI: %d dBm",
                 stats.packets_success, stats.packets_received,
                 stats.crc_errors,
                 stats.acks_success, stats.acks_sent,
                 stats.last_rssi);

        // Статус серво
        if (servo_controller.isInitialized()) {
            ESP_LOGI(TAG, "🤖 Servos: PCA9685 @ 0x%02X | Connected: %s",
                     servo_controller.getI2CAddress(),
                     servo_controller.isChipConnected() ? "YES" : "NO");
        }

        last_stats_print = millis();
    }

    // ========================================================================
    // 🔹 3. Heartbeat (каждую секунду)
    // ========================================================================
    ESP_LOGD(TAG, "⏱️ Tick #%lu | Heap: %u B", tick++, ESP.getFreeHeap());

    // 🔹 4. Отдаём управление RTOS (критично для ESP-IDF)
    delay(100);  // Автоматически вызывает yield()
}
