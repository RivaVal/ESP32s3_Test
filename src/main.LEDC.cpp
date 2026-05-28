
### 📁 3. Обновлённый `main.cpp`
*Интегрирует LEDC в существующую архитектуру. Сохраняет LoRa, I2C, PCA9685. Добавляет обработку тяги (`comThrottle`) и диагностику.*

```cpp
/**
 * @file main.cpp
 * @brief Точка входа для UAV_ESP32S3_Core (ШАГ 5: Интеграция LEDC для моторов)
 * @details
 * - Инициализация: USB CDC, PSRAM, LoRa, I2C, PCA9685, LEDC
 * - Предстартовые тесты серво и моторов
 * - Основной цикл: приём команд LoRa → управление серво + моторы → отправка телеметрии
 *
 * @environment ESP32-S3-N16R8 | PlatformIO | Arduino Framework
 * @version 1.2 (ШАГ 5: Миграция LEDC)
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
#include "modules/unified_ledc/unified_ledc.h" // 🆕 ШАГ 5: LEDC
#include "config/pins.h"
#include "common/types.h"

static const char* TAG = "MAIN";

// ============================================================================
// Глобальные объекты модулей
// ============================================================================
static I2CMasterController i2c_manager;
static PCA9685ServoController servo_controller;
static UnifiedLEDCController motor_controller; // 🆕 ШАГ 5

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

    // 3️⃣ 🆕 LEDC (ШАГ 5)
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

            // 🆕 Моторы (ШАГ 5): маппинг 1000-2000 мкс → 0-100%
            if (motor_controller.isInitialized()) {
                float throttlePct = map(packet.comThrottle, 1000, 2000, 0, 100);
                throttlePct = constrain(throttlePct, 0.0f, 100.0f);
                motor_controller.setMotorPower(0, throttlePct);
                motor_controller.setMotorPower(1, throttlePct);
            }
        }
    }

    // 🔹 2. Статистика
    if (millis() - last_stats_print >= 10000) {
        LoRaStats_t lora_stats = lora_get_stats();
        ESP_LOGI(TAG, "📊 Stats | LoRa RX: %lu/%lu | CRC: %lu | ACK: %lu/%lu | RSSI: %d dBm",
                 lora_stats.packets_success, lora_stats.packets_received, lora_stats.crc_errors,
                 lora_stats.acks_success, lora_stats.acks_sent, lora_stats.last_rssi);

        ESP_LOGI(TAG, "🤖 Servo: %s | ⚡ Motors: %s",
                 servo_controller.isInitialized() ? "OK" : "OFF",
                 motor_controller.isInitialized() ? "OK" : "OFF");

        last_stats_print = millis();
    }

    // 🔹 3. Heartbeat & RTOS yield
    ESP_LOGD(TAG, "⏱️ Tick #%lu | Heap: %u B", tick++, ESP.getFreeHeap());
    delay(100);
}

/*
### 🛠 Ключевые изменения и пояснения для миграции:

| Аспект | Arduino IDE (ESP32) | PlatformIO (ESP32-S3) | Решение в миграции |
|--------|---------------------|------------------------|-------------------|
| **Частота/Разрешение** | `1kHz`, `10-bit` | Совместимо | Оставлено `1000 Гц`, `10 бит` (оптимально для ESC/DC моторов) |
| **Пины** | GPIO 32, 33 | GPIO 16, 21 | Адаптировано под `Config::Pins` ESP32-S3. Пины 16/21 не конфликтуют с USB/PSRAM |
| **Обработка ошибок** | `Serial.println` | `ESP_LOGE/W/I/D` | Полностью переведено на `esp_log`. Вывод через `ESP_LOGD` при `CORE_DEBUG_LEVEL` |
| **Безопасность** | `delay()` в тестах | Аналогично | Добавлен `constrain()` и аппаратный лимит `35%` в `runStartupDiagnostic()` |
| **Интеграция** | `#include` в `.ino` | Модульная структура | Вынесено в `modules/unified_ledc/`, подключено в `main.cpp` |

### ✅ Что нужно сделать в PlatformIO:
1. Создайте папку `modules/unified_ledc/` в `src/`.
2. Поместите туда `unified_ledc.h` и `unified_ledc.cpp`.
3. Замените ваш текущий `main.cpp` на приведённый выше.
4. Убедитесь, что в `platformio.ini` указано:
   ```ini
   board = esp32-s3-devkitc-1
   framework = arduino
   board_build.partitions = default_16MB.csv
   upload_speed = 921600
   monitor_speed = 115200
   build_flags =
       -DCORE_DEBUG_LEVEL=5  ; Включает ESP_LOGD/I/W/E
       -DARDUINO_USB_CDC_ON_BOOT=1

 */
