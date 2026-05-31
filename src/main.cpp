
/**
* @file main.cpp
* @brief Точка входа проекта UAV_ESP32S3_Core
* @details
* - Инициализация USB CDC, PSRAM, I2C, PCA9685, LoRa
* - Условная компиляция модулей через ModuleConfig
* - Основной цикл приёма LoRa-команд и маршрутизации к сервоприводам
* 
* @environment ESP32-S3-N16R8 | PlatformIO | Arduino Framework
* @version 2.0 (Нормализованная версия)
*/
#include <Arduino.h>
#include <esp_log.h>

// ============================================================================
// 1. КОНФИГУРАЦИЯ И БАЗОВЫЕ ТИПЫ
// ============================================================================
#include "config/pins.h"
#include "config/module_config.h"
#include "common/types.h"

// #include "modules/pca9685_servo/pca9685_servo.h"
#include "modules/telemetry_uart/TelemetryUARTBridge.h"
#include "modules/unified_ledc/unified_ledc.h"
#include "modules/flight_stabilizer/FlightStabilizer.h"
#include "modules/filter_manager/FilterManager.h"


// ============================================================================
// 2. ПОДКЛЮЧЕНИЕ МОДУЛЕЙ (условная компиляция)
// ============================================================================
#if CFG_ENABLE_RADIO
#include "modules/lora_communicator/lora_communicator.h"
#endif

#if CFG_ENABLE_I2C_MASTER
#include "modules/i2c_master/i2c_master.h"
static I2CMasterController i2c_manager;
#endif

#if CFG_ENABLE_SERVO_MOTORS
#include "modules/pca9685_servo/pca9685_servo.h"
static PCA9685ServoController servo_controller;
#endif

// 🔧 ВРЕМЕННО ОТКЛЮЧИТЬ, если файлы отсутствуют:
#if CFG_ENABLE_IMU
#include "modules/flight_stabilizer/FlightStabilizer.h"
static FlightStabilizer flightStabilizer;
#endif

// Условная компиляция выберет **только нужный класс**. 
// Старый заголовочный файл с ошибками даже не попадёт 
// в цепочку компиляции PlatformIO.
#if CFG_ENABLE_BATTERY_MONITOR && CFG_ENABLE_BATTERY_MONITOR_LEGACY
#include "modules/batteryMonitor/BatteryMonitor.h"
static BatteryMonitor batteryMonitor;  // ← Правильный тип!
#endif

#if CFG_ENABLE_UART_BRIDGE
#include "modules/telemetry_uart/TelemetryUARTBridge.h"
static ...;
#endif

// #if CFG _ENABLE_BATTERY_MONITOR
// #include "modules/batteryMonitorV2/BatteryMonitorV2.h"
// static ...;
// #endif

static const char* TAG = "MAIN";

// Условная компиляция выберет **только нужный класс**. 
// Старый заголовочный файл с ошибками даже не попадёт 
// в цепочку компиляции PlatformIO.
///#if CFG _ENABLE_BATTERY_MONITOR_LEGACY
//    #include "modules/batteryMonitor/BatteryMonitor.h"  // Старый (будет игнорироваться, пока флаг =0)
//    static BatteryMonitor batteryMonitor;
///#elif CFG _ENABLE_BATTERY_MONITOR_V2
//    #include "modules/batteryMonitorV2/BatteryMonitorV2.h"  // Новый
//    static BatteryMonitorV2 batteryMonitor;
///#endif

// ============================================================================
// setup(): Инициализация системы
// ============================================================================
void setup() {
    //=========================================================================
    // 1. Инициализация Native USB CDC (критично для ESP32-S3)
    //=========================================================================
    Serial.begin(115200);
    uint32_t start = millis();
    while (!Serial && millis() - start < 3000) { 
        delay(10); 
        yield(); 
    }
    delay(500);

    ESP_LOGI(TAG, "🚀 UAV_ESP32S3_Core Startup | Chip: %s Rev%d", 
             ESP.getChipModel(), ESP.getChipRevision());
    ESP_LOGI(TAG, "💾 Heap: %u B | PSRAM: %u B", 
             ESP.getFreeHeap(), ESP.getPsramSize());
    
    // Уровень отладки (можно менять через platformio.ini)
    esp_log_level_set("*", ESP_LOG_DEBUG);

    // ========================================================================
    // 2. Инициализация I2C Master
    // ========================================================================
#if CFG_ENABLE_I2C_MASTER
    if (g_ModuleConfig.enableI2CMaster) {
        ESP_LOGI(TAG, "🔌 Initializing I2C Master...");
        if (!i2c_manager.begin()) {
            ESP_LOGE(TAG, "❌ I2C initialization FAILED!");
        } else {
            ESP_LOGI(TAG, "✅ I2C Master ready");
        }
    }
#endif

    // ========================================================================
    // 3. Инициализация PCA9685 (требует рабочий I2C)
    // ========================================================================
#if CFG_ENABLE_SERVO_MOTORS
    if (g_ModuleConfig.enableServoMotors) {
        ESP_LOGI(TAG, "🤖 Initializing PCA9685 Servo Controller...");
#if CFG_ENABLE_I2C_MASTER
        if (g_ModuleConfig.enableI2CMaster && i2c_manager.isInitialized()) {
            if (!servo_controller.begin(i2c_manager)) {
                ESP_LOGE(TAG, "❌ PCA9685 init FAILED!");
            } else {
                ESP_LOGI(TAG, "✅ PCA9685 ready | 7 servos on CH0-6");
                servo_controller.resetToNeutral(); // Безопасный старт
            }
        } else {
            ESP_LOGW(TAG, "⚠️ PCA9685 skipped: I2C not initialized");
        }
#else
        ESP_LOGE(TAG, "❌ PCA9685 requires I2C, but I2C is disabled in config!");
#endif
    }
#endif

    // ========================================================================
    // 4. Инициализация LoRa SX1278
    // ========================================================================
#if CFG_ENABLE_RADIO
    if (g_ModuleConfig.enableRadio) {
        ESP_LOGI(TAG, "📡 Initializing LoRa (SX1278)...");
        if (!lora_init()) {
            ESP_LOGE(TAG, "❌ LoRa init FAILED! Critical error.");
            // Индикация аппаратной ошибки
            pinMode(LED_BUILTIN, OUTPUT);
            while (true) {
                digitalWrite(LED_BUILTIN, HIGH); delay(100);
                digitalWrite(LED_BUILTIN, LOW);  delay(200);
            }
        }
        ESP_LOGI(TAG, "✅ LoRa ready");
    }
#endif

    // ========================================================================
    // 4. Инициализация BATTERY_MONITOR
    // ========================================================================
    #if CFG_ENABLE_BATTERY_MONITOR
        ESP_LOGI(TAG, "🔋 Initializing Battery Monitor...");
        if (!batteryMonitor.begin(4)) { // 4S по умолчанию
            ESP_LOGE(TAG, "❌ Battery Monitor init failed");
        }
    #endif


    ESP_LOGI(TAG, "🟢 Setup complete. Entering main loop...");
}


// ============================================================================
// loop(): Основной цикл
// ============================================================================
void loop() {
    static uint32_t last_stats_print = 0;

#if CFG_ENABLE_RADIO
    // 1. Обновление FSM LoRa
    lora_loop();

    // 2. Обработка входящих пакетов
    if (lora_packet_available()) {
        DataComSet_t pkt;
        // 🔑 Передача по ссылке (без &), как объявлено в inline-обёртке
        size_t len = lora_read_packet(pkt);
        
        if (len == sizeof(DataComSet_t)) {
            ESP_LOGI(TAG, "📥 RX | ID:%u | Up:%d | Left:%d | RSSI:%d dBm",
                     pkt.packet_id, pkt.comUp, pkt.comLeft,
                     lora_get_stats().lastRssi);

            // Маршрутизация команд к сервоприводам (если активны)
#if CFG_ENABLE_SERVO_MOTORS
            if (servo_controller.isInitialized()) {
                // 0-255 → 0-180° внутри processFlightCommands()
                servo_controller.processFlightCommands(pkt.comUp, pkt.comLeft);
            }
#endif
        }
    }

    // 3. Периодическая печать статистики (каждые 10 сек)
    if (millis() - last_stats_print >= 10000) {
        auto stats = lora_get_stats();
        ESP_LOGI(TAG, "📊 Stats | RX:%lu/%lu | CRC:%lu | RSSI:%d dBm",
                 stats.packetsReceivedSuccess, stats.packetsReceived,
                 stats.crcErrors, stats.lastRssi);
        last_stats_print = millis();
    }
#endif

#if CFG_ENABLE_BATTERY_MONITOR
    batteryMonitor.update();
    if (batteryMonitor.isCritical()) {
        ESP_LOGW(TAG, "🛑 CRITICAL BATTERY VOLTAGE!");
        // Здесь можно добавить emergency actions
    }
#endif

    // 4. Отдача управления планировщику FreeRTOS
    delay(100);
}


//---------------------------------------------------------------------------------
/*
// 🔧 ВРЕМЕННЫЙ ТЕСТОВЫЙ main.cpp (только LoRa)
#include <Arduino.h>
#include <esp_log.h>
#include "config/pins.h"
#include "common/types.h"
#include "config/module_config.h"
#include "modules/lora_communicator/lora_communicator.h"
#include "modules/i2c_master/i2c_master.h"
#include "modules/pca9685_servo/pca9685_servo.h"

 // Глобальный экземпляр контроллера I2C
static I2CMasterController i2c_manager;  // Глобальный экземпляр контроллера I2C


static const char* TAG = "MAIN_TEST";

void setup() {
    Serial.begin(115200);
    uint32_t start = millis();
    while (!Serial && millis() - start < 3000) { delay(10); yield(); }
    delay(500);
    
    ESP_LOGI(TAG, "🚀 LoRa Test Startup | Chip: %s", ESP.getChipModel());
    esp_log_level_set("*", ESP_LOG_DEBUG);
    

    // 🔹 Условная инициализация модулей
    #if CFG_ENABLE_RADIO
    if (g_ModuleConfig.enableRadio) {
        ESP_LOGI(TAG, "📡 Initializing LoRa...");
        // 🔹 Инициализация LoRa
        ESP_LOGI(TAG, "📡 Initializing LoRa...");
        if (!lora_init()) {
            ESP_LOGE(TAG, "❌ LoRa init FAILED!");
            while(true) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(200); }
        }
        ESP_LOGI(TAG, "✅ LoRa ready");
    }
    #endif
    
    #if CFG_ENABLE_I2C_MASTER
    if (g_ModuleConfig.enableI2CMaster) {
        ESP_LOGI(TAG, "🔌 Initializing I2C...");
        i2c_manager.begin();
    }
    #endif

    #if CFG_ENABLE_SERVO_MOTORS
    if (g_ModuleConfig.enableServoMotors) {
        ESP_LOGI(TAG, "🔌 Initializing Servo...");
        i2c_manager.begin();
    }
    #endif

    // 🔹 Runtime-проверка для дополнительной гибкости
    if (g_ModuleConfig.enableServoMotors) {
        ESP_LOGI(TAG, "🤖 Servo module enabled at runtime");
        // ... инициализация серво ...
    }

}

void loop() {
    static uint32_t last_print = 0;
    
    // 🔹 Обработка LoRa
    lora_loop();
    
    if (lora_packet_available()) {
        DataComSet_t pkt;
        // 🔑 ИСПРАВЛЕНО: передаём ссылку, а не указатель!
        size_t len = lora_read_packet(pkt);  // ← Без &
        if (len > 0) {
            ESP_LOGI(TAG, "📥 RX | ID:%u | Up:%d | Left:%d | RSSI:%d",
                     pkt.packet_id, pkt.comUp, pkt.comLeft, 
                     lora_get_stats().lastRssi);  // 🔑 lastRssi с большой буквы!
        }
    }
    
    // 🔹 Статистика каждые 10 сек
    if (millis() - last_print >= 10000) {
        auto stats = lora_get_stats();
        ESP_LOGI(TAG, "📊 Stats | RX:%lu/%lu | CRC:%lu | RSSI:%d dBm",
                 stats.packetsSuccess, stats.packetsReceived,
                 stats.crcErrors, stats.lastRssi);  // 🔑 lastRssi с большой буквы!
        last_print = millis();
    }
    
    delay(100);
}

*/