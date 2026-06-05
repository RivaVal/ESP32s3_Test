
// src/config/module_config.h
#ifndef CFG_MODULE_CONFIG_H
#define CFG_MODULE_CONFIG_H
#endif

/**
* @file module_config.h
* @brief Централизованная конфигурация модулей проекта
* @details Двухуровневая система: compile-time + runtime
*/
#pragma once
#include <cstdint>

// ============================================================================
// 🔧 КОНФИГУРАЦИЯ КОМПИЛЯЦИИ (Preprocessor)
// ============================================================================
// Изменение этих флагов требует перекомпиляции!
// Можно переопределять через build_flags в platformio.ini

// ============================================================================
// 🔧 ПЕРЕКЛЮЧАТЕЛИ ВЕРСИЙ МОДУЛЕЙ ДЛЯ BATTERY_MONITOR
// ============================================================================
// 1 = включить в компиляцию, 0 = полностью исключить из сборки
#define CFG_ENABLE_BATTERY_MONITOR_LEGACY 1   // ✅ ВКЛ: классический ADC API
#define CFG_ENABLE_BATTERY_MONITOR_V2     0   // ❌ ВЫКЛ: новый API (недоступен)

// Обратная совместимость: если включен хоть один, включаем общий флаг
#if (CFG_ENABLE_BATTERY_MONITOR_LEGACY == 1 || CFG_ENABLE_BATTERY_MONITOR_V2 == 1)
    #define CFG_ENABLE_BATTERY_MONITOR 0
#else
    #define CFG_ENABLE_BATTERY_MONITOR 0
#endif


    //----------------------------------------------------------------------------------------
    //                #ifndef CFG_ENABLE_BATTERY_MONITOR
    //                #define CFG_ENABLE_BATTERY_MONITOR 0   // 1 = включить модуль, 0 = выключить
    //                #endif

    //                #ifndef CFG_BATTERY_MONITOR_USE_V2
    //                #define CFG_BATTERY_MONITOR_USE_V2 1   // 1 = использовать V2, 0 = использовать legacy V1
    //                #endif
//----------------------------------------------------------------------------------------

#ifndef CFG_ENABLE_SERVO_MOTORS
#define CFG_ENABLE_SERVO_MOTORS    1   ///< 1 = Включить код сервоприводов (PCA9685)
#endif

#ifndef CFG_ENABLE_TRACTION_MOTORS
#define CFG_ENABLE_TRACTION_MOTORS 1   ///< 1 = Включить код тяговых моторов (LEDC)
#endif

#ifndef CFG_ENABLE_RADIO
#define CFG_ENABLE_RADIO           1   ///< 1 = Включить код радиомодуля (LoRa SX1278)
#endif

#ifndef CFG_ENABLE_IMU
#define CFG_ENABLE_IMU             1   ///< 1 = Включить код инерциального модуля (MPU9250)
#endif

#ifndef CFG_ENABLE_SD
#define CFG_ENABLE_SD              0   ///< 1 = Включить поддержку SD карты
#endif

#ifndef CFG_ENABLE_ESP32_CAM
#define CFG_ENABLE_ESP32_CAM       0   ///< 1 = Включить поддержку ESP32-CAM
#endif

#ifndef CFG_ENABLE_GPS
#define CFG_ENABLE_GPS             0   ///< 1 = Включить поддержку GPS (UART)
#endif

#ifndef CFG_ENABLE_I2C_MASTER
#define CFG_ENABLE_I2C_MASTER      1   ///< 1 = Включить нативный драйвер I2C
#endif

#ifndef CFG_ENABLE_UART_DRIDGE
#define CFG_ENABLE_UART_BRIDGE     1   ///< 1 = Включить UART мост to Rpi
#endif

// ============================================================================
// 🔧 КОНФИГУРАЦИЯ МОДУЛЕЙ (Runtime)
// ============================================================================
struct ModuleConfig {
    bool enableServoMotors;
    bool enableTractionMotors;
    bool enableRadio;
    bool enableIMU;
    bool enableSD;
    bool enableESP32Cam;
    bool enableGPS;
    bool enableI2CMaster;
    bool enableBatteryMonitor;
    bool enableUARTBridgeRPI;
};

// Инициализация на основе макросов
extern const ModuleConfig g_ModuleConfig;

// ============================================================================
// 🔧 ВСПОМОГАТЕЛЬНЫЕ МАКРОСЫ ДЛЯ УСЛОВНОЙ КОМПИЛЯЦИИ
// ============================================================================
// Упрощают написание #if в коде
#define IF_ENABLED_SERVO(x)     (CFG_ENABLE_SERVO_MOTORS ? (x) : nullptr)
#define IF_ENABLED_RADIO(x)     (CFG_ENABLE_RADIO ? (x) : nullptr)
#define IF_ENABLED_IMU(x)       (CFG_ENABLE_IMU ? (x) : nullptr)

// Макрос для отладки конфигурации
#if CFG_ENABLE_RADIO
#define RADIO_DEBUG(msg) ESP_LOGD("RADIO", msg)
#else
#define RADIO_DEBUG(msg) ((void)0)
#endif