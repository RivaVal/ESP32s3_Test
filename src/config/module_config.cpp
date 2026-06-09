/**
* @file module_config.cpp
* @brief Реализация глобальной конфигурации модулей
*/
#include "config/module_config.h"

// Глобальная конфигурация (const → размещается в flash)
const ModuleConfig g_ModuleConfig = {
    .enableServoMotors    = (CFG_ENABLE_SERVO_MOTORS == 1),
    .enableTractionMotors = (CFG_ENABLE_TRACTION_MOTORS == 1),
    .enableRadio          = (CFG_ENABLE_RADIO == 1),
    .enableIMU            = (CFG_ENABLE_IMU == 1),
    .enableGY91           = (CFG_ENABLE_GY91 == 1),  // ← Новый флаг
    .enableSD             = (CFG_ENABLE_SD == 1),
    .enableESP32Cam       = (CFG_ENABLE_ESP32_CAM == 1),
    .enableGPS            = (CFG_ENABLE_GPS == 1),
    .enableI2CMaster      = (CFG_ENABLE_I2C_MASTER == 1),
    .enableBatteryMonitor = (CFG_ENABLE_BATTERY_MONITOR == 1),
    .enableUARTBridgeRPI  = (CFG_ENABLE_UART_BRIDGE == 1)
};
