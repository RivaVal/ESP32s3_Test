
/**
 * @file pins.h
 * @brief Конфигурация пинов и параметров радио для ESP32-S3-N16R8
 * @details 
 * - Адаптировано из оригинального Config.h
 * - Все пины проверены на отсутствие конфликтов с USB, Flash и PSRAM
 * 
 * @note ESP32-S3 НЕ имеет аппаратных дефолтов для SPI/I2C. Пины задаются явно!
 * @version 1.0 (Миграция в PlatformIO)
 */
#pragma once
#include <cstdint>
#include <driver/ledc.h>

namespace Config {
namespace Pins {
    // ============================================================================
    // 📡 LoRa SX1278 (SPI через FSPI)
    // ============================================================================
    // ⚠️ Ограничения ESP32-S3:
    // GPIO 45-48: Flash/PSRAM (КРИТИЧЕСКИ не трогать!)
    // GPIO 19-20: Native USB (не использовать для периферии)
    // GPIO 0: Boot strapping
    
    // FSPI шина (стандартная для ESP32-S3)
    constexpr uint8_t LORA_CS    = 10; // FSPI CS-NSS
    constexpr uint8_t LORA_SCLK  = 12; // FSPI Clock
    constexpr uint8_t LORA_MISO  = 13; // FSPI MISO
    constexpr uint8_t LORA_MOSI  = 11; // FSPI MOSI
    
    // Управляющие пины LoRa
    constexpr uint8_t LORA_RST   = 15; // Reset (перенесен с 9 для избежания конфликта с I2C)
    constexpr uint8_t LORA_DIO0  = 4;  // Прерывание: пакет принят (RxDone)
    constexpr uint8_t LORA_DIO1  = 14; // Прерывание: таймаут приёма (RxTimeout)
} // namespace Pins

namespace Timing {
    constexpr uint32_t STATS_PRINT_INTERVAL_MS = 3500;
    constexpr uint32_t CONNECTION_TIMEOUT_MS   = 5000;
} // namespace Timing
} // namespace Config
