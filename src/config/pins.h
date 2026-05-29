
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

    // --- I2C Шина (Общая для MPU9250, PCA9685) ---
    constexpr uint8_t I2C_SDA    = 8;
    constexpr uint8_t I2C_SCL    = 9; // Конфликт с RST(9) устранен (RST -> 15)

} //END namespace Pins

    // =========================================================================
    // ПАРАМЕТРЫ РАДИО (LoRa)
    // =========================================================================
    namespace Radio {
        // ============================================================================
        // 🔧 ФЛАГИ УСЛОВНОЙ КОМПИЛЯЦИИ LoRa
        // ============================================================================
        /**
        * @brief Флаг поддержки DIO1 для SX1278
        * @details Установите в 1 для модулей с DIO1 (RadioLib 7.6.0+)
        *          Установите в 0 для старых модулей (только DIO0)
        * @note Все пины остаются зафиксированными в Config::Pins
        */
        #ifndef LORA_ENABLE_DIO1
        #define LORA_ENABLE_DIO1 1  // 🔑 ИЗМЕНИТЬ: 1 => с DIO1, 0 = только DIO0
        #endif

        // ============================================================================
        // 🔧 АВТОПРОВЕРКА КОНФИГУРАЦИИ (compile-time assert)
        // ============================================================================
        // ✅ СТАЛО (ИСПРАВЛЕНО):
        #if LORA_ENABLE_DIO1
            // 🔑 static_assert выполняется на этапе компиляции C++ и корректно 
            //    проверяет constexpr константы из namespace Config
            static_assert(Config::Pins::LORA_DIO1 != RADIOLIB_NC,
                    "❌ ОШИБКА: LORA_ENABLE_DIO1=1, но Config::Pins::LORA_DIO1 не задан (равен RADIOLIB_NC)!");
        #endif        
        //====================================================

        constexpr uint16_t TRANSCEIVER_DELAY = 850;
        constexpr uint16_t CONNECTION_CHECK_INTERVAL = 10000;
        constexpr uint16_t DATA_SEND_INTERVAL = 1000;
        constexpr uint16_t ACK_REQUEST_INTERVAL = 17;

        constexpr uint16_t TRANSMIT_INTERVAL = 500;
        constexpr uint16_t AUX_TIMEOUT_MS = 300;
        constexpr uint16_t RECEIVE_TIMEOUT_MS = 1000;
        constexpr uint16_t AUX_WAIT_TIMEOUT_MS = 500;
        constexpr uint16_t SEND_INTERVAL_MS = 50; // 20 Гц // или 200,// Transmitter
        constexpr  bool    REQUEST_ACK = true; // или false — зависит от режима        
 
        // 🔧 ОБНОВИТЬ для SX1278  CRC
        //====================================================
        constexpr float FREQUENCY = 433.0f;     // MHz
        constexpr uint8_t SPREADING_FACTOR = 7;   // 7-12 SF7
        constexpr float BANDWIDTH = 125.0f;     // kHz (125 kHz) - ДОПУСТИМОЕ ЗНАЧЕНИЕ ДЛЯ SX1278
        constexpr uint8_t CODING_RATE = 5;      // 5-8 (4/5) // 5 -> 4/5, 6 -> 4/6, etc.
        constexpr uint8_t OUTPUT_POWER = 14;    // dBm (обычно 2-20dBm)  // dBm (max 17 for high power modules)
        constexpr uint8_t PREAMBLE_LEN = 8;     // символов // Number of symbols
        constexpr uint8_t SYNC_WORD = 0x12;     // 0x12 - стандартное, 0x34 - пример

        // --- Параметры работы ---
        // constexpr uint32_t SEND_INTERVAL_MS = 50; // 20 Гц
        constexpr uint32_t ACK_TIMEOUT_MS = 20; // 20 мс (только для передатчика)
        constexpr uint8_t RX_PRINT_INTERVAL = 16; // Печатать каждый N-й пакет
        constexpr uint8_t TX_PRINT_INTERVAL = 10;    // Каждые 10 пакетов
    };  // end namespace Radio


    namespace Timing {   

            // Таймауты и интервалы
        constexpr uint16_t TX_TIMEOUT_MS = 200;
        constexpr uint16_t RX_TIMEOUT_MS = 200;
        constexpr uint16_t ACK_TIMEOUT_MS = 300; // заменим старый ACK_TIMEOUT
        constexpr uint16_t RESEND_DELAY_MS = 100; // заменим старый RESEND_DELAY
        constexpr uint8_t MAX_RETRIES = 2;
        constexpr uint16_t STATS_PRINT_INTERVAL_MS = 3500 ;
 
        // Неблокирующие таймеры
        constexpr uint32_t SYSTEM_TEST_INTERVAL = 100;
        constexpr uint32_t SERVO_TEST_DURATION = 3000;
        constexpr uint32_t MOTOR_TEST_DURATION = 2000;
        constexpr uint32_t STATUS_UPDATE_INTERVAL = 5000;
        
        // Таймауты инициализации
        constexpr uint32_t INIT_TIMEOUT_MS = 5000;
        constexpr uint32_t MODULE_INIT_DELAY = 100;
        constexpr uint32_t PRINT_DATACOMSET_INTERVAL = 17 ; // Интервал печати поступивших в RECEIVER данных
        // LED индикация
        constexpr uint8_t LED_PIN = 2; // Используем LED_STATUS пин
        constexpr uint16_t LED_BLINK_INTERVAL_MS = 100;

            //  namespace Timing {
        constexpr uint32_t PACKET_FRESHNESS_MS = 10000;  // Свежесть пакета
        constexpr uint32_t CONNECTION_TIMEOUT_MS = 5000; // Таймаут соединения
            //  };

    }; // end namespace Timing 


    //        namespace Timing {
    //            constexpr uint32_t STATS_PRINT_INTERVAL_MS = 3500;
    //            constexpr uint32_t CONNECTION_TIMEOUT_MS   = 5000;
    //        } //END namespace Timing
} //END namespace Config
