
/**
 * @file pins.h
 * @brief Конфигурация пинов и параметров радио для ESP32-S3-N16R8
 * @details 
 * - Адаптировано из оригинального Config.h
 * - Все пины проверены на отсутствие конфликтов с USB, Flash и PSRAM
 * 
 * @note ESP32-S3 НЕ имеет аппаратных дефолтов для SPI/I2C. Пины задаются явно!
 * @version 1.0 (Миграция в PlatformIO)
 * 
 * 
 * 
 * 
 * 
 */
#pragma once
#include <cstdint>
#include <driver/ledc.h>
#include <RadioLib.h>
#include <driver/adc.h>

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
    constexpr uint8_t I2C_SCL    = 9; // Конфликт с LORA_RST(9), устранен (LORA_RST -> 15)

    // ============================================================================
    // 🔗 UART1 ДЛЯ GPS МОДУЛЯ
    // ============================================================================
    //
    // Flow Control для GPS НЕ требуется. Используем стандартные свободные пины.
    constexpr uint8_t GPS_RX = 18;  ///< ESP32-S3 RX <- GPS TX
    constexpr uint8_t GPS_TX = 17;  ///< ESP32-S3 TX -> GPS RX


    // ============================================================================
    // 🔗 UART2 ДЛЯ RPi Zero 2W + АППАРАТНЫЙ FLOW CONTROL
    // ============================================================================
    // Пины 5, 6, 41, 42 выбраны как наиболее безопасные на ESP32-S3-N16R8.
    // Они не являются strapping-пинами и не конфликтуют с OPI PSRAM/Flash.
    constexpr uint8_t UART_RPI_TX  = 5;   ///< ESP32-S3 TX  -> RPi RX
    constexpr uint8_t UART_RPI_RX  = 6;   ///< ESP32-S3 RX  <- RPi TX
    constexpr uint8_t UART_RPI_RTS = 41;  ///< ESP32-S3 RTS -> RPi CTS (готовность принять)
    constexpr uint8_t UART_RPI_CTS = 42;  ///< ESP32-S3 CTS <- RPi RTS (готовность RPi принять)

                       
    // ============================================================================
    // ⚡ СИЛОВЫЕ МОТОРЫ (LEDC PWM)
    // ============================================================================
    // ⚡ Моторы через LEDC (аппаратный ШИМ)
    // 🔑 Проверка: пины 40,41 поддерживают LEDC на ESP32-S3
    // constexpr uint8_t MOTOR_PIN_1 = 16;      // GPIO16 -> LEDC CH1
    // constexpr uint8_t MOTOR_PIN_2 = 21;      // GPIO21 -> LEDC CH2
    constexpr uint8_t MOTOR_LEFT_PIN  = 16;  // GPIO16 -> LEDC CH1
    constexpr uint8_t MOTOR_RIGHT_PIN = 21;  // GPIO21 -> LEDC CH2

    // 🔑 КОНФИГУРАЦИЯ ПИНОВ И КАНАЛОВ (ESP32-S3)
    static constexpr uint8_t _motorPins[2]   =      {16, 21};            // GPIO16, GPIO21
        //                static constexpr ledc_channel_t _channels[2] = {LEDC_CHANNEL_1, LEDC_CHANNEL_2};
        //                static constexpr uint32_t _freqHz        = 1000;                     // Частота PWM для моторов
        //                static constexpr uint32_t _dutyResolution = 10;                      // 10 бит (0-1023)

    // 🔑 ПАРАМЕТРЫ ШИМ (Частота и разрешение)
    // static constexpr uint32_t freqHz         = 1000;  // 1 кГц (стандарт для ESC/моторов)
    // static constexpr uint32_t dutyResolution = 10;    // 10 бит (0-1023)
    // static constexpr ledc_mode_t speedMode   = LEDC_LOW_SPEED_MODE;

    // Настройки таймеров и каналов LEDC (ESP32-S3 имеет 4 таймера, 8 каналов)
    constexpr uint8_t LEDC_TIMER_NUM  = 0;   // LEDC_TIMER_0
    constexpr uint8_t LEDC_CH_LEFT    = 1;   // LEDC_CHANNEL_1
    constexpr uint8_t LEDC_CH_RIGHT   = 2;   // LEDC_CHANNEL_2

    // --- Мониторинг Батареи (ADC) ---
    // Безопасный пин, не конфликтует с USB (19/20) и Flash/PSRAM.
    // GPIO 7 = ADC1_CH6. Поддерживает калибровку (Line Fitting).
    constexpr uint8_t BATTERY_ADC_PIN = 7;

    // --- Периферия ---
    constexpr uint8_t BUZZER_PIN = 255;   // ⛔ Отключен по запросу (освобождает ресурсы)
    // constexpr uint8_t LED_STATUS = 2; // Встроенный LED
    constexpr uint8_t LED_STATUS = 2; // Встроенный LED //
            // ============================================================================
            // 🔗 UART ШИНА ДЛЯ RASPBERRY PI ZERO 2W (UART2)
            // ============================================================================
            /**
            * @brief Аппаратный UART2 для связи с RPi Zero 2W
            * @details 
            *   • TX (ESP32-S3 GPIO5) → RX (RPi GPIO14/BCM14)
            *   • RX (ESP32-S3 GPIO6) ← TX (RPi GPIO15/BCM15)
            *   • Скорость: 115200 или 921600 бод (аппаратно стабильно)
            *   • Логика 3.3V, общий GND обязателен. Уровневые конвертеры НЕ нужны.
            *   • RTS (GPIO3) → RPi CTS (ESP32 сообщает, что готов принять)
            *   • CTS (GPIO42)← RPi RTS (ESP32 получает сигнал о готовности RPi принять)
            *   • Скорость: 921600 бод. Без Flow Control при такой скорости возможны потери пакетов.
            * @note GPIO 5 и 6 безопасны на ESP32-S3-WROOM-1, не конфликтуют с Flash/PSRAM/USB.
            */
            //        constexpr uint8_t UART_RPI_TX  = 5;  ///< ESP32-S3 TX → RPi RX
            //        constexpr uint8_t UART_RPI_RX  = 6;  ///< ESP32-S3 RX ← RPi TX
            //        constexpr uint8_t UART_RPI_RTS = 3;  ///< Ready To Send (ESP32)
            //        constexpr uint8_t UART_RPI_CTS = 42; ///< Clear To Send (RPi)    

            // Опционально: аппаратный flow control (раскомментировать при высоких нагрузках)
            // constexpr uint8_t UART_RPI_RTS = 3;  ///< Ready To Send (ESP32)
            // constexpr uint8_t UART_RPI_CTS = 42; ///< Clear To Send (RPi)

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

}; // end namespace Timing adc_atten_t

namespace Battery {
        // === Конфигурация мониторинга АКБ ===
        constexpr uint8_t DEFAULT_CELL_COUNT = 4;           // 4S по умолчанию
        constexpr uint8_t BATTERY_ADC_PIN = 7;              // GPIO для ADC
            //constexpr adc_atten_t BATTERY_ADC_ATTENUATION = ADC_ATTEN_DB_12; // или ADC_ATTEN_DB_11
            // ✅ Вместо макроса используйте прямое значение:
            // ADC_ATTEN_DB_12 = 3 (см. adc_atten_t в driver/adc.h)
        constexpr uint8_t BATTERY_ADC_ATTENUATION = 3;
        constexpr uint32_t BATTERY_ADC_RESOLUTION = 4095;   // 12-bit ADC
        constexpr float BATTERY_ADC_MAX_VOLTAGE = 3.3f;     // Макс. напряжение на пине (В)
        constexpr float BATTERY_VOLTAGE_DIVIDER_RATIO = 4.0f; // Коэффициент делителя (R1+R2)/R2
        // 🔧 ДОБАВИТЬ отсутствующую константу:
        constexpr uint16_t ADC_VREF_MV = 1100;  // ← Референсное напряжение, мВ
        
        // === Тайминги ===
        constexpr uint32_t STARTUP_CHECK_DELAY = 2000;      // Задержка при старте (мс)
        constexpr uint32_t CHECK_INTERVAL_MS = 5000;        // Интервал проверки (мс)
} // END namespace Battery

} //END namespace Config


