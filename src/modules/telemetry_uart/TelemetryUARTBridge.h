/**
 * @file TelemetryUARTBridge.h
 * @brief Бинарный мост UART2 для передачи телеметрии на RPi Zero 2W
 * @version 1.0.0
 * @date 2026
 * 
 * @details
 * • Формат пакета: [0xAA 0x55][payload 44B][CRC8][0xCC 0x33]
 * • Частота отправки: 20-50 мс (20-50 Гц)
 * • Скорость: 921600 бод с аппаратным Flow Control
 * • Обратный канал: команды RPi → ESP32 (камера/режим)
 */
#pragma once
#ifndef TELEMETRY_UART_BRIDGE_H
#define TELEMETRY_UART_BRIDGE_H

#include <Arduino.h>
#include <HardwareSerial.h>

#include "common/types.h"
#include "common/CommonTypes.h"
#include "config/pins.h"

/**
 * @brief Структура телеметрического пакета (44 байта + заголовок/футер + CRC)
 */
#pragma pack(push, 1)
struct TelemetryPacket_t {
    // === Управление (из DataComSet_t) ===
    uint16_t packet_id;           // ID пакета (0-65535)
    uint32_t timestamp;           // millis() при формировании
    uint8_t  com_up;              // Команда тангажа (0-255)
    uint8_t  com_left;            // Команда крена (0-255)
    uint16_t com_throttle;        // Тяга (1000-2000)
    uint8_t  com_flags;           // Битовая маска команд
    
    // === Стабилизация (из FlightStabilizer) ===
    float roll;                   // Крен (градусы, -90..+90)
    float pitch;                  // Тангаж (градусы, -90..+90)
    float yaw;                    // Рыскание (градусы, 0..360)
    float altitude;               // Высота (метры)
    float speed;                  // Скорость (м/с)
    
    // === Навигация ===
    double latitude;              // Широта (градусы)
    double longitude;             // Долгота (градусы)
    
    // === Батарея ===
    float battery_voltage;        // Напряжение (В)
    uint8_t battery_percent;      // Заряд (%)
    
    // === Сигнал и статус ===
    int8_t  rssi;                 // RSSI LoRa (dBm)
    uint8_t flight_mode;          // 1=MANUAL, 2=STAB, 3=FULL
    uint8_t imu_status;           // Статус IMU (битовая маска)
    
    // === CRC8 (вычисляется отдельно, не входит в payload) ===
    // uint8_t crc8;              // Вычисляется по полю [packet_id..flight_mode]
};
#pragma pack(pop)

/**
 * @brief Структура команд обратного канала RPi → ESP32
 * @details 0x01: Перезапуск камеры, 0x02: Смена режима полета, 0xFF: Ping
 */
enum class RPiCommand : uint8_t {
    CMD_RESTART_CAM = 0x01,
    CMD_CHANGE_MODE = 0x02,
    CMD_HEARTBEAT   = 0xFF
};


constexpr size_t TELEMETRY_PAYLOAD_SIZE = sizeof(TelemetryPacket_t);  // 44 байта
constexpr uint8_t  UART_HEADER_1 = 0xAA;
constexpr uint8_t  UART_HEADER_2 = 0x55;
constexpr uint8_t  UART_FOOTER_1 = 0xCC;
constexpr uint8_t  UART_FOOTER_2 = 0x33;

/**
 * @brief Класс управления UART-мостом телеметрии
 */
class TelemetryUARTBridge {
public:
    /**
     * @brief Инициализация UART2 с аппаратным Flow Control
     * @param baudrate Скорость (рекомендуется 921600)
     * @param tx_pin GPIO для TX (Config::Pins::UART_RPI_TX = 5)
     * @param rx_pin GPIO для RX (Config::Pins::UART_RPI_RX = 6)
     * @param rts_pin GPIO для RTS (опционально, -1 = отключено)
     * @param cts_pin GPIO для CTS (опционально, -1 = отключено)
     * @return true при успешной инициализации
     */
    static bool begin(uint32_t baudrate = 921600, 
                     uint8_t tx_pin = Config::Pins::UART_RPI_TX,
                     uint8_t rx_pin = Config::Pins::UART_RPI_RX,
                     int8_t rts_pin = -1, int8_t cts_pin = -1);
    
    /**
     * @brief Отправка телеметрического пакета (неблокирующая)
     * @param pkt Ссылка на структуру телеметрии
     * @return true если пакет поставлен в очередь отправки
     */
    static bool sendTelemetry(const TelemetryPacket_t& pkt);
    
    /**
     * @brief Проверка наличия входящих команд от RPi
     * @param cmd[out] Буфер для команды (если есть)
     * @return true если команда получена
     */
    static bool checkCommand(uint8_t& cmd);
    
    /**
     * @brief Статический расчёт CRC8 (Poly 0x07, Init 0x00)
     * @param data Указатель на данные
     * @param length Длина данных
     * @return Вычисленный CRC8
     */
    static uint8_t calculateCRC8(const uint8_t* data, size_t length);
    
    /**
     * @brief Получение статистики отправки
     */
    static void getStats(uint32_t& sent, uint32_t& errors, float& avg_latency_ms);

    /**
    * @brief Проверка состояния инициализации UART-моста
    * @return true если UART2 успешно настроен и готов к работе
    * @note Статический метод — вызывается без экземпляра класса
    */
    static bool isInitialized() { return _initialized; }  // ← inline-реализация
    
    /**
     * @brief Неблокирующая обработка входящих команд от RPi
     * @param handler Лямбда или указатель на функцию обработки команды
     */
    static void processRPiCommands(void (*handler)(RPiCommand cmd));
   

private:
    static HardwareSerial* _uart;
    static bool _initialized;
    static uint32_t _packets_sent;
    static uint32_t _send_errors;
    static uint64_t _total_latency_us;
    
    /**
     * @brief Внутренняя отправка с фреймингом и CRC
     */
    static bool _sendRaw(const uint8_t* payload, size_t length);
};

#endif // TELEMETRY_UART_BRIDGE_H
