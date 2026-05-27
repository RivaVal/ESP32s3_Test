/**
 * @file types.h
 * @brief Общие типы данных, структуры пакетов и макросы
 * @details Адаптировано из CommonTypes.h. Содержит только необходимое для LoRa модуля.
 * @version 1.0
 */
#pragma once
#include <Arduino.h>
#include <cstdint>
#include <esp_log.h>

// ============================================================================
// 📦 Структуры пакетов (упакованы для точного размера)
// ============================================================================
#pragma pack(push, 1)
struct DataComSet_t {
    uint8_t  preamble[2] = {0xAA, 0x55};
    uint16_t packet_id = 0;
    uint8_t  comUp = 90;
    uint8_t  comLeft = 90;
    uint16_t comThrottle = 1500;
    uint8_t  comParashut = 0;
    uint32_t timestamp = 0;
    uint8_t  comSetAll = 0;
    uint8_t  crc8 = 0;
};

struct AckPacket_t {
    uint8_t  preamble[2] = {0x55, 0xAA};
    uint16_t packet_id = 0;
    uint32_t timestamp = 0;
    uint8_t  status = 0;
    uint8_t  crc8 = 0;
};
#pragma pack(pop)

// Проверка размеров на этапе компиляции (защита от ошибок выравнивания)
static_assert(sizeof(DataComSet_t) == 15, "Invalid DataComSet_t size");
static_assert(sizeof(AckPacket_t) == 10,  "Invalid AckPacket_t size");

// ============================================================================
// 🔑 Флаги команд (битовая маска comSetAll)
// ============================================================================
#define COMAND_IS_EMPTY         0b00000000
#define STABILIZATION_FLAG      0b00000001
#define PARASHUT_FLAG           0b00000010
#define PHOTO_CARD_FLAG         0b00000100
#define GPS_DATA_FLAG           0b00001000
#define SCREW_FLAG              0b00010000
#define ACK_REQUEST_FLAG        0b00100000
#define BATTERY_CHECK           0b01000000
#define SYSTEM_STATUS           0b10000000

// ============================================================================
// 📊 Статистика приёмника
// ============================================================================
struct ReceiverStats {
    uint32_t packetsReceived = 0;
    uint32_t packetsSuccess = 0;
    uint32_t crcErrors = 0;
    uint32_t acksSent = 0;
    uint32_t acksSuccess = 0;
    int16_t  lastRssi = -999;
    uint32_t lastPacketTime = 0;
};

// ============================================================================
// 🔧 CRC8 прототип
// ============================================================================
uint8_t calculateCRC8(const uint8_t* data, size_t length);

// ============================================================================
// 🎚️ Уровни отладки
// ============================================================================
enum class DebugLevel : uint8_t {
    DEBUG_NONE = 0, DEBUG_ERRORS = 1, DEBUG_WARN = 2,
    DEBUG_INFO = 3, DEBUG_STATS = 4, DEBUG_PRINT = 5,
    DEBUG_VERBOSE = 6, DEBUG_ALL = 7
};
