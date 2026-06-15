// Проект Sender / Receiver (_Receiver_)
// Файл :: CommonTypes.h
//========================================
//  Шаг 2: Создаем файл CommonTypes.h
//========================================
//**1. `CommonTypes.h` (исправленный)**
/**
* @file CommonTypes.h
* @brief Common data structures, constants, and types for the project.
* @version 1.1.0 - Добавлена поддержка конфигураций АКБ 3S-12S
* @date 2026
*/

/**
 * @file CommonTypes.h
 * @brief Common data structures, constants, and types for the project.
 * @version 1.0.0
 * @date 2026
 */

#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#pragma once
 
#include <Arduino.h>
#include <cstdint>
#include <esp_err.h>
//  #include "Config.h"
//==================================================================================



// 🔧 ДОБАВЛЯЕМ предварительные объявления чтобы избежать циклических зависимостей
struct ModuleConfig; // Предварительное объявление

//==================================================================================
// 🔧 Макросы для обработки ошибок ESP-IDF
//==================================================================================
/**
 * @brief Типобезопасные макросы проверки ESP-IDF ошибок
 * @details Разделены на BOOL/VOID версии, чтобы избежать 
 *          'return from void function' при компиляции ESP-IDF 5.0+
 */
#define ESP_ERROR_CHECK_RETURN_BOOL(x) do {   \
    esp_err_t __err_rc = (x);                 \
    if (__err_rc != ESP_OK) {                 \
        ESP_LOGE("ERROR", "Failed at %s:%d (%s)", __FILE__, __LINE__, esp_err_to_name(__err_rc)); \
        return false;                         \
    }                                         \
} while(0)

#define ESP_ERROR_CHECK_RETURN_VOID(x) do {   \
    esp_err_t __err_rc = (x);                 \
    if (__err_rc != ESP_OK) {                 \
        ESP_LOGE("ERROR", "Failed at %s:%d (%s)", __FILE__, __LINE__, esp_err_to_name(__err_rc)); \
        return;                               \
    }                                         \
} while(0)


#define ESP_ERROR_CHECK_VOID(x) do {          \
    esp_err_t __err_rc = (x);                 \
    if (__err_rc != ESP_OK) {                 \
        ESP_LOGE("ERROR", "Failed at %s:%d (%s)", __FILE__, __LINE__, esp_err_to_name(__err_rc)); \
        return;                               \
    }                                         \
} while(0)

//==================================================================================
// 🔋 ТАБЛИЦА РАСЧЕТА CRC8 НА ОСНОВЕ ПОЛИНОМА  0x07
//==================================================================================
// Таблица CRC8 (полином 0x07)
static const uint8_t CRC8_TABLE[256] = {
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};

inline uint8_t calculateCRC8(const uint8_t* data, size_t length) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < length; i++) crc = CRC8_TABLE[crc ^ data[i]];
    return crc;
}

//==================================================================================
// 🔋 ТАБЛИЦА КОНФИГУРАЦИЙ АККУМУЛЯТОРОВ (3S - 12S LiPo)
//==================================================================================
/**
 * @struct BatteryConfig_t
 * @brief Конфигурация для конкретного типа аккумуляторной сборки
 * 
 * @note Напряжения указаны для LiPo/Li-Ion элементов:
 * - Номинал: 3.7В на ячейку
 * - Макс: 4.2В на ячейку (полный заряд)
 * - Мин: 3.0В на ячейку (разряд)
 * - Крит: 2.8В на ячейку (аварийный разряд)
 */
struct BatteryConfig_t {
    uint8_t cellCount;          ///< Количество ячеек (S)
    float voltageMax;           ///< Максимальное напряжение (В) - 100%
    float voltageNominal;       ///< Номинальное напряжение (В) - ~50%
    float voltageMin;           ///< Минимальное напряжение (В) - 5%
    float voltageCritical;      ///< Критическое напряжение (В) - 0%
    const char* name;           ///< Человеко-читаемое имя
};

/**
 * @brief Таблица поддерживаемых конфигураций АКБ
 * @note Индекс массива = cellCount - 3 (для 3S индекс 0, для 12S индекс 9)
 */
constexpr BatteryConfig_t BATTERY_CONFIG_TABLE[] = {
    {3,  12.6f, 11.1f, 9.0f,  8.4f,  "3S LiPo"},
    {4,  16.8f, 14.8f, 12.0f, 11.2f, "4S LiPo"},
    {5,  21.0f, 18.5f, 15.0f, 14.0f, "5S LiPo"},
    {6,  25.2f, 22.2f, 18.0f, 16.8f, "6S LiPo"},
    {7,  29.4f, 25.9f, 21.0f, 19.6f, "7S LiPo"},
    {8,  33.6f, 29.6f, 24.0f, 22.4f, "8S LiPo"},
    {9,  37.8f, 33.3f, 27.0f, 25.2f, "9S LiPo"},
    {10, 42.0f, 37.0f, 30.0f, 28.0f, "10S LiPo"},
    {11, 46.2f, 40.7f, 33.0f, 30.8f, "11S LiPo"},
    {12, 50.4f, 44.4f, 36.0f, 33.6f, "12S LiPo"}
};

constexpr uint8_t BATTERY_CONFIG_COUNT = sizeof(BATTERY_CONFIG_TABLE) / sizeof(BATTERY_CONFIG_TABLE[0]);
constexpr uint8_t MIN_CELL_COUNT = 3;
constexpr uint8_t MAX_CELL_COUNT = 12;

//==================================================================================
// 🔋 Структура состояния батареи
//==================================================================================
/**
* @struct BatteryStatus_t
* @brief Структура состояния батареи с расширенной информацией
*/
struct BatteryStatus_t {
    float voltage;              ///< Текущее напряжение (В)
    float percentage;           ///< Процент заряда (0-100)
    bool isOk;                  ///< Флаг нормального состояния
    bool isLow;                 ///< Флаг низкого заряда (<20%)
    bool isCritical;            ///< Флаг критического состояния (<5%)
    uint32_t lastCheckTime;     ///< Время последней проверки (millis)
    uint16_t adcValue;          ///< Сырое значение ADC (0-4095)
    uint8_t cellCount;          ///< Количество ячеек (3-12)
    float voltagePerCell;       ///< Напряжение на ячейку (В)
    const char* configName;     ///< Имя конфигурации из таблицы
};



// 🔧 ПЕРЕНЕСЛИ СТРУКТУРЫ ДАННЫХ СЮДА
    // В секцию флагов управления в comSetAll добавьте:
    //#define STABILIZATION_FLAG     0b00000001  ///< Активация системы стабилизации

// Флаги управления в comSetAll

    #define COMAND_IS_EMPTY         0b00000000  ///< Отсутствуют активные ФЛАГИ
    #define STABILIZATION_FLAG      0b00000001  ///< Активация системы стабилизации
    #define PARASHUT_FLAG           0b00000010  ///< Активация парашюта
    #define PHOTO_CARD_FLAG         0b00000100  ///< Запрос фотосъемки
    #define GPS_DATA_FLAG           0b00001000  ///< Запрос GPS данных
    #define SCREW_FLAG              0b00010000  ///< Команда поворота винтом
    #define ACK_REQUEST_FLAG        0b00100000  ///< Запрос подтверждения приема
    //    #define EMERGENCY_STOP     0b00100000  ///< Аварийная остановка
    #define BATTERY_CHECK           0b01000000  ///< Запрос состояния батареи
    #define SYSTEM_STATUS           0b10000000  ///< Запрос статуса системы

// Уровни детализации отладки
enum class DebugLevel : uint8_t  {
    DEBUG_NONE    = 0,  // Без вывода (минимальная нагрузка)
    DEBUG_ERRORS  = 1,  // Только критические ошибки
    DEBUG_WARN    = 2,  // Печать предупреждений по коду
    DEBUG_INFO    = 3,  // Печать информационных сообщений
    DEBUG_STATS   = 4,  // Ошибки + периодическая статистика
    DEBUG_PRINT   = 5,  // Печать отладочных сообщений Basic prints
    DEBUG_VERBOSE = 6,  // Detailed prints
    DEBUG_ALL     = 7   // Полная отладка (все события)
};

#pragma pack(push, 1)

// --- Добавить в CommonTypes.h ---
struct LoRaConfig_t {
    float frequency;
    float bandwidth;
    uint8_t spreadingFactor;
    uint8_t codingRate;
    uint8_t syncWord;
    // Добавьте другие параметры, если нужно
};


#pragma pack(push, 1)
/**
 * @struct DataComSet_t
 * @brief Структура передаваемых данных управления
 */
struct DataComSet_t {
    // uint8_t  preamble[2];    // 0xAA 0x55 - идентификатор начала пакета
    uint8_t preamble[2] ;    // 0xAA 0x55 - идентификатор начала пакета
    uint16_t packet_id = 0;      // Счетчик пакетов (0-65535)
    uint8_t  comUp = 90;          // Команда вверх 0-255
    uint8_t  comLeft = 90;        // Команда влево 0-255  
    uint16_t comThrottle = 1500;    // Тяга двигателя 1000-2000
    uint8_t  comParashut = 0;    // Парашют 0/1
    uint32_t timestamp = 0;      // Временная метка (millis())
    uint8_t  comSetAll = 0;      // Битовая маска команд
    uint8_t  crc8 = 0;           // Контрольная сумма
};

/**
 * @struct AckPacket_t  
 * @brief Структура подтверждения приема
 */
struct AckPacket_t {
    // uint8_t preamble[2];     // 0x55 0xAA - идентификатор ACK
    uint8_t preamble[2] ;     // 0x55 0xAA - идентификатор ACK
    uint16_t packet_id = 0;      // ID подтверждаемого пакета
    uint32_t timestamp = 0;      // Время отправки пакета
    uint8_t status = 0;          // Статус получения (1-OK, 0-ERROR)
    uint8_t crc8 = 0;            // Контрольная сумма
};
#pragma pack(pop)

// 🔧 ПЕРЕНЕСЛИ РАЗМЕРЫ БУФЕРОВ И ВАЛИДАЦИЮ 
constexpr uint8_t MAX_PACKET_SIZE = sizeof(DataComSet_t);
constexpr uint8_t ACK_PACKET_SIZE = sizeof(AckPacket_t);

// ================== ВАЛИДАЦИЯ КОНФИГУРАЦИИ ==================
static_assert(sizeof(DataComSet_t) == 15, "Invalid DataComSet_t size");
static_assert(sizeof(AckPacket_t) == 10, "Invalid AckPacket_t size");


// Удобные макросы для доступа к данным датчиков
#define GYRO_X(sd) ((sd).gyro.x)
#define GYRO_Y(sd) ((sd).gyro.y)
#define GYRO_Z(sd) ((sd).gyro.z)

#define ACCEL_X(sd) ((sd).accel.x)
#define ACCEL_Y(sd) ((sd).accel.y)
#define ACCEL_Z(sd) ((sd).accel.z)

#define MAG_X(sd) ((sd).mag.x)
#define MAG_Y(sd) ((sd).mag.y)
#define MAG_Z(sd) ((sd).mag.z)

/**
* @struct SensorData
* @brief Данные сенсора GY-87 (MPU6050 + HMC5883L + BMP180)
*/
/*   
struct SensorData {
    uint32_t timestamp;  // Временная метка
    
    // Сырые данные MPU6050
    float accel[3];      // Акселерометр (g)
    float gyro[3];       // Гироскоп (°/с)
    float temp;          // Температура MPU6050
    
    // HMC5883L магнитометр
    float mag[3];        // Магнитометр (μT)
    float heading;       // Расчетный курс (°)
    
    // BMP180 барометр
    float pressure;      // Давление (гПа)
    float altitude;      // Высота (м)
    float temperature;   // Температура BMP180 (°C)
    
    // 🔑 КЛЮЧЕВОЕ ДОПОЛНЕНИЕ: УГЛЫ ЭЙЛЕРА ДЛЯ СТАБИЛИЗАЦИИ ПОЛЁТА 🔑
    float roll;          // Крен (ось X) -90°..+90°
    float pitch;         // Тангаж (ось Y) -90°..+90°
    float yaw;           // Рыскание (ось Z) 0°..360°
    
    uint8_t status;      // Статус (битовая маска)
};
*/

/**
 * @struct SensorData
 * @brief Данные сенсора GY-87 с углами Эйлера для стабилизации
 * 
 * @note ВАЖНО: Структура расширена полями углов Эйлера и статуса
 * для совместимости с системой стабилизации полёта.
 */
struct SensorData {
    // Сырые данные датчиков MPU6050
    struct { float x, y, z; } gyro;    ///< Гироскоп (рад/с)
    struct { float x, y, z; } accel;   ///< Акселерометр (м/с²)
    struct { float x, y, z; } mag;     ///< Магнитометр (мкТ)

    // HMC5883L магнитометр
    // float mag[3];        // Магнитометр (μT)
    float heading;       // Расчетный курс (°)
  
  // Производные данные (вычисляются в GY87_Handler)
    // 🔑 КЛЮЧЕВОЕ ДОПОЛНЕНИЕ: УГЛЫ ЭЙЛЕРА ДЛЯ СТАБИЛИЗАЦИИ ПОЛЁТА 🔑
    float roll;          ///< Крен (ось X) -90°..+90°
    float pitch;         ///< Тангаж (ось Y) -90°..+90°
    float yaw;           ///< Рыскание (ось Z) 0°..360°
  
    // Данные барометра
    float temperature;   ///< Температура BMP180 (°C)
    float pressure;      ///< Давление (гПа)
    float altitude;      ///< Высота (м)
  
    // Статус датчика (битовая маска)
    uint8_t status;      ///< 0x01 = данные валидны, 0x02 = калибровка завершена
  
    // Конструктор по умолчанию
    SensorData() : roll(0), pitch(0), yaw(0), temperature(0), 
                    pressure(0), altitude(0), status(0) {}
};


/**
 * @struct ModuleStats_t
 * @brief Статистика работы модуля
 */
struct ModuleStats_t {
    uint32_t tx_packets;             // Отправленные пакеты
    uint32_t rx_packets;             // Принятые пакеты
    uint32_t crc_errors;             // Ошибки CRC
    uint32_t tx_errors;              // Ошибки передачи
    uint32_t rx_errors;              // Ошибки приема
    int16_t last_rssi;               // Последний RSSI
    uint32_t resets;                 // Сбросы модуля
    uint32_t reconnects;             // Переподключения
    uint32_t aux_timeouts;           // Таймауты AUX
    uint32_t busy_errors;            // Ошибки занятости
    uint32_t not_ready_errors;       // Ошибки неготовности
    uint32_t connection_lost_events; // Потеря соединения
    uint32_t receive_timeouts;       // Таймауты приема
    uint16_t preamble_errors;        // Ошибки преамбулы
};

// ============================================================================
// STRUCTS
// ============================================================================
/**
 * @brief Статистика приёмника
 */
struct ReceiverStats {
    uint32_t packetsReceived = 0;
    uint32_t packetsReceivedSuccess = 0;
    uint32_t crcErrors = 0;
    uint32_t invalidPreamble = 0;
    uint32_t duplicatePackets = 0;
    uint32_t packetErrors = 0;
    uint32_t lastPacketTime = 0;
    int16_t lastRssi = 0;
    float lastSNR = 0.0f;
    uint32_t fsmErrors = 0;
    uint32_t recoveryAttempts = 0;
    
    // ACK stats
    uint32_t acksSent = 0;
    uint32_t acksSentSuccess = 0;
    uint32_t acksSuccess = 0;
    uint32_t ackSendErrors = 0;
    uint32_t ackRequests = 0;
    uint32_t ackResponses = 0;
    uint32_t ackErrors = 0;
    uint32_t ackTimeouts = 0;
    float ackSuccessRate = 0.0f;
    
    // 🆕 ДОБАВЛЕНО: Счетчик таймаутов приема (DIO1)
    uint32_t receive_timeouts = 0; 
};


#pragma pack(pop)

// ================== ПЕРЕЧИСЛЕНИЯ ==================
/**
 * @enum SystemState
 * @brief Состояния системы
 */
enum class SystemState {
    BOOT,           // Загрузка системы
    INITIALIZING,   // Инициализация компонентов  
    STANDBY,        // Ожидание команд
    ACTIVE,         // Активная работа
    ERROR,          // Ошибка системы
    RECOVERY        // Восстановление
};

constexpr uint32_t MODULE_UPDATE_INTERVAL = 100;
/**
 * @enum EbyteStatus
 * @brief Статусы операций модуля E49
 */
enum class EbyteStatus {
    SUCCESS,                        // Успешное выполнение
    ERROR_INIT,                     // Ошибка инициализации
    ERROR_SEND,                     // Ошибка отправки
    ERROR_RECEIVE,                  // Ошибка приема
    ERROR_CRC,                      // Ошибка контрольной суммы
    ERROR_AUX_TIMEOUT,              // Таймаут AUX пина
    ERROR_NO_NEW_DATA,              // Нет новых данных
    ERROR_INVALID_DATA,             // Невалидные данные
    ERROR_INVALID_PREAMBLE,         // Неправильная преамбула
    ERROR_PREAMBLE,                 // Ошибка преамбулы
    ERROR_SEND_FAILED,              // Сбой отправки
    ERROR_RECEIVE_FAILED,           // Сбой приема
    ERROR_MODULE_NOT_RESPONDING,    // Модуль не отвечает
    ERROR_INVALID_STATE,            // Неверное состояние
    ERROR_BUSY,                     // Модуль занят
    ERROR_NOT_READY,                // Модуль не готов
    ERROR_CONNECTION_LOST,          // Потеря соединения
    EBYTE_ERROR_ACK_TIMEOUT,        // Таймаут подтверждения
    EBYTE_OPERATION_PENDING         // Операция в процессе
};


// В Receiver.ino — ДО setup()
// ControllerDebug_t CDebug = {
struct ControllerDebug_t {
  bool recieve ;
  bool sender;
  bool lora ;
  bool motors ;
  bool servos;
  bool ledc ;
  bool slide_pot ;
  bool joystick ;
  bool cc ; 
};


/**
 * @class TimerMillis
 * @brief Работа с таймерами
 */
class TimerMillis
{
  private:
    static constexpr uint32_t  MIN_INTERVAL = 1;     //
    uint32_t                   last         = 0;     //
    uint32_t                   interval     = MIN_INTERVAL;

  public:
    // Конструктор по умолчанию
    TimerMillis() 
    {
        reset(); // Инициализируем временем создания
    }

    // Конструктор с интервалом
    TimerMillis(uint32_t _interval) 
    {
        setInterval(_interval);
        reset();
    }

    // Сброс таймера
    void reset() 
    {
    last = millis();
    }

    // Проверка готовности таймера
    bool is_ready()
    {
    uint32_t current = millis();
    
    // ✅ БЕЗОПАСНОЕ СРАВНЕНИЕ С УЧЕТОМ ПЕРЕПОЛНЕНИЯ
    if ((current - last) >= interval) 
    {
        last = current; // Обновляем время последнего срабатывания
        return true;
    }
    return false;
    }

    // Установка интервала
    void setInterval(uint32_t _interval) 
    {
        interval = (_interval < MIN_INTERVAL) ? MIN_INTERVAL : _interval;
    }

    // Получение текущего интервала
    uint32_t getInterval() const 
    {
        return interval;
    }

    // Получение прошедшего времени
    uint32_t getElapsed() const 
    {
    return millis() - last;
    }

    // Получение оставшегося времени
    uint32_t getRemaining() const 
    {
    uint32_t elapsed = getElapsed();
    return (elapsed >= interval) ? 0 : (interval - elapsed);
    }
};

    // #endif // end TimerMillis_h


/**
 * @class BitMask
 * @brief Работа с битовыми масками
 */
class BitMask {
private:
    uint8_t mask;
public:
    BitMask() : mask(0) {}
    explicit BitMask(uint8_t initialMask) : mask(initialMask) {}
    
    /**
     * @brief Установка бита
     * @param bit Номер бита (0-7)
     * @param value Значение (true/false)
     */
    void setBit(uint8_t bit, bool value) {
        if (bit > 7) return;
        if (value) {
            mask |= (1 << bit);
        } else {
            mask &= ~(1 << bit);
        }
    }
    
    /**
     * @brief Чтение бита
     * @param bit Номер бита (0-7)
     * @return Значение бита
     */
    bool getBit(uint8_t bit) const {
        if (bit > 7) return false;
        return (mask & (1 << bit)) != 0;
    }

    /**
     * @brief Очистка бита
     * @param bit Номер бита (0-7)
     */
    void clearBit(uint8_t bit) {
        mask &= ~(1 << bit); 
    }

    /**
     * @brief Вывод битовой маски в двоичном формате
     */
    void printBinary() const {
        Serial.print("0b");
        for (int i = 7; i >= 0; i--) {
            Serial.print(getBit(i) ? "1" : "0");
        }
    }

    /**
     * @brief Получение маски
     * @return Текущая битовая маска
     */
    uint8_t getMask() const { 
        return mask; 
    }

    /**
     * @brief Установка маски
     * @param newMask Новая битовая маска
     */
    void setMask(uint8_t newMask) { 
        mask = newMask; 
    }
    
    /**
     * @brief Отладочный вывод маски
     * @param name Имя маски для вывода
     */
    void printDebug(const char* name) const {
        Serial.printf("%s: ", name);
        printBinary();
        Serial.printf(" (0x%02X)\n", mask);
    }
};

// Добавить в CommonTypes.h или LoRaCommunicator.h
/**
 * @brief Печать содержимого структуры DataComSet_t
 * @param data Структура данных для печати
 * @param source Источник данных (для заголовка)
 */ 
void printDataComSet(const DataComSet_t& data, const char* source);

// В CommonTypes.h — после структур
//uint8_t calculateCRC8(const uint8_t* data, size_t length);

// В самом конце CommonTypes.h, перед #endif
uint8_t calculateCRC8(const uint8_t* data, size_t length);

// Глобальная переменная отладки — объявляется ОДИН РАЗ
extern ControllerDebug_t CDebug;


#endif // COMMON_TYPES_H
