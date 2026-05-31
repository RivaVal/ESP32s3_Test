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
