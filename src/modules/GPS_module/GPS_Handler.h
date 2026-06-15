

//  Отличная задача! Разработаем полноценный модуль GPS_Handler для ATGM336H
//  с поддержкой GPS+BDS (двухрежимный). Вот комплексное решение:
//  🛰️ GPS_Handler.h
//

#ifndef GPS_HANDLER_H
#define GPS_HANDLER_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <esp_log.h>
#include <vector>       // 🔑 ДОБАВИТЬ ЭТУ СТРОКУ для std::vector<Waypoint>
#include "common/CommonTypes.h"
#include "config/pins.h"

/**
 * @file GPS_Handler.h
 * @brief Обработчик GPS/BDS навигатора ATGM336H с двухрежимным позиционированием
 * @version 3.0.0
 * 
 * @details
 * - Поддержка GPS (США) и BDS (Китай) одновременно
 * - Высокоточное позиционирование с фильтрацией данных
 * - Навигационные расчеты (расстояние, азимут, скорость)
 * - Автоматическая калибровка и валидация данных
 * - Поддержка путевых точек и маршрутов
 */

// Режимы работы навигационной системы
enum class NavigationMode {
    GPS_ONLY,           // Только GPS
    BDS_ONLY,           // Только BeiDou (BDS)
    GPS_BDS_COMBINED,   // Комбинированный режим (приоритет)
    AUTO_SWITCHING      // Автоматическое переключение
};

// Статус фиксации позиции
enum class FixStatus {
    NO_FIX,             // Нет фиксации
    FIX_2D,             // 2D фиксация (широта/долгота)
    FIX_3D,             // 3D фиксация (с высотой)
    FIX_DGPS,           // Дифференциальная коррекция
    FIX_RTK             // Высокоточная RTK фиксация
};

// Структура спутниковой информации
struct SatelliteInfo {
    uint8_t prn;        // Номер спутника
    uint8_t elevation;  // Угол места (0-90°)
    uint16_t azimuth;   // Азимут (0-359°)
    uint8_t snr;        // Отношение сигнал/шум (0-99 dBHz)
    bool used;          // Используется в решении
};

// Структура путевой точки
struct Waypoint {
    double latitude;
    double longitude;
    double altitude;
    String name;
    uint32_t arrival_radius; // Радиус прибытия (метры)
};

// Основная структура GPS данных
struct GPSData {
    // Основные координаты
    double latitude;        // Широта в градусах
    double longitude;       // Долгота в градусах  
    double altitude;        // Высота в метрах (над эллипсоидом)
    double geoid_height;    // Высота геоида
    
    // Навигационные параметры
    float speed;           // Скорость в км/ч
    float course;          // Курс в градусах
    float magnetic_variation; // Магнитное склонение
    
    // Качество сигнала
    uint8_t satellites_visible; // Видимые спутники
    uint8_t satellites_used;    // Используемые спутники
    uint8_t fix_quality;        // Качество фиксации (1-5)
    float hdop;            // Горизонтальная точность
    float vdop;            // Вертикальная точность  
    float pdop;            // Позиционная точность
    
    // Время и дата
    uint8_t hour;          // Часы (0-23)
    uint8_t minute;        // Минуты (0-59)
    uint8_t second;        // Секунды (0-59)
    uint8_t day;           // День (1-31)
    uint8_t month;         // Месяц (1-12)
    uint16_t year;         // Год
    
    // Дополнительные параметры
    uint32_t timestamp;    // Временная метка (millis())
    FixStatus fix_status;  // Статус фиксации
    NavigationMode nav_mode; // Режим навигации
    
    // Массивы детальной информации
    SatelliteInfo satellites[24]; // Информация о спутниках
};

class GPSHandler {
private:
    HardwareSerial* _gpsSerial;
    GPSData _currentData;
    NavigationMode _navMode;
    bool _initialized;
    bool _dataValid;
    uint32_t _lastUpdate;
    uint32_t _fixAcquisitionTime;
    String _nmeaBuffer;
    
    // Навигационные параметры
    Waypoint _homePoint;
    Waypoint _targetPoint;
    std::vector<Waypoint> _route;
    uint8_t _currentWaypoint;
    
    // Статистика
    uint32_t _parseErrors;
    uint32_t _validPackets;
    uint32_t _satelliteChanges;
    
    // Приватные методы
    bool initializeGPSModule();
    bool sendConfiguration();
    bool setNavigationMode(NavigationMode mode);
    void parseNMEA(const String& nmea);
    void processGGA(const String& data);
    void processRMC(const String& data);
    void processGSA(const String& data);
    void processGSV(const String& data);
    double toDecimalDegrees(const String& coord, const String& direction);
    bool validateChecksum(const String& nmea);
    
public:
    // Конструктор
    GPSHandler(HardwareSerial* serial = &Serial1);
    
    // Основные методы
    bool begin(int baudRate = 9600, NavigationMode mode = NavigationMode::GPS_BDS_COMBINED);
    void update();
    bool setBaudRate(int baudRate);
    
    // Получение данных
    const GPSData& getData() const { return _currentData; }
    bool hasFix() const { return _currentData.fix_status != FixStatus::NO_FIX; }
    bool hasValidData() const { return _dataValid && (millis() - _lastUpdate < 5000); }
    uint8_t getSatelliteCount() const { return _currentData.satellites_used; }
    float getHDOP() const { return _currentData.hdop; }
    
    // Навигационные функции
    double calculateDistance(double lat1, double lon1, double lat2, double lon2) const;
    float calculateBearing(double lat1, double lon1, double lat2, double lon2) const;
    float calculateBearingToTarget() const;
    double calculateDistanceToTarget() const;
    bool setHomePosition(double lat, double lon, double alt = 0);
    bool setTargetPosition(double lat, double lon, double alt = 0);
    bool addWaypoint(const Waypoint& wp);
    void clearRoute();
    bool navigateToNextWaypoint();
    
    // Управление режимами
    bool switchToGPSOnly();
    bool switchToBDSOnly();
    bool switchToCombinedMode();
    NavigationMode getCurrentMode() const { return _navMode; }
    
    // Диагностика и тестирование
    void printDetailedInfo() const;
    void printSatelliteInfo() const;
    void printNavigationInfo() const;
    bool runSelfTest();
    void resetModule();
    
    // Статистика
    uint32_t getParseErrors() const { return _parseErrors; }
    uint32_t getValidPackets() const { return _validPackets; }
    float getSuccessRate() const;
    
    // Вспомогательные функции
    static String fixStatusToString(FixStatus status);
    static String navModeToString(NavigationMode mode);
    bool isInitialized() { return _initialized; }
};

#endif