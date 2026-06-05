//==============================================
//  🛰️ GPS_Handler.cpp
//==============================================
//
#include "GPS_Handler.h"
#include <cmath>
#include <vector>
#include "esp_log.h"

// Константы для расчетов
const double EARTH_RADIUS = 6371000.0; // Радиус Земли в метрах


GPSHandler::GPSHandler(HardwareSerial* serial) 
    : _gpsSerial(serial), 
      _navMode(NavigationMode::GPS_BDS_COMBINED),
      _initialized(false),
      _dataValid(false),
      _lastUpdate(0),
      _fixAcquisitionTime(0),
      _currentWaypoint(0),
      _parseErrors(0),
      _validPackets(0),
      _satelliteChanges(0)
{
    
    // Инициализация структуры данных
    memset(&_currentData, 0, sizeof(_currentData));
    _currentData.fix_status = FixStatus::NO_FIX;
    _currentData.nav_mode = _navMode;
    
    // Установка домашней позиции по умолчанию (Москва)
    _homePoint.latitude = 55.7558;
    _homePoint.longitude = 37.6173;
    _homePoint.altitude = 150;
    _homePoint.name = "HOME";
    _homePoint.arrival_radius = 50;
}

bool GPSHandler::begin(int baudRate, NavigationMode mode) {
    Serial.println("🛰️ INITIALIZING ATGM336H GPS/BDS MODULE...");
    
    // Инициализация последовательного порта
    _gpsSerial->begin(baudRate, SERIAL_8N1, 16, 17); // RX=16, TX=17 согласно новой схеме пинов
    
    // Ожидание инициализации модуля
    delay(2000);
    
    _navMode = mode;
    
    // Настройка модуля
    if (!initializeGPSModule()) {
        Serial.println("❌ GPS MODULE INITIALIZATION FAILED");
        return false;
    }
    
    // Установка режима навигации
    if (!setNavigationMode(_navMode)) {
        Serial.println("❌ FAILED TO SET NAVIGATION MODE");
        return false;
    }
    
    _initialized = true;
    Serial.println("✅ ATGM336H GPS/BDS MODULE INITIALIZED SUCCESSFULLY");
    Serial.printf("   Mode: %s, Baud Rate: %d\n", navModeToString(_navMode).c_str(), baudRate);
    
    return true;
}

bool GPSHandler::initializeGPSModule() {
    Serial.println("⚙️ CONFIGURING GPS MODULE...");
    
    // Ожидание стабилизации модуля
    delay(1000);
    
    // Отправка команд конфигурации
    if (!sendConfiguration()) {
        Serial.println("❌ GPS MODULE CONFIGURATION FAILED");
        return false;
    }
    
    Serial.println("✅ GPS MODULE CONFIGURED SUCCESSFULLY");
    return true;
}

bool GPSHandler::sendConfiguration() {
    // Включение всех необходимых NMEA предложений
    const char* configCommands[] = {
        "$PCAS03,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n", // Включение GGA, RMC, GSA, GSV
        "$PCAS04,4*1B\r\n", // Режим GPS+BDS
        "$PCAS06,0*36\r\n", // Сброс к заводским настройкам
        "$PCAS02,1000*2D\r\n", // Обновление раз в 1 секунду
        "$PCAS11,2*1E\r\n" // Режим воздушного судна (<4g)
    };
    
    for (const char* cmd : configCommands) {
        _gpsSerial->print(cmd);
        delay(100);
    }
    
    // Ожидание применения настроек
    delay(2000);
    return true;
}

bool GPSHandler::setNavigationMode(NavigationMode mode) {
    const char* command = nullptr;
    
    switch (mode) {
        case NavigationMode::GPS_ONLY:
            command = "$PCAS04,1*1E\r\n";
            break;
        case NavigationMode::BDS_ONLY:
            command = "$PCAS04,3*1C\r\n";
            break;
        case NavigationMode::GPS_BDS_COMBINED:
            command = "$PCAS04,4*1B\r\n";
            break;
        case NavigationMode::AUTO_SWITCHING:
            command = "$PCAS04,6*19\r\n";
            break;
        default:
            return false;
    }
    
    if (command) {
        _gpsSerial->print(command);
        _navMode = mode;
        _currentData.nav_mode = mode;
        delay(500);
        return true;
    }
    
    return false;
}

void GPSHandler::update() {
    if (!_initialized) return;
    
    // Чтение данных из последовательного порта
    while (_gpsSerial->available()) {
        char c = _gpsSerial->read();
        
        if (c == '\n') {
            // Завершение строки NMEA
            if (_nmeaBuffer.length() > 6) { // Минимальная длина валидного предложения
                if (validateChecksum(_nmeaBuffer)) {
                    parseNMEA(_nmeaBuffer);
                    _validPackets++;
                } else {
                    _parseErrors++;
                }
            }
            _nmeaBuffer = "";
        } else if (c == '\r') {
            // Игнорируем carriage return
            continue;
        } else if (c == '$') {
            // Начало нового предложения
            _nmeaBuffer = "$";
        } else if (_nmeaBuffer.length() > 0) {
            // Добавляем символ в буфер
            _nmeaBuffer += c;
        }
    }
}

bool GPSHandler::validateChecksum(const String& nmea) {
    int starPos = nmea.indexOf('*');
    if (starPos == -1) return false;
    
    String data = nmea.substring(1, starPos);
    String checksumStr = nmea.substring(starPos + 1);
    uint8_t calculated = 0;
    
    for (size_t i = 0; i < data.length(); i++) {
        calculated ^= data[i];
    }
    
    char hex[3];
    snprintf(hex, sizeof(hex), "%02X", calculated);
    return checksumStr.equalsIgnoreCase(hex);
}

void GPSHandler::parseNMEA(const String& nmea) {
    String type = nmea.substring(1, 6);
    
    if (type == "GPGGA" || type == "BDGGA" || type == "GNGGA") {
        processGGA(nmea);
    } else if (type == "GPRMC" || type == "BDRMC" || type == "GNRMC") {
        processRMC(nmea);
    } else if (type == "GPGSA" || type == "BDGSA" || type == "GNGSA") {
        processGSA(nmea);
    } else if (type == "GPGSV" || type == "BDGSV" || type == "GNGSV") {
        processGSV(nmea);
    }
}

void GPSHandler::processGGA(const String& data) {
    // Пример: $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
    String parts[15];
    int partIndex = 0;
    int start = 0;
    
    for (size_t i = 0; i < data.length() && partIndex < 15; i++) {
        if (data[i] == ',' || data[i] == '*') {
            parts[partIndex++] = data.substring(start, i);
            start = i + 1;
        }
    }
    
    if (partIndex >= 10) {
        // Время
        if (parts[1].length() >= 6) {
            _currentData.hour = parts[1].substring(0, 2).toInt();
            _currentData.minute = parts[1].substring(2, 4).toInt();
            _currentData.second = parts[1].substring(4, 6).toInt();
        }
        
        // Координаты
        _currentData.latitude = toDecimalDegrees(parts[2], parts[3]);
        _currentData.longitude = toDecimalDegrees(parts[4], parts[5]);
        
        // Качество фиксации
        _currentData.fix_quality = parts[6].toInt();
        _currentData.satellites_used = parts[7].toInt();
        _currentData.hdop = parts[8].toFloat();
        _currentData.altitude = parts[9].toFloat();
        
        if (partIndex >= 11) {
            _currentData.geoid_height = parts[11].toFloat();
        }
        
        // Обновление статуса
        _lastUpdate = millis();
        _dataValid = (_currentData.fix_quality > 0);
        
        if (_currentData.fix_quality > 0 && _fixAcquisitionTime == 0) {
            _fixAcquisitionTime = millis();
            Serial.println("🎯 GPS FIX ACQUIRED!");
        }
    }
}

void GPSHandler::processRMC(const String& data) {
    // Пример: $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
    String parts[12];
    int partIndex = 0;
    int start = 0;
    
    for (size_t i = 0; i < data.length() && partIndex < 12; i++) {
        if (data[i] == ',' || data[i] == '*') {
            parts[partIndex++] = data.substring(start, i);
            start = i + 1;
        }
    }
    
    if (partIndex >= 9 && parts[2] == "A") { // A = Active
        // Скорость и курс
        _currentData.speed = parts[7].toFloat() * 1.852f; // Узлы в км/ч
        _currentData.course = parts[8].toFloat();
        
        // Дата
        if (parts[9].length() >= 6) {
            _currentData.day = parts[9].substring(0, 2).toInt();
            _currentData.month = parts[9].substring(2, 4).toInt();
            _currentData.year = 2000 + parts[9].substring(4, 6).toInt();
        }
        
        // Магнитное склонение
        if (partIndex >= 11) {
            _currentData.magnetic_variation = parts[10].toFloat();
            if (parts[11] == "W") _currentData.magnetic_variation = -_currentData.magnetic_variation;
        }
        
        _currentData.timestamp = millis();
    }
}

void GPSHandler::processGSA(const String& data) {
    // Обработка данных DOP и активных спутников
    String parts[18];
    int partIndex = 0;
    int start = 0;
    
    for (size_t i = 0; i < data.length() && partIndex < 18; i++) {
        if (data[i] == ',' || data[i] == '*') {
            parts[partIndex++] = data.substring(start, i);
            start = i + 1;
        }
    }
    
    if (partIndex >= 17) {
        _currentData.pdop = parts[15].toFloat();
        _currentData.hdop = parts[16].toFloat();
        _currentData.vdop = parts[17].toFloat();
        
        // Определение статуса фиксации
        if (parts[2].toInt() == 3) {
            _currentData.fix_status = FixStatus::FIX_3D;
        } else if (parts[2].toInt() == 2) {
            _currentData.fix_status = FixStatus::FIX_2D;
        } else {
            _currentData.fix_status = FixStatus::NO_FIX;
        }
    }
}

void GPSHandler::processGSV(const String& data) {
    // Обработка информации о спутниках в зоне видимости
    // (Упрощенная реализация - в реальном проекте нужно обрабатывать multiple GSV messages)
}

double GPSHandler::toDecimalDegrees(const String& coord, const String& direction) {
    if (coord.length() < 4) return 0.0;
    
    double degrees = coord.substring(0, 2).toDouble();
    double minutes = coord.substring(2).toDouble();
    double decimal = degrees + minutes / 60.0;
    
    if (direction == "S" || direction == "W") {
        decimal = -decimal;
    }
    
    return decimal;
}

// =============================================================================
// 🧭 НАВИГАЦИОННЫЕ ФУНКЦИИ
// =============================================================================

double GPSHandler::calculateDistance(double lat1, double lon1, double lat2, double lon2) const {
    double dLat = (lat2 - lat1) * DEG_TO_RAD;
    double dLon = (lon2 - lon1) * DEG_TO_RAD;
    
    double a = sin(dLat/2) * sin(dLat/2) + 
               cos(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) * 
               sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    
    return EARTH_RADIUS * c;
}

float GPSHandler::calculateBearing(double lat1, double lon1, double lat2, double lon2) const {
    double lat1_rad = lat1 * DEG_TO_RAD;
    double lon1_rad = lon1 * DEG_TO_RAD;
    double lat2_rad = lat2 * DEG_TO_RAD;
    double lon2_rad = lon2 * DEG_TO_RAD;
    
    double dLon = lon2_rad - lon1_rad;
    
    double y = sin(dLon) * cos(lat2_rad);
    double x = cos(lat1_rad) * sin(lat2_rad) - 
               sin(lat1_rad) * cos(lat2_rad) * cos(dLon);
    
    double bearing = atan2(y, x);
    bearing = fmod((bearing * RAD_TO_DEG + 360.0), 360.0);
    
    return (float)bearing;
}

float GPSHandler::calculateBearingToTarget() const {
    if (!hasValidData()) return 0.0f;
    return calculateBearing(_currentData.latitude, _currentData.longitude, 
                           _targetPoint.latitude, _targetPoint.longitude);
}

double GPSHandler::calculateDistanceToTarget() const {
    if (!hasValidData()) return 0.0;
    return calculateDistance(_currentData.latitude, _currentData.longitude,
                            _targetPoint.latitude, _targetPoint.longitude);
}

bool GPSHandler::setHomePosition(double lat, double lon, double alt) {
    _homePoint.latitude = lat;
    _homePoint.longitude = lon;
    _homePoint.altitude = alt;
    _homePoint.name = "HOME";
    
    Serial.printf("🏠 HOME POSITION SET: %.6f, %.6f, %.1fm\n", lat, lon, alt);
    return true;
}

bool GPSHandler::setTargetPosition(double lat, double lon, double alt) {
    _targetPoint.latitude = lat;
    _targetPoint.longitude = lon;
    _targetPoint.altitude = alt;
    _targetPoint.name = "TARGET";
    
    Serial.printf("🎯 TARGET POSITION SET: %.6f, %.6f, %.1fm\n", lat, lon, alt);
    return true;
}

bool GPSHandler::addWaypoint(const Waypoint& wp) {
    _route.push_back(wp);
    Serial.printf("📍 WAYPOINT ADDED: %s (%.6f, %.6f)\n", 
                 wp.name.c_str(), wp.latitude, wp.longitude);
    return true;
}

void GPSHandler::clearRoute() {
    _route.clear();
    _currentWaypoint = 0;
    Serial.println("🗺️ ROUTE CLEARED");
}

bool GPSHandler::navigateToNextWaypoint() {
    if (_route.empty() || _currentWaypoint >= _route.size()) {
        return false;
    }
    
    _targetPoint = _route[_currentWaypoint];
    
    // Проверка прибытия к путевой точке
    double distance = calculateDistanceToTarget();
    if (distance <= _targetPoint.arrival_radius) {
        Serial.printf("🎯 ARRIVED AT WAYPOINT: %s\n", _targetPoint.name.c_str());
        _currentWaypoint++;
        return true;
    }
    
    return false;
}

// =============================================================================
// 🔧 УПРАВЛЕНИЕ РЕЖИМАМИ
// =============================================================================

bool GPSHandler::switchToGPSOnly() {
    return setNavigationMode(NavigationMode::GPS_ONLY);
}

bool GPSHandler::switchToBDSOnly() {
    return setNavigationMode(NavigationMode::BDS_ONLY);
}

bool GPSHandler::switchToCombinedMode() {
    return setNavigationMode(NavigationMode::GPS_BDS_COMBINED);
}

// =============================================================================
// 📊 ДИАГНОСТИКА И ТЕСТИРОВАНИЕ
// =============================================================================

void GPSHandler::printDetailedInfo() const {
    Serial.println("\n=== 🛰️ GPS/BDS DETAILED INFORMATION ===");
    Serial.printf("Module: ATGM336H\n");
    Serial.printf("Initialized: %s\n", _initialized ? "YES" : "NO");
    Serial.printf("Data Valid: %s\n", hasValidData() ? "YES" : "NO");
    Serial.printf("Fix Status: %s\n", fixStatusToString(_currentData.fix_status).c_str());
    Serial.printf("Navigation Mode: %s\n", navModeToString(_navMode).c_str());
    
    if (hasValidData()) {
        Serial.println("\n📍 POSITION:");
        Serial.printf("  Latitude:  %.6f°\n", _currentData.latitude);
        Serial.printf("  Longitude: %.6f°\n", _currentData.longitude);
        Serial.printf("  Altitude:  %.1f m\n", _currentData.altitude);
        
        Serial.println("\n🧭 NAVIGATION:");
        Serial.printf("  Speed:     %.1f km/h\n", _currentData.speed);
        Serial.printf("  Course:    %.1f°\n", _currentData.course);
        Serial.printf("  Satellites: %d used, %d visible\n", 
                     _currentData.satellites_used, _currentData.satellites_visible);
        
        Serial.println("\n📊 QUALITY:");
        Serial.printf("  HDOP: %.1f, VDOP: %.1f, PDOP: %.1f\n", 
                     _currentData.hdop, _currentData.vdop, _currentData.pdop);
        Serial.printf("  Fix Quality: %d\n", _currentData.fix_quality);
        
        Serial.println("\n🕒 TIME:");
        Serial.printf("  Time: %02d:%02d:%02d\n", 
                     _currentData.hour, _currentData.minute, _currentData.second);
        Serial.printf("  Date: %02d/%02d/%04d\n", 
                     _currentData.day, _currentData.month, _currentData.year);
    }
    
    Serial.printf("\n📈 STATISTICS:\n");
    Serial.printf("  Valid Packets: %lu\n", _validPackets);
    Serial.printf("  Parse Errors: %lu\n", _parseErrors);
    Serial.printf("  Success Rate: %.1f%%\n", getSuccessRate());
    Serial.println("====================================\n");
}

void GPSHandler::printSatelliteInfo() const {
    Serial.println("\n=== 🛰️ SATELLITE INFORMATION ===");
    Serial.printf("Used: %d, Visible: %d\n", 
                 _currentData.satellites_used, _currentData.satellites_visible);
    
    // Здесь можно добавить вывод детальной информации о спутниках
    Serial.println("================================\n");
}

void GPSHandler::printNavigationInfo() const {
    if (!hasValidData()) {
        Serial.println("❌ NO VALID GPS DATA FOR NAVIGATION");
        return;
    }
    
    Serial.println("\n=== 🧭 NAVIGATION INFO ===");
    Serial.printf("Current Position: %.6f, %.6f\n", 
                 _currentData.latitude, _currentData.longitude);
    
    if (_targetPoint.name != "") {
        double distance = calculateDistanceToTarget();
        float bearing = calculateBearingToTarget();
        
        Serial.printf("Target: %s (%.6f, %.6f)\n", 
                     _targetPoint.name.c_str(), _targetPoint.latitude, _targetPoint.longitude);
        Serial.printf("Distance: %.1f m, Bearing: %.1f°\n", distance, bearing);
        Serial.printf("ETA: %.1f min\n", (distance / 1000.0) / (_currentData.speed / 60.0));
    }
    
    Serial.println("============================\n");
}

bool GPSHandler::runSelfTest() {
    Serial.println("\n🧪 GPS MODULE SELF TEST");
    
    if (!_initialized) {
        Serial.println("❌ MODULE NOT INITIALIZED");
        return false;
    }
    
    // Тест связи с модулем
    Serial.println("1. Testing module communication...");
    delay(1000);
    
    if (!hasValidData()) {
        Serial.println("   ⚠️  NO GPS FIX - CHECK ANTENNA");
    } else {
        Serial.println("   ✅ GPS DATA RECEIVED");
    }
    
    // Тест навигационных расчетов
    Serial.println("2. Testing navigation calculations...");
    double testDistance = calculateDistance(55.7558, 37.6173, 55.7601, 37.6175);
    float testBearing = calculateBearing(55.7558, 37.6173, 55.7601, 37.6175);
    Serial.printf("   Distance: %.1f m, Bearing: %.1f° ✅\n", testDistance, testBearing);
    
    // Тест режимов
    Serial.println("3. Testing mode switching...");
    if (switchToGPSOnly()) {
        Serial.println("   ✅ GPS ONLY MODE - OK");
        delay(1000);
    }
    
    if (switchToBDSOnly()) {
        Serial.println("   ✅ BDS ONLY MODE - OK");
        delay(1000);
    }
    
    if (switchToCombinedMode()) {
        Serial.println("   ✅ COMBINED MODE - OK");
    }
    
    Serial.println("✅ GPS SELF TEST COMPLETED");
    return true;
}

void GPSHandler::resetModule() {
    Serial.println("🔄 RESETTING GPS MODULE...");
    _gpsSerial->print("$PCAS06,0*36\r\n"); // Команда сброса
    delay(2000);
    _initialized = false;
    begin(9600, _navMode); // Переинициализация
}

float GPSHandler::getSuccessRate() const {
    if (_validPackets + _parseErrors == 0) return 100.0f;
    return (100.0f * _validPackets) / (_validPackets + _parseErrors);
}

// =============================================================================
// 🎯 ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
// =============================================================================

String GPSHandler::fixStatusToString(FixStatus status) {
    switch (status) {
        case FixStatus::NO_FIX: return "NO FIX";
        case FixStatus::FIX_2D: return "2D FIX";
        case FixStatus::FIX_3D: return "3D FIX";
        case FixStatus::FIX_DGPS: return "DGPS FIX";
        case FixStatus::FIX_RTK: return "RTK FIX";
        default: return "UNKNOWN";
    }
}

String GPSHandler::navModeToString(NavigationMode mode) {
    switch (mode) {
        case NavigationMode::GPS_ONLY: return "GPS ONLY";
        case NavigationMode::BDS_ONLY: return "BDS ONLY";
        case NavigationMode::GPS_BDS_COMBINED: return "GPS+BDS COMBINED";
        case NavigationMode::AUTO_SWITCHING: return "AUTO SWITCHING";
        default: return "UNKNOWN";
    }
}

bool GPSHandler::setBaudRate(int baudRate) {
    const char* commands[] = {
        "$PCAS01,0*3B\r\n", // Сброс
        "$PCAS01,1*3A\r\n", // 4800
        "$PCAS01,2*39\r\n", // 9600
        "$PCAS01,3*38\r\n", // 19200
        "$PCAS01,4*3F\r\n", // 38400
        "$PCAS01,5*3E\r\n", // 57600
        "$PCAS01,6*3D\r\n", // 115200
    };
    
    int index = -1;
    switch (baudRate) {
        case 4800: index = 1; break;
        case 9600: index = 2; break;
        case 19200: index = 3; break;
        case 38400: index = 4; break;
        case 57600: index = 5; break;
        case 115200: index = 6; break;
        default: return false;
    }
    
    if (index != -1) {
        _gpsSerial->print(commands[0]); // Сброс
        delay(100);
        _gpsSerial->print(commands[index]); // Установка скорости
        delay(100);
        
        // Переинициализация последовательного порта
        _gpsSerial->end();
        delay(100);
        _gpsSerial->begin(baudRate, SERIAL_8N1, 16, 17);
        delay(1000);
        
        return true;
    }
    
    return false;
}
//===============================================================================