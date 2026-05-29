


// VibrationLogger.cpp 
// cpp
// 

#include "VibrationLogger.h"
#include <esp_log.h>

VibrationLogger::VibrationLogger() 
    : _enabled(false), _sdReady(false), _sumVib(0), _maxVib(0), _sampleCount(0), _lastFlushTime(0) {}

bool VibrationLogger::begin(const char* filename) {
    if (!SD.begin()) { // Используйте свой CS пин, если нужно: SD.begin(CS_PIN)
        ESP_LOGW("SD_LOG", "⚠️ SD Card failed, logging to Serial only.");
        _sdReady = false;
    } else {
        _sdReady = true;
        // Создаем/открываем файл для дозаписи
        _file = SD.open(filename, FILE_APPEND);
        if (!_file) {
            ESP_LOGE("SD_LOG", "❌ Failed to open %s", filename);
            _sdReady = false;
        } else {
            writeHeader();
            ESP_LOGI("SD_LOG", "✅ SD Logging started: %s", filename);
        }
    }
    _enabled = true;
    _lastFlushTime = millis();
    return true;
}

void VibrationLogger::enableLogging(bool enable) {
    _enabled = enable;
}

void VibrationLogger::recordSample(float vibrationLevel, const char* filterName, float currentKp) {
    if (!_enabled) return;
    
    // Усреднение
    _sumVib += vibrationLevel;
    if (vibrationLevel > _maxVib) _maxVib = vibrationLevel;
    _sampleCount++;
    
    // Вывод в монитор порта (всегда, если enabled)
    ESP_LOGI("VIB_MON", "📊 Vib: %.3f | Filter: %s | PID_Kp: %.2f", vibrationLevel, filterName, currentKp);
}

void VibrationLogger::flushToSD() {
    if (!_enabled || !_sdReady || _sampleCount == 0) return;
    
    // Запись только если прошло > 500 мс
    if (millis() - _lastFlushTime < 500) return;

    float avgVib = _sumVib / _sampleCount;
    _file.printf("%lu, %.4f, %.4f, %s, %.2f\n", 
                 millis(), avgVib, _maxVib, "Filter", 0.0); // Здесь можно передать имя фильтра
    
    _file.flush(); // Гарантируем запись на диск
    
    // Сброс буфера
    _sumVib = 0;
    _maxVib = 0;
    _sampleCount = 0;
    _lastFlushTime = millis();
}

void VibrationLogger::writeHeader() {
    _file.println("timestamp_ms, avg_vibration, max_vibration, active_filter, pid_kp");
    _file.flush();
}
