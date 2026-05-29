

//### 📁 1. Модуль логирования `VibrationLogger.h` / `.cpp`
// Этот модуль отвечает за вывод в консоль и запись на SD карту.
// Оптимизация:  Логирование на карту происходит не каждый 
// тик (это убило бы производительность), а  каждые 500 мс , 
// записывая усредненные данные.

// VibrationLogger.h 
// cpp
//
#pragma once
#include <Arduino.h>
#include <SD.h>
#include <FS.h>

class VibrationLogger {
public:
    VibrationLogger();
    
    // Инициализация (вызывать в setup)
    bool begin(const char* filename = "/vib_log.csv");
    
    // Вызывать часто (например, в updateSensors)
    void recordSample(float vibrationLevel, const char* filterName, float currentKp);
    
    // Вызывать периодически (например, раз в 500мс) для сброса на SD
    void flushToSD();
    
    void enableLogging(bool enable);

private:
    File _file;
    bool _enabled;
    bool _sdReady;
    
    // Буфер для усреднения
    float _sumVib;
    float _maxVib;
    uint32_t _sampleCount;
    uint32_t _lastFlushTime;
    
    void writeHeader();
};


