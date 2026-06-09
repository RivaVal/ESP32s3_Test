/**
 * @file GY91_Handler.h
 * @brief Обработчик модуля GY-91 (MPU-9250 + BMP280)
 * @version 1.0.0
 * @date 2026
 * 
 * @details
 * Модуль GY-91 содержит два датчика на одной шине I2C:
 * - MPU-9250 (адрес 0x68): IMU 9 осей (акселерометр + гироскоп + магнитометр AK8963)
 * - BMP280 (адрес 0x76/0x77): Барометр + температура
 * 
 * Архитектура:
 * - Использует нативный I2C драйвер ESP-IDF 5.0+
 * - Интеграция с FilterManager для фильтрации IMU данных
 * - Расчёт высоты по барометрической формуле
 * - Подробная отладка через ESP_LOG
 * 
 * @note MPU-9250 использует I2C bypass для доступа к магнитометру AK8963 (адрес 0x0C)
 */
#pragma once

#ifndef GY91_HANDLER_H
#define GY91_HANDLER_H

#include <Arduino.h>
#include <esp_log.h>
#include "common/CommonTypes.h"
#include "modules/i2c_master/i2c_master.h"
#include "modules/filter_manager/FilterManager.h"

// ============================================================================
// Адреса устройств на шине I2C
// ============================================================================
#define MPU9250_ADDR            0x68    ///< Адрес MPU-9250
#define AK8963_ADDR             0x0C    ///< Адрес магнитометра AK8963 (через I2C bypass)
#define BMP280_ADDR             0x76    ///< Адрес BMP280 (может быть 0x77)

// ============================================================================
// Регистры MPU-9250
// ============================================================================
#define MPU9250_WHO_AM_I        0x75    ///< WHO_AM_I (должно быть 0x71)
#define MPU9250_PWR_MGMT_1      0x6B    ///< Power Management 1
#define MPU9250_PWR_MGMT_2      0x6C    ///< Power Management 2
#define MPU9250_CONFIG          0x1A    ///< Configuration (DLPF)
#define MPU9250_GYRO_CONFIG     0x1B    ///< Gyroscope Configuration
#define MPU9250_ACCEL_CONFIG    0x1C    ///< Accelerometer Configuration
#define MPU9250_ACCEL_CONFIG2   0x1D    ///< Accelerometer Configuration 2
#define MPU9250_INT_PIN_CFG     0x37    ///< Interrupt Pin / Bypass Enable
#define MPU9250_INT_ENABLE      0x38    ///< Interrupt Enable
#define MPU9250_ACCEL_XOUT_H    0x3B    ///< Accelerometer X-axis High Byte
#define MPU9250_GYRO_XOUT_H     0x43    ///< Gyroscope X-axis High Byte
#define MPU9250_TEMP_OUT_H      0x41    ///< Temperature High Byte

// ============================================================================
// Регистры AK8963 (магнитометр)
// ============================================================================
#define AK8963_WIA              0x00    ///< Device ID (должно быть 0x48)
#define AK8963_ST1              0x02    ///< Status 1
#define AK8963_HXL              0x03    ///< Magnetic X-axis Low Byte
#define AK8963_HXH              0x04    ///< Magnetic X-axis High Byte
#define AK8963_HYL              0x05    ///< Magnetic Y-axis Low Byte
#define AK8963_HYH              0x06    ///< Magnetic Y-axis High Byte
#define AK8963_HZL              0x07    ///< Magnetic Z-axis Low Byte
#define AK8963_HZH              0x08    ///< Magnetic Z-axis High Byte
#define AK8963_ST2              0x09    ///< Status 2
#define AK8963_CNTL1            0x0A    ///< Control 1
#define AK8963_CNTL2            0x0B    ///< Control 2
#define AK8963_ASAX             0x10    ///< Sensitivity Adjustment X
#define AK8963_ASAY             0x11    ///< Sensitivity Adjustment Y
#define AK8963_ASAZ             0x12    ///< Sensitivity Adjustment Z

// ============================================================================
// Регистры BMP280
// ============================================================================
#define BMP280_CHIP_ID          0xD0    ///< Chip ID (должно быть 0x58)
#define BMP280_RESET            0xE0    ///< Reset register
#define BMP280_STATUS           0xF3    ///< Status register
#define BMP280_CTRL_MEAS        0xF4    ///< Control and Measurement
#define BMP280_CONFIG           0xF5    ///< Configuration
#define BMP280_PRESS_MSB        0xF7    ///< Pressure MSB
#define BMP280_PRESS_LSB        0xF8    ///< Pressure LSB
#define BMP280_PRESS_XLSB       0xF9    ///< Pressure XLSB
#define BMP280_TEMP_MSB         0xFA    ///< Temperature MSB
#define BMP280_TEMP_LSB         0xFB    ///< Temperature LSB
#define BMP280_TEMP_XLSB        0xFC    ///< Temperature XLSB
#define BMP280_CALIB_START      0x88    ///< Calibration data start

// ============================================================================
// Калибровочные коэффициенты BMP280
// ============================================================================
struct BMP280CalibData {
    uint16_t dig_T1;
    int16_t dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
};

/**
 * @class GY91Handler
 * @brief Основной класс для работы с модулем GY-91
 */
class GY91Handler {
public:
    /**
     * @brief Конструктор
     */
    GY91Handler();
    
    /**
     * @brief Инициализация всех датчиков модуля GY-91
     * @param i2c_manager Ссылка на инициализированный I2C контроллер
     * @return true при успешной инициализации
     */
    bool begin(I2CMasterController& i2c_manager);
    
    /**
     * @brief Обновление данных со всех датчиков
     * @details Вызывать с частотой 250 Гц (4 мс)
     */
    void update();
    
    /**
     * @brief Проверка успешности инициализации
     */
    bool isInitialized() const { return _initialized; }
    
    /**
     * @brief Проверка валидности данных IMU
     */
    bool isDataValid() const { return _initialized && _imuCalibrated; }
    
    /**
     * @brief Получение структуры SensorData (для совместимости с FlightStabilizer)
     */
    const SensorData& getData() const { return _sensorData; }
    
    /**
     * @brief Получение высоты от барометра (метры)
     */
    float getAltitude() const { return _altitude; }
    
    /**
     * @brief Получение температуры от барометра (°C)
     */
    float getBaroTemperature() const { return _baroTemp; }
    
    /**
     * @brief Получение давления (гПа)
     */
    float getPressure() const { return _pressure; }
    
    /**
     * @brief Установка опорного давления для расчёта высоты
     * @param seaLevelPressure Давление на уровне моря (гПа, по умолчанию 1013.25)
     */
    void setSeaLevelPressure(float seaLevelPressure = 1013.25f) { 
        _seaLevelPressure = seaLevelPressure; 
    }
    
    /**
     * @brief Калибровка барометра (считать текущее давление как уровень моря)
     */
    void calibrateBarometer();

private:
    I2CMasterController* _i2cManager;
    FilterManager _filterManager;
    SensorData _sensorData;
    
    bool _initialized;
    bool _imuCalibrated;
    bool _bmp280Initialized;
    bool _hasMagnetometer;  ///< 🔑 НОВОЕ: Флаг наличия магнитометра (true для MPU-9250, false для MPU-6050/6000)
    
    // Калибровка MPU-9250
    uint16_t _calibrationCount;
    float _gyroOffsetX, _gyroOffsetY, _gyroOffsetZ;
    float _magSensitivity[3];
    
    // Калибровка BMP280
    BMP280CalibData _bmp280Calib;
    int32_t _bmp280_t_fine;
    
    // Данные барометра
    float _altitude;
    float _baroTemp;
    float _pressure;
    float _seaLevelPressure;
    
    uint32_t _lastUpdate;
    
    // ========================================================================
    // Приватные методы MPU-9250
    // ========================================================================
    bool _initMPU9250();
    bool _enableI2CBypass();
    bool _initAK8963();
    bool _readMPU9250Data(int16_t accel[3], int16_t gyro[3], int16_t temp, int16_t mag[3]);
    
    // ========================================================================
    // Приватные методы BMP280
    // ========================================================================
    bool _initBMP280();
    bool _readBMP280Calibration();
    bool _readBMP280Data();
    int32_t _bmp280_compensate_T(int32_t adc_T);
    uint32_t _bmp280_compensate_P(int32_t adc_P);
    float _calculateAltitude(float pressure);
    
    // ========================================================================
    // Вспомогательные методы
    // ========================================================================
    void _updateIMUData();
    void _updateBaroData();
};

#endif // GY91_HANDLER_H
