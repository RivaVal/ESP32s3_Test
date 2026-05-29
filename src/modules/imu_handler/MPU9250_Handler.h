

//📄 **Файл:** `MPU9250_Handler.h` (создаётся на основе `GY87_Handler.h`)
//⚙️ **Класс:** `class MPU9250Handler`
//💻 **Код:**
//cpp
//

#pragma once
#ifndef MPU9250_HANDLER_H
#define MPU9250_HANDLER_H

#include <Arduino.h>
#include "esp_log.h"
#include "CommonTypes.h"
#include "I2C_Master.h"
#include "FilterManager.h"

/**
 * @file MPU9250_Handler.h
 * @brief Обработчик 9-осевого датчика MPU-9250 (MPU6500 + AK8963)
 * @version 1.0.0
 * @date 2026-04-03
 * @details
 * Заменяет устаревший GY87_Handler (MPU6050).
 * Поддерживает: акселерометр, гироскоп, магнитометр, температуру.
 * Использует комплементарный фильтр для Roll/Pitch и сырые данные магнитометра для Yaw.
 * Архитектура: неблокирующая, I2C Master, калибровка гироскопа при старте.
 */

class MPU9250Handler {
public:
    // 🔧 В СЕКЦИЮ PUBLIC КЛАССА MPU9250Handler ДОБАВИТЬ:
    FilterManager filterManager; ///< 🔑 Менеджер адаптивной фильтрации
    MPU9250Handler();
    bool begin(I2CMasterController& i2c_manager);
    bool isInitialized() const { return _initialized; }
    bool isDataValid() const  { return _initialized && _gyroCalibrated && (_sensorData.status & 0x01); }
    bool updateSensors();
    const SensorData& getData() const { return _sensorData; }

    // Геттеры для обратной совместимости
    float getAccelX_mss() const { return _sensorData.accel.x; }
    float getGyroX_rads() const { return _sensorData.gyro.x; }
    float getMagX_uT()    const { return _sensorData.mag.x; }
    float getTemperature_C() const { return _sensorData.temperature; }

private:
    // FilterManager _filter;
    bool _magAvailable = false; // если магнитометр отключён
    I2CMasterController* _i2cManager;
    bool _initialized;
    SensorData _sensorData;

    // Калибровка гироскопа
    bool _gyroCalibrated;
    uint16_t _calibrationCount;
    float _gyroOffsetX, _gyroOffsetY, _gyroOffsetZ;

    // Магнитометр AK8963
    float _magSensitivity[3]; ///< Заводская чувствительность (0.6 uT/LSB * ASA)
    uint32_t _lastUpdate;

    bool _initMPU6500();
    bool _enableI2CBypass();
    bool _initAK8963();
    bool _readRawData(int16_t accel[3], int16_t gyro[3], int16_t temp, int16_t mag[3]);
    void _applyComplementaryFilter(float dt);
};
#endif // MPU9250_HANDLER_H

