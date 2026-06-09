/**
 * @file GY91_Handler.cpp
 * @brief Реализация обработчика модуля GY-91 (MPU-9250 + BMP280)
 * @version 1.0.0
 * @date 2026
 */

#include "GY91_Handler.h"
#include <cmath>

static const char* TAG = "GY91_HANDLER";

// ============================================================================
// Константы масштабирования
// ============================================================================
constexpr float ACCEL_SCALE_2G = 9.80665f / 16384.0f;   ///< ±2g → м/с²
constexpr float GYRO_SCALE_250DPS = (250.0f / 32768.0f) * (M_PI / 180.0f);  ///< ±250°/s → рад/с
constexpr float MAG_SCALE = 0.6f;  ///< Базовая чувствительность магнитометра (мкТ/LSB)

// ============================================================================
// Конструктор
// ============================================================================
GY91Handler::GY91Handler() 
    : _i2cManager(nullptr)
    , _initialized(false)
    , _imuCalibrated(false)
    , _bmp280Initialized(false)
    , _hasMagnetometer(false)  // 🔑 НОВОЕ: По умолчанию магнитометра нет
    , _calibrationCount(0)
    , _gyroOffsetX(0), _gyroOffsetY(0), _gyroOffsetZ(0)
    , _bmp280_t_fine(0)
    , _altitude(0.0f)
    , _baroTemp(0.0f)
    , _pressure(0.0f)
    , _seaLevelPressure(1013.25f)
    , _lastUpdate(0)
{
    memset(&_sensorData, 0, sizeof(_sensorData));
    memset(&_bmp280Calib, 0, sizeof(_bmp280Calib));
    memset(_magSensitivity, 0, sizeof(_magSensitivity));
    
    ESP_LOGI(TAG, "✅ GY91Handler: конструктор завершён");
}

// ============================================================================
// Инициализация
// ============================================================================
bool GY91Handler::begin(I2CMasterController& i2c_manager) {
    ESP_LOGI(TAG, "=== 🚀 ИНИЦИАЛИЗАЦИЯ GY-91 (MPU-9250 + BMP280) ===");
    _i2cManager = &i2c_manager;
    
    // ========================================================================
    // 1. Инициализация MPU-9250
    // ========================================================================
    ESP_LOGI(TAG, "📡 Инициализация MPU-9250...");
    if (!_initMPU9250()) {
        ESP_LOGE(TAG, "❌ Ошибка инициализации MPU-9250");
        return false;
    }
    
    // ========================================================================
    // 2. Инициализация магнитометра AK8963
    // ========================================================================
    if (_hasMagnetometer) {
        ESP_LOGI(TAG, "🧲 Инициализация магнитометра AK8963...");
        if (!_enableI2CBypass()) {
            ESP_LOGE(TAG, "❌ Ошибка включения I2C Bypass");
            return false;
        }
        
        if (!_initAK8963()) {
            ESP_LOGE(TAG, "❌ Ошибка инициализации AK8963");
            return false;
        }
    } else {
        ESP_LOGW(TAG, "⚠️ Пропуск инициализации магнитометра (чип не поддерживает)");
    }  

    // ========================================================================
    // 3. Инициализация BMP280
    // ========================================================================
    ESP_LOGI(TAG, "🌡️ Инициализация BMP280...");
    if (!_initBMP280()) {
        ESP_LOGW(TAG, "⚠️ BMP280 не инициализирован (барометр недоступен)");
        _bmp280Initialized = false;
    } else {
        _bmp280Initialized = true;
        ESP_LOGI(TAG, "✅ BMP280 инициализирован");
    }
    
    // ========================================================================
    // 4. Инициализация FilterManager
    // ========================================================================
    ESP_LOGI(TAG, "🔧 Инициализация FilterManager (250 Гц)...");
    _filterManager.begin(250.0f);
    _filterManager.enableAdaptive(true);
    
    _initialized = true;
    ESP_LOGI(TAG, "✅ GY-91 полностью инициализирован");
    ESP_LOGI(TAG, "   • MPU-9250: ✅ (IMU 9 осей)");
    ESP_LOGI(TAG, "   • BMP280: %s", _bmp280Initialized ? "✅" : "❌");
    
    return true;
}

// ============================================================================
// Инициализация MPU-9250
// ============================================================================
bool GY91Handler::_initMPU9250() {
    // Сброс питания
    uint8_t data = 0x80;  // Bit 7: DEVICE_RESET
    if (!_i2cManager->writeRegister(MPU9250_ADDR, MPU9250_PWR_MGMT_1, &data, 1)) {
        ESP_LOGE(TAG, "❌ Не удалось сбросить MPU-9250");
        return false;
    }
    delay(100);
    
    // Выход из сна, выбор источника тактирования
    data = 0x01;  // CLKSEL=PLL with X axis gyroscope reference
    if (!_i2cManager->writeRegister(MPU9250_ADDR, MPU9250_PWR_MGMT_1, &data, 1)) {
        ESP_LOGE(TAG, "❌ Не удалось выйти из сна MPU-9250");
        return false;
    }
    delay(10);
    
    // Проверка WHO_AM_I
        //                uint8_t whoami;
        //                if (!_i2cManager->readRegister(MPU9250_ADDR, MPU9250_WHO_AM_I, &whoami, 1)) {
        //                    ESP_LOGE(TAG, "❌ Не удалось прочитать WHO_AM_I");
        //                    return false;
        //                }
                        
        //                if (whoami != 0x71) {
        //                    ESP_LOGE(TAG, "❌ WHO_AM_I mismatch: ожидал 0x71, получил 0x%02X", whoami);
        //                    return false;
        //                }
        //                ESP_LOGI(TAG, "✅ MPU-9250 обнаружен (WHO_AM_I=0x%02X)", whoami);

    // Проверка WHO_AM_I
    uint8_t whoami;
    if (!_i2cManager->readRegister(MPU9250_ADDR, MPU9250_WHO_AM_I, &whoami, 1)) {
        ESP_LOGE(TAG, "❌ Не удалось прочитать WHO_AM_I");
        return false;
    }
    
    // 🔑 ИСПРАВЛЕНИЕ: Поддержка MPU-9250 (0x71), MPU-6050 (0x68) и MPU-6000 (0x70)
    if (whoami == 0x71) {
        ESP_LOGI(TAG, "✅ MPU-9250 обнаружен (WHO_AM_I=0x71). Магнитометр доступен.");
        _hasMagnetometer = true;
    } else if (whoami == 0x68 || whoami == 0x70) {
        ESP_LOGW(TAG, "⚠️ MPU-6050/6000 обнаружен (WHO_AM_I=0x%02X). Магнитометр ОТСУТСТВУЕТ!", whoami);
        ESP_LOGW(TAG, "   Будет использоваться только акселерометр и гироскоп (6 осей).");
        _hasMagnetometer = false;
    } else {
        ESP_LOGE(TAG, "❌ WHO_AM_I mismatch: ожидал 0x71/0x68/0x70, получил 0x%02X", whoami);
        return false;
    }
                        
    // Настройка DLPF (Digital Low Pass Filter)
    data = 0x03;  // DLPF_CFG=3 → 41Hz bandwidth
    if (!_i2cManager->writeRegister(MPU9250_ADDR, MPU9250_CONFIG, &data, 1)) {
        ESP_LOGE(TAG, "❌ Не удалось настроить DLPF");
        return false;
    }
    
    // Настройка гироскопа: ±250 °/s
    data = 0x00;  // GYRO_FS_SEL=0 → ±250°/s
    if (!_i2cManager->writeRegister(MPU9250_ADDR, MPU9250_GYRO_CONFIG, &data, 1)) {
        ESP_LOGE(TAG, "❌ Не удалось настроить гироскоп");
        return false;
    }
    
    // Настройка акселерометра: ±2g
    data = 0x00;  // ACCEL_FS_SEL=0 → ±2g
    if (!_i2cManager->writeRegister(MPU9250_ADDR, MPU9250_ACCEL_CONFIG, &data, 1)) {
        ESP_LOGE(TAG, "❌ Не удалось настроить акселерометр");
        return false;
    }
    
    ESP_LOGI(TAG, "✅ MPU-9250 настроен: ±250°/s, ±2g, DLPF=41Hz");
    return true;
}

// ============================================================================
// Включение I2C Bypass для доступа к магнитометру
// ============================================================================
bool GY91Handler::_enableI2CBypass() {
    uint8_t data = 0x02;  // Bit 1: BYPASS_EN
    if (!_i2cManager->writeRegister(MPU9250_ADDR, MPU9250_INT_PIN_CFG, &data, 1)) {
        ESP_LOGE(TAG, "❌ Не удалось включить I2C Bypass");
        return false;
    }
    delay(10);
    ESP_LOGI(TAG, "✅ I2C Bypass включён");
    return true;
}

// ============================================================================
// Инициализация магнитометра AK8963
// ============================================================================
bool GY91Handler::_initAK8963() {
    // Проверка WHO_AM_I
    uint8_t whoami;
    if (!_i2cManager->readRegister(AK8963_ADDR, AK8963_WIA, &whoami, 1)) {
        ESP_LOGE(TAG, "❌ Не удалось прочитать AK8963 WHO_AM_I");
        return false;
    }
    
    if (whoami != 0x48) {
        ESP_LOGE(TAG, "❌ AK8963 WHO_AM_I mismatch: ожидал 0x48, получил 0x%02X", whoami);
        return false;
    }
    ESP_LOGI(TAG, "✅ AK8963 обнаружен (WHO_AM_I=0x%02X)", whoami);
    
    // Перевод в Power-Down режим
    uint8_t data = 0x00;
    _i2cManager->writeRegister(AK8963_ADDR, AK8963_CNTL1, &data, 1);
    delay(10);
    
    // Чтение калибровочных коэффициентов (ASA)
    data = 0x0F;  // Fuse ROM access
    _i2cManager->writeRegister(AK8963_ADDR, AK8963_CNTL1, &data, 1);
    delay(10);
    
    uint8_t asa[3];
    if (!_i2cManager->readRegister(AK8963_ADDR, AK8963_ASAX, asa, 3)) {
        ESP_LOGE(TAG, "❌ Не удалось прочитать ASA коэффициенты");
        return false;
    }
    
    // Расчёт чувствительности
    for (int i = 0; i < 3; i++) {
        _magSensitivity[i] = MAG_SCALE * ((asa[i] - 128.0f) * 0.5f / 128.0f + 1.0f);
    }
    ESP_LOGI(TAG, "   📊 ASA: [%d,%d,%d] → Sens: [%.3f,%.3f,%.3f] мкТ/LSB",
             asa[0], asa[1], asa[2],
             _magSensitivity[0], _magSensitivity[1], _magSensitivity[2]);
    
    // Возврат в Power-Down
    data = 0x00;
    _i2cManager->writeRegister(AK8963_ADDR, AK8963_CNTL1, &data, 1);
    delay(10);
    
    // Установка режима: 16-bit output, 100Hz ODR
    data = 0x16;  // 0b00010110
    if (!_i2cManager->writeRegister(AK8963_ADDR, AK8963_CNTL1, &data, 1)) {
        ESP_LOGE(TAG, "❌ Не удалось установить режим AK8963");
        return false;
    }
    
    ESP_LOGI(TAG, "✅ AK8963 инициализирован: 16-bit, 100Hz");
    return true;
}

// ============================================================================
// Инициализация BMP280
// ============================================================================
bool GY91Handler::_initBMP280() {
    // Проверка Chip ID
    uint8_t chip_id;
    if (!_i2cManager->readRegister(BMP280_ADDR, BMP280_CHIP_ID, &chip_id, 1)) {
        ESP_LOGE(TAG, "❌ Не удалось прочитать BMP280 Chip ID");
        return false;
    }
    
    if (chip_id != 0x58) {
        ESP_LOGE(TAG, "❌ BMP280 Chip ID mismatch: ожидал 0x58, получил 0x%02X", chip_id);
        return false;
    }
    ESP_LOGI(TAG, "✅ BMP280 обнаружен (Chip ID=0x%02X)", chip_id);
    
    // Чтение калибровочных коэффициентов
    if (!_readBMP280Calibration()) {
        ESP_LOGE(TAG, "❌ Не удалось прочитать калибровку BMP280");
        return false;
    }
    
    // Настройка конфигурации
    uint8_t config = 0x20;  // t_sb=010 (62.5ms), filter=100 (x16)
    if (!_i2cManager->writeRegister(BMP280_ADDR, BMP280_CONFIG, &config, 1)) {
        ESP_LOGE(TAG, "❌ Не удалось настроить BMP280 CONFIG");
        return false;
    }
    
    // Настройка режима измерения
    uint8_t ctrl_meas = 0x27;  // osrs_t=010 (x2), osrs_p=101 (x16), mode=11 (normal)
    if (!_i2cManager->writeRegister(BMP280_ADDR, BMP280_CTRL_MEAS, &ctrl_meas, 1)) {
        ESP_LOGE(TAG, "❌ Не удалось настроить BMP280 CTRL_MEAS");
        return false;
    }
    
    ESP_LOGI(TAG, "✅ BMP280 настроен: T=x2, P=x16, filter=x16, normal mode");
    return true;
}

// ============================================================================
// Чтение калибровки BMP280
// ============================================================================
bool GY91Handler::_readBMP280Calibration() {
    uint8_t calib[24];
    if (!_i2cManager->readRegister(BMP280_ADDR, BMP280_CALIB_START, calib, 24)) {
        return false;
    }
    
    _bmp280Calib.dig_T1 = (calib[1] << 8) | calib[0];
    _bmp280Calib.dig_T2 = (calib[3] << 8) | calib[2];
    _bmp280Calib.dig_T3 = (calib[5] << 8) | calib[4];
    _bmp280Calib.dig_P1 = (calib[7] << 8) | calib[6];
    _bmp280Calib.dig_P2 = (calib[9] << 8) | calib[8];
    _bmp280Calib.dig_P3 = (calib[11] << 8) | calib[10];
    _bmp280Calib.dig_P4 = (calib[13] << 8) | calib[12];
    _bmp280Calib.dig_P5 = (calib[15] << 8) | calib[14];
    _bmp280Calib.dig_P6 = (calib[17] << 8) | calib[16];
    _bmp280Calib.dig_P7 = (calib[19] << 8) | calib[18];
    _bmp280Calib.dig_P8 = (calib[21] << 8) | calib[20];
    _bmp280Calib.dig_P9 = (calib[23] << 8) | calib[22];
    
    ESP_LOGD(TAG, "📊 BMP280 калибровка: T1=%u, T2=%d, T3=%d", 
             _bmp280Calib.dig_T1, _bmp280Calib.dig_T2, _bmp280Calib.dig_T3);
    return true;
}

// ============================================================================
// Основной цикл обновления
// ============================================================================
void GY91Handler::update() {
    if (!_initialized) return;
    
    uint32_t now = micros();
    uint32_t elapsed = now - _lastUpdate;
    
    // 🔑 ОПТИМИЗАЦИЯ: Читаем данные не чаще чем раз в 4 мс (250 Гц)
    // Это защищает от избыточных I2C-транзакций, если loop() вызывается слишком часто
    if (elapsed < 4000) return;  // 4000 мкс = 4 мс = 250 Гц
    
    float dt = elapsed * 1e-6f;
    
    // Защита от аномальных dt (например, если был watchdog reset)
    if (dt > 0.1f) dt = 0.004f;
    
    _lastUpdate = now;
    
    // ========================================================================
    // 1. Обновление IMU данных (MPU-9250)
    // ========================================================================
    _updateIMUData();
    
    // ========================================================================
    // 2. Обновление барометра (BMP280)
    // ========================================================================
    // 🔑 Барометр обновляем реже (50 Гц), т.к. давление меняется медленно
    static uint32_t lastBaroUpdate = 0;
    if (_bmp280Initialized && (now - lastBaroUpdate >= 20000)) {  // 20 мс = 50 Гц
        _updateBaroData();
        lastBaroUpdate = now;
    }
}


/* Код Устаревший, может перегрузить прос
void GY91Handler::update() {
    if (!_initialized) return;
    
    uint32_t now = micros();
    float dt = (now - _lastUpdate) * 1e-6f;
    if (dt < 0.004f || dt > 0.1f) dt = 0.004f;
    _lastUpdate = now;
    
    // ========================================================================
    // 1. Обновление IMU данных
    // ========================================================================
    _updateIMUData();
    
    // ========================================================================
    // 2. Обновление барометра
    // ========================================================================
    if (_bmp280Initialized) {
        _updateBaroData();
    }
}
*/

// ============================================================================
// Обновление IMU данных
// ============================================================================
void GY91Handler::_updateIMUData() {
    int16_t rawAccel[3], rawGyro[3], rawMag[3];
    int16_t rawTemp = 0;
    
    if (!_readMPU9250Data(rawAccel, rawGyro, rawTemp, rawMag)) {
        ESP_LOGW(TAG, "⚠️ Сбой чтения MPU-9250 данных");
        return;
    }
    
    // Калибровка гироскопа (первые 200 семплов)
    if (!_imuCalibrated && _calibrationCount < 200) {
        _gyroOffsetX += rawGyro[0];
        _gyroOffsetY += rawGyro[1];
        _gyroOffsetZ += rawGyro[2];
        _calibrationCount++;
        
        if (_calibrationCount == 200) {
            _gyroOffsetX /= 200.0f;
            _gyroOffsetY /= 200.0f;
            _gyroOffsetZ /= 200.0f;
            _imuCalibrated = true;
            _sensorData.status |= 0x02;
            ESP_LOGI(TAG, "✅ Калибровка гироскопа завершена");
            ESP_LOGI(TAG, "   Offsets: X=%.1f, Y=%.1f, Z=%.1f", 
                     _gyroOffsetX, _gyroOffsetY, _gyroOffsetZ);
        }
        return;
    }
    
    // Масштабирование данных
    _sensorData.accel.x = rawAccel[0] * ACCEL_SCALE_2G;
    _sensorData.accel.y = rawAccel[1] * ACCEL_SCALE_2G;
    _sensorData.accel.z = rawAccel[2] * ACCEL_SCALE_2G;
    
    _sensorData.gyro.x = (rawGyro[0] - _gyroOffsetX) * GYRO_SCALE_250DPS;
    _sensorData.gyro.y = (rawGyro[1] - _gyroOffsetY) * GYRO_SCALE_250DPS;
    _sensorData.gyro.z = (rawGyro[2] - _gyroOffsetZ) * GYRO_SCALE_250DPS;
    
    _sensorData.mag.x = rawMag[0] * _magSensitivity[0];
    _sensorData.mag.y = rawMag[1] * _magSensitivity[1];
    _sensorData.mag.z = rawMag[2] * _magSensitivity[2];
    
    _sensorData.temperature = (rawTemp / 333.87f) + 21.0f;
    
    // ========================================================================
    // Фильтрация через FilterManager
    // ========================================================================
    float ax = _sensorData.accel.x;
    float ay = _sensorData.accel.y;
    float az = _sensorData.accel.z;
    float gx = _sensorData.gyro.x;
    float gy = _sensorData.gyro.y;
    float gz = _sensorData.gyro.z;
    float mx = _sensorData.mag.x;
    float my = _sensorData.mag.y;
    float mz = _sensorData.mag.z;
    
    float dt = 0.004f;  // 250 Hz
    _filterManager.update(dt, ax, ay, az, gx, gy, gz, mx, my, mz);
    _filterManager.getEuler(_sensorData.roll, _sensorData.pitch, _sensorData.yaw);
    
    _sensorData.status |= 0x01;
}

// ============================================================================
// Чтение сырых данных MPU-9250
// ============================================================================
bool GY91Handler::_readMPU9250Data(int16_t accel[3], int16_t gyro[3], int16_t temp, int16_t mag[3]) {
    // 1. Чтение акселерометра + гироскопа + температуры (14 байт)
    uint8_t buffer[14];
    if (!_i2cManager->readRegister(MPU9250_ADDR, MPU9250_ACCEL_XOUT_H, buffer, 14)) {
        ESP_LOGW(TAG, "⚠️ Сбой чтения MPU-9250 данных");
        return false;
    }
    
    accel[0] = (buffer[0] << 8) | buffer[1];
    accel[1] = (buffer[2] << 8) | buffer[3];
    accel[2] = (buffer[4] << 8) | buffer[5];
    temp     = (buffer[6] << 8) | buffer[7];
    gyro[0]  = (buffer[8] << 8) | buffer[9];
    gyro[1]  = (buffer[10] << 8) | buffer[11];
    gyro[2]  = (buffer[12] << 8) | buffer[13];

    // 🔑 ИСПРАВЛЕНИЕ: Чтение магнитометра ТОЛЬКО если он физически присутствует
    if (!_hasMagnetometer) {
        mag[0] = mag[1] = mag[2] = 0;
        return true; // Успешно прочитали только IMU (6 осей), магнитометр пропущен
    }

    // 2. Чтение магнитометра (7 байт: ST1 + 6 байт данных + ST2)
    uint8_t magBuffer[7];
    if (!_hasMagnetometer) {
        if (!_i2cManager->readRegister(AK8963_ADDR, AK8963_ST1, magBuffer, 7)) {
            mag[0] = mag[1] = mag[2] = 0;
            return true; // Не фатально
        }
        //
        
        if ((magBuffer[0] & 0x01) == 0 || (magBuffer[0] & 0x08) != 0) {
            mag[0] = mag[1] = mag[2] = 0;
            return true;
        }
    }//
    
    mag[0] = (int16_t)((magBuffer[2] << 8) | magBuffer[1]);
    mag[1] = (int16_t)((magBuffer[4] << 8) | magBuffer[3]);
    mag[2] = (int16_t)((magBuffer[6] << 8) | magBuffer[5]);
    
    return true;
}

// ============================================================================
// Обновление данных барометра
// ============================================================================
void GY91Handler::_updateBaroData() {
    if (!_readBMP280Data()) {
        ESP_LOGW(TAG, "⚠️ Сбой чтения BMP280 данных");
        return;
    }
    
    // Расчёт высоты
    _altitude = _calculateAltitude(_pressure);
    
    // Обновление SensorData
    _sensorData.altitude = _altitude;
    _sensorData.pressure = _pressure;
}

// ============================================================================
// Чтение сырых данных BMP280
// ============================================================================
bool GY91Handler::_readBMP280Data() {
    uint8_t buffer[6];
    if (!_i2cManager->readRegister(BMP280_ADDR, BMP280_PRESS_MSB, buffer, 6)) {
        return false;
    }
    
    int32_t adc_P = ((buffer[0] << 16) | (buffer[1] << 8) | buffer[2]) >> 4;
    int32_t adc_T = ((buffer[3] << 16) | (buffer[4] << 8) | buffer[5]) >> 4;
    
    // Компенсация температуры
    int32_t T = _bmp280_compensate_T(adc_T);
    _baroTemp = T / 100.0f;
    
    // Компенсация давления
    uint32_t P = _bmp280_compensate_P(adc_P);
    _pressure = P / 100.0f;  // Па → гПа
    
    return true;
}

// ============================================================================
// Компенсация температуры BMP280
// ============================================================================
int32_t GY91Handler::_bmp280_compensate_T(int32_t adc_T) {
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)_bmp280Calib.dig_T1 << 1))) * 
            ((int32_t)_bmp280Calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)_bmp280Calib.dig_T1)) * 
              ((adc_T >> 4) - ((int32_t)_bmp280Calib.dig_T1))) >> 12) * 
            ((int32_t)_bmp280Calib.dig_T3)) >> 14;
    _bmp280_t_fine = var1 + var2;
    T = (_bmp280_t_fine * 5 + 128) >> 8;
    return T;
}

// ============================================================================
// Компенсация давления BMP280
// ============================================================================
uint32_t GY91Handler::_bmp280_compensate_P(int32_t adc_P) {
    int64_t var1, var2, p;
    var1 = ((int64_t)_bmp280_t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)_bmp280Calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)_bmp280Calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)_bmp280Calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)_bmp280Calib.dig_P3) >> 8) + 
           ((var1 * (int64_t)_bmp280Calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)_bmp280Calib.dig_P1) >> 33;
    
    if (var1 == 0) return 0;
    
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)_bmp280Calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)_bmp280Calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)_bmp280Calib.dig_P7) << 4);
    
    return (uint32_t)p;
}

// ============================================================================
// Расчёт высоты по барометрической формуле
// ============================================================================
float GY91Handler::_calculateAltitude(float pressure) {
    // Барометрическая формула: h = 44330 * (1 - (P/P0)^0.1903)
    return 44330.0f * (1.0f - powf(pressure / _seaLevelPressure, 0.1903f));
}

// ============================================================================
// Калибровка барометра
// ============================================================================
void GY91Handler::calibrateBarometer() {
    if (!_bmp280Initialized) {
        ESP_LOGW(TAG, "⚠️ BMP280 не инициализирован, калибровка невозможна");
        return;
    }
    
    // Считываем текущее давление как уровень моря
    _readBMP280Data();
    _seaLevelPressure = _pressure;
    
    ESP_LOGI(TAG, "✅ Барометр откалиброван");
    ESP_LOGI(TAG, "   Опорное давление: %.2f гПа", _seaLevelPressure);
    ESP_LOGI(TAG, "   Температура: %.1f°C", _baroTemp);
}
