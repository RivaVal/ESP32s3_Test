


// 📄 **Файл:** `MPU9250_Handler.cpp` (фрагменты, которые нужно добавить/заменить)
// ⚙️ **Функция:** `bool MPU9250Handler::begin(I2CMasterController& i2c_manager)`
// 💻 **Код:**
// cpp
//
#include "modules/imu_handler/MPU9250_Handler.h"
// #include "MPU9250_Handler.h"
#include <cmath>

// Константы регистров MPU-9250
#define MPU6500_ADDR        0x68
#define AK8963_ADDR         0x0C
#define PWR_MGMT_1          0x6B
#define INT_PIN_CFG         0x37
#define I2C_BYPASS_EN       0x02  // Bit 1
#define AK8963_WIA          0x00  // Who Am I (0x48)
#define AK8963_CNTL1        0x0A  // Control 1
#define AK8963_ASAX         0x10  // Sensitivity Adjustment

static const char* TAG_MPU = "MPU9250";

MPU9250Handler::MPU9250Handler() : _i2cManager(nullptr), _initialized(false), _gyroCalibrated(false), _calibrationCount(0), _lastUpdate(0) {
    memset(&_sensorData, 0, sizeof(_sensorData));
}

bool MPU9250Handler::begin(I2CMasterController& i2c_manager) {
    ESP_LOGI(TAG_MPU, "=== 🚀 ИНИЦИАЛИЗАЦИЯ MPU-9250 (9-AXIS) ===");
    _i2cManager = &i2c_manager;

    if (!_initMPU6500()) {
        ESP_LOGE(TAG_MPU, "❌ Ошибка инициализации MPU6500");
        return false;
    }
    if (!_enableI2CBypass()) {
        ESP_LOGE(TAG_MPU, "❌ Ошибка включения I2C Bypass для AK8963");
        return false;
    }
    if (!_initAK8963()) {
        ESP_LOGE(TAG_MPU, "❌ Ошибка инициализации магнитометра AK8963");
        return false;
    }

    _initialized = true;
    ESP_LOGI(TAG_MPU, "✅ MPU-9250 инициализирован. Калибровка гироскопа запущена...");
    return true;
}


// ⚙️ **Функция:** `bool MPU9250Handler::updateSensors()`
// 💻 **Код:**
// 
bool MPU9250Handler::updateSensors() {
    if (!_initialized || !_i2cManager) return false;

    uint32_t now = micros(); // 🔑 Высокоточный таймер
    float dt = (now - _lastUpdate) * 1e-6f;
    if (dt < 0.004f || dt > 0.1f) dt = 0.004f; // Защита от джиттера (min 4ms / 250Hz)
    _lastUpdate = now;

    int16_t rawAccel[3], rawGyro[3], rawMag[3], rawTemp = 0;
    if (!_readRawData(rawAccel, rawGyro, rawTemp, rawMag)) {
        ESP_LOGW(TAG_MPU, "⚠️ Сбой чтения данных");
        return false;
    }

    // Калибровка гироскопа (пропускаем первые 200 семплов)
    if (!_gyroCalibrated && _calibrationCount < 200) {
        _gyroOffsetX += rawGyro[0]; _gyroOffsetY += rawGyro[1]; _gyroOffsetZ += rawGyro[2];
        _calibrationCount++;
        if (_calibrationCount == 200) {
            _gyroOffsetX /= 200.0f; _gyroOffsetY /= 200.0f; _gyroOffsetZ /= 200.0f;
            _gyroCalibrated = true;
            _sensorData.status |= 0x03;
            ESP_LOGI(TAG_MPU, "✅ Калибровка гироскопа завершена");
        }
        return true;
    }

    // Масштабирование
    constexpr float ACCEL_SCALE = 9.80665f / 16384.0f;
    constexpr float GYRO_SCALE  = (250.0f / 32768.0f) * (M_PI / 180.0f);
    _sensorData.accel.x = rawAccel[0] * ACCEL_SCALE;
    _sensorData.accel.y = rawAccel[1] * ACCEL_SCALE;
    _sensorData.accel.z = rawAccel[2] * ACCEL_SCALE;
    _sensorData.gyro.x  = (rawGyro[0] - _gyroOffsetX) * GYRO_SCALE;
    _sensorData.gyro.y  = (rawGyro[1] - _gyroOffsetY) * GYRO_SCALE;
    _sensorData.gyro.z  = (rawGyro[2] - _gyroOffsetZ) * GYRO_SCALE;
    _sensorData.mag.x   = rawMag[0] * _magSensitivity[0];
    _sensorData.mag.y   = rawMag[1] * _magSensitivity[1];
    _sensorData.mag.z   = rawMag[2] * _magSensitivity[2];
    _sensorData.temperature = (rawTemp / 333.87f) + 21.0f;

    // 🔑 ИНТЕГРАЦИЯ АДАПТИВНОГО ФИЛЬТРА
    // Извлекаем данные в локальные переменные для передачи в FilterManager
    float ax = _sensorData.accel.x, ay = _sensorData.accel.y, az = _sensorData.accel.z;
    float gx = _sensorData.gyro.x, gy = _sensorData.gyro.y, gz = _sensorData.gyro.z;
    float mx = _sensorData.mag.x, my = _sensorData.mag.y, mz = _sensorData.mag.z;

    filterManager.update(dt, ax, ay, az, gx, gy, gz, mx, my, mz);
    filterManager.getEuler(_sensorData.roll, _sensorData.pitch, _sensorData.yaw);

    _sensorData.status |= 0x01;
    return true;
}

void MPU9250Handler::_applyComplementaryFilter(float dt) {
    if (dt <= 0.0f) dt = 0.01f;
    const float ALPHA = 0.98f; // Коэффициент фильтра (оптимально для 100Гц)

    // Интегрирование гироскопа (градусы)
    float gyroRoll  = _sensorData.roll  + (_sensorData.gyro.x * dt * (180.0f / M_PI));
    float gyroPitch = _sensorData.pitch + (_sensorData.gyro.y * dt * (180.0f / M_PI));

    // Коррекция акселерометром (гравитационный вектор)
    float accRoll  = atan2f(_sensorData.accel.y, _sensorData.accel.z) * (180.0f / M_PI);
    float accPitch = atan2f(-_sensorData.accel.x, 
                           sqrtf(_sensorData.accel.y * _sensorData.accel.y + _sensorData.accel.z * _sensorData.accel.z)) * (180.0f / M_PI);

    // Комплементарный фильтр
    _sensorData.roll  = ALPHA * gyroRoll  + (1.0f - ALPHA) * accRoll;
    _sensorData.pitch = ALPHA * gyroPitch + (1.0f - ALPHA) * accPitch;

    // Yaw из магнитометра (с компенсацией наклона)
    float magX = _sensorData.mag.x * cosf(_sensorData.pitch * M_PI / 180.0f) +
                 _sensorData.mag.z * sinf(_sensorData.pitch * M_PI / 180.0f);
    float magY = _sensorData.mag.x * sinf(_sensorData.roll * M_PI / 180.0f) * sinf(_sensorData.pitch * M_PI / 180.0f) +
                 _sensorData.mag.y * cosf(_sensorData.roll * M_PI / 180.0f) -
                 _sensorData.mag.z * sinf(_sensorData.roll * M_PI / 180.0f) * cosf(_sensorData.pitch * M_PI / 180.0f);
    
    _sensorData.yaw = atan2f(-magY, magX) * (180.0f / M_PI);
    if (_sensorData.yaw < 0) _sensorData.yaw += 360.0f;
}

// ============================================================================
// 🔧 РЕАЛИЗАЦИЯ ПРИВАТНЫХ МЕТОДОВ (заглушки для MVP)
// Файл: MPU9250_Handler.cpp | Позиция: конец файла
// ============================================================================

/**
 * @brief Инициализация MPU6500 (акселерометр+гироскоп)
 * @return true при успехе
 */
bool MPU9250Handler::_initMPU6500() {
    ESP_LOGI(TAG_MPU, "⚙️  _initMPU6500: сброс питания...");
    
    // Сброс через PWR_MGMT_1
    uint8_t data = 0x80;  // Bit 7: DEVICE_RESET
    if (!_i2cManager->writeRegister(MPU6500_ADDR, PWR_MGMT_1, &data, 1)) {
        ESP_LOGE(TAG_MPU, "❌ Не удалось сбросить MPU6500");
        return false;
    }
    delay(100);  // Ждём сброса
    
    // Выход из сна, выбор источника тактирования
    data = 0x01;  // CLKSEL=PLL with X axis gyroscope reference
    if (!_i2cManager->writeRegister(MPU6500_ADDR, PWR_MGMT_1, &data, 1)) {
        ESP_LOGE(TAG_MPU, "❌ Не удалось выйти из сна MPU6500");
        return false;
    }
    delay(10);
    
    // Настройка DLPF (Digital Low Pass Filter)
    data = 0x03;  // DLPF_CFG=3 → 41Hz bandwidth
    if (!_i2cManager->writeRegister(MPU6500_ADDR, 0x1A, &data, 1)) {  // CONFIG reg
        ESP_LOGE(TAG_MPU, "❌ Не удалось настроить DLPF");
        return false;
    }
    
    // Настройка гироскопа: ±250 °/с
    data = 0x00;  // GYRO_FS_SEL=0 → ±250°/s
    if (!_i2cManager->writeRegister(MPU6500_ADDR, 0x1B, &data, 1)) {  // GYRO_CONFIG
        ESP_LOGE(TAG_MPU, "❌ Не удалось настроить гироскоп");
        return false;
    }
    
    // Настройка акселерометра: ±2g
    data = 0x00;  // ACCEL_FS_SEL=0 → ±2g
    if (!_i2cManager->writeRegister(MPU6500_ADDR, 0x1C, &data, 1)) {  // ACCEL_CONFIG
        ESP_LOGE(TAG_MPU, "❌ Не удалось настроить акселерометр");
        return false;
    }
    
    // Проверка WHO_AM_I (должно быть 0x71 для MPU6500)
    uint8_t whoami;
    if (!_i2cManager->readRegister(MPU6500_ADDR, 0x75, &whoami, 1)) {
        ESP_LOGE(TAG_MPU, "❌ Не удалось прочитать WHO_AM_I");
        return false;
    }
    if (whoami != 0x71) {
        ESP_LOGE(TAG_MPU, "❌ WHO_AM_I mismatch: ожидал 0x71, получил 0x%02X", whoami);
        return false;
    }
    
    ESP_LOGI(TAG_MPU, "✅ MPU6500 инициализирован (WHO_AM_I=0x%02X)", whoami);
    return true;
}

/**
 * @brief Включение I2C Bypass для доступа к магнитометру AK8963
 * @return true при успехе
 */
bool MPU9250Handler::_enableI2CBypass() {
    ESP_LOGI(TAG_MPU, "⚙️  _enableI2CBypass: включение bypass...");
    
    // INT_PIN_CFG: бит 1 (BYPASS_EN) = 1, бит 4 (INT_LEVEL) = 0 (active high)
    uint8_t data = 0x12;  // 0b00010010: BYPASS_EN + INT_RD_CLEAR
    if (!_i2cManager->writeRegister(MPU6500_ADDR, INT_PIN_CFG, &data, 1)) {
        ESP_LOGE(TAG_MPU, "❌ Не удалось включить I2C Bypass");
        return false;
    }
    delay(10);
    
    // Проверка: чтение из AK8963 должно работать
    uint8_t whoami;
    if (!_i2cManager->readRegister(AK8963_ADDR, AK8963_WIA, &whoami, 1)) {
        ESP_LOGE(TAG_MPU, "❌ Не удалось прочитать AK8963 WHO_AM_I");
        return false;
    }
    if (whoami != 0x48) {
        ESP_LOGE(TAG_MPU, "❌ AK8963 WHO_AM_I mismatch: ожидал 0x48, получил 0x%02X", whoami);
        return false;
    }
    
    ESP_LOGI(TAG_MPU, "✅ I2C Bypass включён, AK8963 обнаружен (0x%02X)", whoami);
    return true;
}

/**
 * @brief Инициализация магнитометра AK8963
 * @return true при успехе
 */
bool MPU9250Handler::_initAK8963() {
    ESP_LOGI(TAG_MPU, "⚙️  _initAK8963: настройка магнитометра...");
    
    // Сначала перевести в Power-Down режим
    uint8_t data = 0x00;
    if (!_i2cManager->writeRegister(AK8963_ADDR, AK8963_CNTL1, &data, 1)) {
        ESP_LOGE(TAG_MPU, "❌ Не удалось перевести AK8963 в Power-Down");
        return false;
    }
    delay(10);
    
    // Чтение калибровочных коэффициентов (ASA)
    // Режим: Fuse ROM access (0x0F)
    data = 0x0F;
    if (!_i2cManager->writeRegister(AK8963_ADDR, AK8963_CNTL1, &data, 1)) {
        ESP_LOGE(TAG_MPU, "❌ Не удалось войти в режим Fuse ROM");
        return false;
    }
    delay(10);
    
    // Чтение чувствительности (0x10-0x12)
    uint8_t asa[3];
    if (!_i2cManager->readRegister(AK8963_ADDR, AK8963_ASAX, asa, 3)) {
        ESP_LOGE(TAG_MPU, "❌ Не удалось прочитать ASA коэффициенты");
        return false;
    }
    
    // Расчёт чувствительности: 0.6 uT/LSB * ((ASA-128)*0.5/128 + 1)
    for (int i = 0; i < 3; i++) {
        _magSensitivity[i] = 0.6f * ((asa[i] - 128.0f) * 0.5f / 128.0f + 1.0f);
    }
    ESP_LOGI(TAG_MPU, "   📊 ASA: [%d,%d,%d] → Sens: [%.3f,%.3f,%.3f] uT/LSB",
             asa[0], asa[1], asa[2],
             _magSensitivity[0], _magSensitivity[1], _magSensitivity[2]);
    
    // Возврат в Power-Down
    data = 0x00;
    _i2cManager->writeRegister(AK8963_ADDR, AK8963_CNTL1, &data, 1);
    delay(10);
    
    // Установка режима: 16-bit output, 100Hz ODR
    data = 0x16;  // 0b00010110: 16-bit + 100Hz
    if (!_i2cManager->writeRegister(AK8963_ADDR, AK8963_CNTL1, &data, 1)) {
        ESP_LOGE(TAG_MPU, "❌ Не удалось установить режим AK8963");
        return false;
    }
    
    ESP_LOGI(TAG_MPU, "✅ AK8963 инициализирован: 16-bit, 100Hz");
    return true;
}

/**
 * @brief Чтение сырых данных со всех сенсоров
 * @param accel[out] Массив [3] для акселерометра
 * @param gyro[out]  Массив [3] для гироскопа  
 * @param temp[out]  Температура
 * @param mag[out]   Массив [3] для магнитометра
 * @return true при успехе
 */
// MPU9250_Handler.cpp — функция _readRawData
// 🔧 Добавьте (void)temp для подавления предупреждения:
bool MPU9250Handler::_readRawData(int16_t accel[3], int16_t gyro[3], int16_t temp, int16_t mag[3]) {
    (void)temp;  // 🔧 Подавляем предупреждение: параметр пока не используется
    // Чтение акселерометра+гироскопа+температуры (14 регистров подряд)
    // ACCEL_XOUT_H = 0x3B, всего 14 байт
    uint8_t buffer[14];
    if (!_i2cManager->readRegister(MPU6500_ADDR, 0x3B, buffer, 14)) {
        ESP_LOGW(TAG_MPU, "⚠️ Сбой чтения MPU6500 данных");
        return false;
    }
    
    // Декодирование (big-endian)
    accel[0] = (buffer[0] << 8) | buffer[1];   // ACCEL_X
    accel[1] = (buffer[2] << 8) | buffer[3];   // ACCEL_Y
    accel[2] = (buffer[4] << 8) | buffer[5];   // ACCEL_Z
    temp     = (buffer[6] << 8) | buffer[7];   // TEMP
    gyro[0]  = (buffer[8] << 8) | buffer[9];   // GYRO_X
    gyro[1]  = (buffer[10] << 8) | buffer[11]; // GYRO_Y
    gyro[2]  = (buffer[12] << 8) | buffer[13]; // GYRO_Z
    
    // Чтение магнитометра (7 регистров: ST1 + 6 байт данных + ST2)
    // ST1 регистр = 0x02, проверка бит 0 (DRDY)
    uint8_t magBuffer[7];
    if (!_i2cManager->readRegister(AK8963_ADDR, 0x02, magBuffer, 7)) {
        ESP_LOGW(TAG_MPU, "⚠️ Сбой чтения AK8963 данных");
        // Не фатально: продолжаем без магнитометра
        mag[0] = mag[1] = mag[2] = 0;
        return true;
    }
    
    // Проверка: бит 0 = 1 (данные готовы), бит 3 = 0 (нет ошибки)
    if ((magBuffer[0] & 0x01) == 0 || (magBuffer[0] & 0x08) != 0) {
        ESP_LOGW(TAG_MPU, "⚠️ AK8963: данные не готовы или ошибка");
        mag[0] = mag[1] = mag[2] = 0;
        return true;  // Не фатально
    }
    
    // Декодирование магнитометра (little-endian!)
    mag[0] = (int16_t)((magBuffer[2] << 8) | magBuffer[1]);  // MAG_X
    mag[1] = (int16_t)((magBuffer[4] << 8) | magBuffer[3]);  // MAG_Y
    mag[2] = (int16_t)((magBuffer[6] << 8) | magBuffer[5]);  // MAG_Z
    
    return true;
}
