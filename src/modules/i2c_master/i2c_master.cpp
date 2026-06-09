/**
 * @file i2c_master.cpp
 * @brief Реализация нативного I2C контроллера для ESP32-S3
 * @details
 * - Использует driver/i2c.h из ESP-IDF 5.0+
 * - Все операции с подробной отладкой через ESP_LOG
 * - Потокобезопасность через мьютексы (при необходимости)
 *
 * @version 1.0 (миграция из Arduino IDE)
 * @date 2026-05-26
 */

#include "i2c_master.h"
#include <cstring>  // для memset

// ============================================================================
// Статический тег для отладки
// ============================================================================
// const char* TAG = "I2C_MASTER";
const char* I2CMasterController::TAG = "I2C_MASTER";

// ============================================================================
// Реализация публичного API
// ============================================================================

bool I2CMasterController::begin() {
    ESP_LOGI(TAG, "=== 🚀 ИНИЦИАЛИЗАЦИЯ НАТИВНОГО I2C MASTER (ESP-IDF 5.0+) ===");
    ESP_LOGI(TAG, "Параметры: SDA=GPIO%d, SCL=GPIO%d, частота=%d Гц, порт=%d",
             Config::Pins::I2C_SDA, Config::Pins::I2C_SCL,
             I2C_MASTER_FREQ_HZ, I2C_MASTER_NUM);

    // Конфигурация I2C
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = static_cast<gpio_num_t>(Config::Pins::I2C_SDA);
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = static_cast<gpio_num_t>(Config::Pins::I2C_SCL);
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    // 🔑 Критично для ESP32-S3: авто-выбор источника тактирования
    conf.clk_flags = 0;  // I2C_SCLK_SRC_FLAG_* = 0 → авто-выбор

    // Применение конфигурации
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ Ошибка конфигурации I2C: %s", esp_err_to_name(err));
        return false;
    }

    // Установка драйвера
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                            I2C_MASTER_RX_BUF_LEN,
                            I2C_MASTER_TX_BUF_LEN, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ Ошибка установки драйвера I2C: %s", esp_err_to_name(err));
        return false;
    }

    _initialized = true;
    ESP_LOGI(TAG, "✅ I2C Master инициализирован успешно");
    return true;
}


/**
 * @brief Расширенное сканирование устройств на шине I2C
 * @details 
 * - Сканирует все адреса от 0x08 до 0x77
 * - Выводит подробную информацию о каждом найденном устройстве
 * - Пытается прочитать идентификационный регистр для неизвестных устройств
 * - Выводит статистику сканирования
 */
void I2CMasterController::scanDevices() {
    if (!_initialized) {
        ESP_LOGW(TAG, "⚠️ Сканирование невозможно: I2C не инициализирован");
        return;
    }
    
    ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║         🔍 ДЕТАЛЬНОЕ СКАНИРОВАНИЕ I2C ШИНЫ                  ║");
    ESP_LOGI(TAG, "║         Диапазон адресов: 0x08 - 0x77 (120 адресов)         ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
    
    uint32_t start_time = millis();
    uint8_t found_count = 0;
    uint8_t unknown_count = 0;
    
    // Структура для хранения информации об устройстве
    struct DeviceInfo {
        uint8_t address;
        const char* name;
        const char* type;
        bool identified;
        uint8_t who_am_i;
    };
    
    // Расширенная база известных I2C устройств
    // Формат: {адрес, "название", "тип"}
    const struct {
        uint8_t address;
        const char* name;
        const char* type;
    } known_devices[] = {
        // === Датчики движения и ориентации ===
        {0x68, "MPU-6050/MPU-9250", "IMU (6/9-axis)"},
        {0x69, "MPU-6050 (ALT)", "IMU (alternate addr)"},
        {0x1E, "HMC5883L", "Magnetometer (3-axis)"},
        {0x0C, "AK8963", "Magnetometer (MPU-9250)"},
        {0x1D, "ADXL345", "Accelerometer (3-axis)"},
        {0x53, "ADXL345 (ALT)", "Accelerometer (ALT addr)"},
        {0x6A, "LSM6DS3", "IMU (6-axis)"},
        {0x6B, "LSM6DS3 (ALT)", "IMU (6-axis ALT)"},
        {0x19, "LIS3DH", "Accelerometer (3-axis)"},
        {0x5D, "LIS3MDL", "Magnetometer (3-axis)"},
        {0x6C, "LSM303D", "IMU (6-axis)"},
        {0x1E, "LSM303DLHC", "IMU (accel+mag)"},
        {0x6B, "LSM9DS1", "IMU (9-axis)"},
        {0x1D, "LSM9DS1 (MAG)", "Magnetometer (9-axis)"},
        
        // === Барометры и датчики давления ===
        {0x76, "BMP180/BMP280/BME280", "Barometer/Temp/Humidity"},
        {0x77, "BMP180/BMP280/BME280 (ALT)", "Barometer/Temp/Humidity"},
        {0x5C, "BME680", "Environmental (Gas)"},
        {0x76, "BMP388", "Barometer (high precision)"},
        {0x5D, "DPS310", "Barometer (Infineon)"},
        {0x60, "MPL3115A2", "Altimeter/Pressure"},
        {0x70, "MS5611", "Barometer (high precision)"},
        {0x76, "MS5607", "Barometer"},
        
        // === Датчики освещенности ===
        {0x29, "TSL2561/TSL2591", "Light Sensor"},
        {0x39, "TSL2561 (ALT)", "Light Sensor"},
        {0x29, "VL6180X", "ToF Distance/Light"},
        {0x44, "SHT31/SHT30", "Temp/Humidity"},
        {0x40, "SHT31 (ALT)", "Temp/Humidity"},
        {0x5F, "SI1145", "UV/IR/Visible Light"},
        {0x39, "VEML6070", "UV Sensor"},
        {0x44, "OPT3001", "Ambient Light Sensor"},
        
        // === Датчики газа и качества воздуха ===
        {0x5A, "CCS811", "eCO2/TVOC Sensor"},
        {0x5B, "CCS811 (ALT)", "eCO2/TVOC Sensor"},
        {0x76, "BME680", "Gas/Pressure/Temp/Humidity"},
        {0x5C, "SGP30", "eCO2/TVOC Sensor"},
        {0x70, "SGP40", "VOC Sensor"},
        
        // === Дисплеи ===
        {0x3C, "SSD1306 OLED", "OLED Display (128x64)"},
        {0x3D, "SSD1306 OLED (ALT)", "OLED Display (ALT addr)"},
        {0x27, "PCF8574/HD44780", "LCD/I2C Expander"},
        {0x3F, "PCF8574A/HD44780", "LCD/I2C Expander (ALT)"},
        {0x20, "PCF8574", "I/O Expander"},
        {0x21, "PCF8574A", "I/O Expander (ALT)"},
        
        // === ШИМ контроллеры ===
        {0x40, "PCA9685", "16-ch PWM/Servo Driver"},
        {0x41, "PCA9685 (A0)", "PWM Driver (addr A0)"},
        {0x42, "PCA9685 (A1)", "PWM Driver (addr A1)"},
        {0x43, "PCA9685 (A2)", "PWM Driver (addr A2)"},
        {0x44, "PCA9685 (A3)", "PWM Driver (addr A3)"},
        {0x45, "PCA9685 (A4)", "PWM Driver (addr A4)"},
        {0x46, "PCA9685 (A5)", "PWM Driver (addr A5)"},
        {0x47, "PCA9685 (A6)", "PWM Driver (addr A6)"},
        
        // === EEPROM ===
        {0x50, "AT24C32/64", "EEPROM (4K/8K)"},
        {0x51, "AT24C32 (A1)", "EEPROM (addr A1)"},
        {0x52, "AT24C32 (A2)", "EEPROM (addr A2)"},
        {0x53, "AT24C32 (A3)", "EEPROM (addr A3)"},
        {0x54, "AT24C32 (A4)", "EEPROM (addr A4)"},
        {0x55, "AT24C32 (A5)", "EEPROM (addr A5)"},
        {0x56, "AT24C32 (A6)", "EEPROM (addr A6)"},
        {0x57, "AT24C32 (A7)", "EEPROM (addr A7)"},
        
        // === Часы реального времени (RTC) ===
        {0x68, "DS3231/DS1307", "RTC (Real Time Clock)"},
        {0x6F, "DS1338", "RTC with Crystal"},
        {0x68, "PCF8563", "RTC (Philips)"},
        
        // === АЦП/ЦАП ===
        {0x48, "ADS1115/ADS1015", "4-ch ADC (16/12-bit)"},
        {0x49, "ADS1115 (ALT)", "ADC (alternate addr)"},
        {0x4A, "ADS1115 (ADDR)", "ADC (addr pin high)"},
        {0x4B, "ADS1115 (ADDR)", "ADC (addr pin SDA)"},
        {0x60, "MCP4725", "12-bit DAC"},
        {0x61, "MCP4725 (A0)", "DAC (addr A0)"},
        {0x62, "MCP4725 (A1)", "DAC (addr A1)"},
        {0x63, "MCP4725 (A2)", "DAC (addr A2)"},
        
        // === Датчики тока/напряжения ===
        {0x40, "INA219", "Current/Power Monitor"},
        {0x41, "INA219 (A0)", "Current Monitor (A0)"},
        {0x44, "INA226", "Current/Power Monitor"},
        {0x45, "INA226 (ALT)", "Current Monitor (ALT)"},
        
        // === Датчики расстояния ===
        {0x29, "VL53L0X", "ToF Laser Ranger"},
        {0x30, "VL53L1X", "ToF Time-of-Flight"},
        {0x57, "TFMini", "LiDAR Distance Sensor"},
        
        // === Температурные датчики ===
        {0x48, "LM75/LM75A", "Temperature Sensor"},
        {0x49, "LM75 (A0)", "Temperature (A0)"},
        {0x4A, "LM75 (A1)", "Temperature (A1)"},
        {0x4B, "LM75 (A2)", "Temperature (A2)"},
        {0x4C, "LM75 (A3)", "Temperature (A3)"},
        {0x4D, "LM75 (A4)", "Temperature (A4)"},
        {0x4E, "LM75 (A5)", "Temperature (A5)"},
        {0x4F, "LM75 (A6)", "Temperature (A6)"},
        {0x48, "TMP102", "Temperature Sensor"},
        {0x49, "TMP102 (ALT)", "Temperature (ALT)"},
        
        // === Компасы ===
        {0x1E, "HMC5883L", "Digital Compass"},
        {0x42, "QMC5883L", "Magnetometer/Compass"},
        {0x0C, "AK8963", "Magnetometer (MPU-9250)"},
        
        // === GPIO расширители ===
        {0x20, "PCF8574/MCP23017", "I/O Expander (8/16-bit)"},
        {0x21, "PCF8574A/MCP23017", "I/O Expander (ALT)"},
        {0x22, "MCP23017 (A1)", "16-bit I/O Expander"},
        {0x23, "MCP23017 (A2)", "16-bit I/O Expander"},
        {0x24, "MCP23017 (A3)", "16-bit I/O Expander"},
        {0x25, "MCP23017 (A4)", "16-bit I/O Expander"},
        {0x26, "MCP23017 (A5)", "16-bit I/O Expander"},
        {0x27, "MCP23017 (A6)", "16-bit I/O Expander"},
        
        // === Мультиплексоры ===
        {0x70, "TCA9548A", "8-ch I2C Multiplexer"},
        {0x71, "TCA9548A (A0)", "I2C Mux (addr A0)"},
        {0x72, "TCA9548A (A1)", "I2C Mux (addr A1)"},
        {0x73, "TCA9548A (A2)", "I2C Mux (addr A2)"},
        {0x74, "TCA9548A (A3)", "I2C Mux (addr A3)"},
        {0x75, "TCA9548A (A4)", "I2C Mux (addr A4)"},
        {0x76, "TCA9548A (A5)", "I2C Mux (addr A5)"},
        {0x77, "TCA9548A (A6)", "I2C Mux (addr A6)"},
        
        // === RFID/NFC ===
        {0x28, "PN532", "NFC/RFID Reader"},
        {0x24, "PN532 (ALT)", "NFC/RFID (alternate)"},
        {0x2A, "MFRC522", "RFID Reader (13.56MHz)"},
        
        // === Аудио ===
        {0x1A, "WM8960", "Audio Codec"},
        {0x1B, "WM8960 (ALT)", "Audio Codec (ALT)"},
        {0x30, "PCM5102", "I2S DAC"},
        
        // === Драйверы моторов ===
        {0x60, "DRV2605", "Haptic Driver"},
        {0x5D, "DRV8830", "DC Motor Driver"},
        {0x60, "DRV8830 (A0)", "Motor Driver (A0)"},
        {0x61, "DRV8830 (A1)", "Motor Driver (A1)"},
        {0x62, "DRV8830 (A2)", "Motor Driver (A2)"},
        {0x63, "DRV8830 (A3)", "Motor Driver (A3)"},
        
        // === Сенсорные контроллеры ===
        {0x5A, "MPR121", "12-key Capacitive Touch"},
        {0x5B, "MPR121 (ALT)", "Touch Sensor (ALT)"},
        {0x5C, "MPR121 (ADDR)", "Touch Sensor (ADDR)"},
        {0x5D, "MPR121 (ADDR)", "Touch Sensor (ADDR)"},
        
        // === Специализированные датчики ===
        {0x40, "HTS221", "Humidity Sensor"},
        {0x5F, "LPS22HB", "Pressure Sensor"},
        {0x6A, "LIS2DH12", "Accelerometer"},
        {0x1E, "LIS2MDL", "Magnetometer"},
        {0x6B, "LSM6DSO", "IMU (6-axis)"},
        {0x5D, "LIS3MDL", "Magnetometer"},
        
        // === Камеры ===
        {0x21, "OV7670", "VGA Camera Module"},
        {0x30, "OV2640", "2MP Camera Module"},
        
        // === Часы и таймеры ===
        {0x6F, "DS1307", "RTC (Real Time Clock)"},
        {0x68, "DS3231", "High Precision RTC"},
    };
    
    const int known_devices_count = sizeof(known_devices) / sizeof(known_devices[0]);
    
    ESP_LOGI(TAG, "┌──────────────────────────────────────────────────────────────┐");
    ESP_LOGI(TAG, "│ Адрес │ Название устройства          │ Тип                      │");
    ESP_LOGI(TAG, "├──────────────────────────────────────────────────────────────┤");
    
    // Сканирование всех адресов
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (isDeviceConnected(addr)) {
            // Пытаемся найти устройство в базе
            bool found = false;
            for (int i = 0; i < known_devices_count; i++) {
                if (known_devices[i].address == addr) {
                    ESP_LOGI(TAG, "│ 0x%02X  │ %-28s │ %-24s │", 
                             addr, known_devices[i].name, known_devices[i].type);
                    found = true;
                    found_count++;
                    break;
                }
            }
            
            // Если устройство не найдено в базе, пытаемся прочитать WHO_AM_I
            if (!found) {
                uint8_t who_am_i = 0;
                bool read_success = false;
                
                // Пробуем прочитать разные регистры WHO_AM_I
                const uint8_t who_am_i_registers[] = {0x00, 0x0F, 0x75, 0xD0, 0xD1, 0xD2};
                for (int i = 0; i < sizeof(who_am_i_registers); i++) {
                    if (readRegister(addr, who_am_i_registers[i], &who_am_i, 1)) {
                        read_success = true;
                        break;
                    }
                }
                
                if (read_success) {
                    ESP_LOGW(TAG, "│ 0x%02X  │ %-28s │ WHO_AM_I: 0x%02X            │", 
                             addr, "Unknown Device", who_am_i);
                } else {
                    ESP_LOGW(TAG, "│ 0x%02X  │ %-28s │ %-24s │", 
                             addr, "Unknown Device", "Cannot identify");
                }
                unknown_count++;
            }
        }
        
        // Небольшая задержка для стабильности шины
        vTaskDelay(1);
    }
    
    uint32_t scan_time = millis() - start_time;
    
    ESP_LOGI(TAG, "└──────────────────────────────────────────────────────────────┘");
    ESP_LOGI(TAG, "📊 СТАТИСТИКА СКАНИРОВАНИЯ:");
    ESP_LOGI(TAG, "   ✅ Найдено устройств: %d", found_count);
    ESP_LOGI(TAG, "   ❓ Неизвестных устройств: %d", unknown_count);
    ESP_LOGI(TAG, "   ⏱️ Время сканирования: %lu мс", scan_time);
    
    if (found_count == 0 && unknown_count == 0) {
        ESP_LOGW(TAG, "⚠️ На шине I2C не обнаружено ни одного устройства!");
        ESP_LOGW(TAG, "💡 Проверьте:");
        ESP_LOGW(TAG, "   • Подключение SDA и SCL");
        ESP_LOGW(TAG, "   • Наличие pull-up резисторов (4.7кΩ)");
        ESP_LOGW(TAG, "   • Питание устройств (3.3V или 5V)");
    } else {
        ESP_LOGI(TAG, "✅ Сканирование завершено успешно!");
    }
}


/*
//==========================================================================  
// Устаревшая элементарная версия сканера для фмзмческого подтверждения 
// Устройств, указанных в config.(DATA)  и найденных на ШИНЕ!! 
void I2CMasterController::scanDevices() {
    if (!_initialized) {
        ESP_LOGW(TAG, "⚠️ Сканирование невозможно: I2C не инициализирован");
        return;
    }

    ESP_LOGI(TAG, "=== 🔍 СКАНИРОВАНИЕ I2C ШИНЫ (адреса 0x08-0x77) ===");
    uint8_t found_count = 0;

    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (isDeviceConnected(addr)) {
            // Определяем имя устройства по адресу
            const char* device_name = "Unknown";
            switch (addr) {
                case 0x40: device_name = "PCA9685 (PWM driver)"; break;
                case 0x68: device_name = "MPU6050/MPU9250 (IMU)"; break;
                case 0x1E: device_name = "HMC5883L (Magnetometer)"; break;
                case 0x76:
                case 0x77: device_name = "BMP180/BMP280 (Barometer)"; break;
                default: break;
            }
            ESP_LOGI(TAG, "✅ Устройство: 0x%02X (%s)", addr, device_name);
            found_count++;
        }
        vTaskDelay(1);  // Небольшая задержка для стабильности
    }

    if (found_count == 0) {
        ESP_LOGW(TAG, "⚠️ На шине I2C не обнаружено устройств");
    } else {
        ESP_LOGI(TAG, "✅ Обнаружено %d устройств(а) на шине I2C", found_count);
    }
}
*/

bool I2CMasterController::isDeviceConnected(uint8_t address) {
    if (!_initialized) return false;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);

    esp_err_t err = _i2c_master_cmd_begin(cmd);
    i2c_cmd_link_delete(cmd);

    if (err == ESP_OK) {
        ESP_LOGD(TAG, "Устройство 0x%02X отвечает", address);
        return true;
    } else {
        ESP_LOGD(TAG, "Устройство 0x%02X не отвечает (%s)",
                 address, esp_err_to_name(err));
        return false;
    }
}

bool I2CMasterController::readRegister(uint8_t dev_addr, uint8_t reg_addr,
                                       uint8_t* data, size_t len) {
    if (!_initialized || !data || len == 0) return false;

    ESP_LOGV(TAG, "Чтение: устройство=0x%02X, регистр=0x%02X, длина=%u",
             dev_addr, reg_addr, len);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Запись адреса регистра
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    // Чтение данных (повторный старт)
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);

    i2c_master_stop(cmd);

    esp_err_t err = _i2c_master_cmd_begin(cmd);
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ Ошибка чтения регистра 0x%02X устройства 0x%02X: %s",
                 reg_addr, dev_addr, esp_err_to_name(err));
        return false;
    }

    // Отладочный вывод прочитанных данных
    ESP_LOGV(TAG, "📥 Данные: [");
    for (size_t i = 0; i < len; i++) {
        ESP_LOGV(TAG, "0x%02X%s", data[i], (i < len - 1) ? " " : "");
    }
    ESP_LOGV(TAG, "]");

    return true;
}

bool I2CMasterController::writeRegister(uint8_t dev_addr, uint8_t reg_addr,
                                        const uint8_t* data, size_t len) {
    if (!_initialized || !data || len == 0) return false;

    ESP_LOGV(TAG, "Запись: устройство=0x%02X, регистр=0x%02X, длина=%u",
             dev_addr, reg_addr, len);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    for (size_t i = 0; i < len; i++) {
        i2c_master_write_byte(cmd, data[i], true);
    }

    i2c_master_stop(cmd);

    esp_err_t err = _i2c_master_cmd_begin(cmd);
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ Ошибка записи в регистр 0x%02X устройства 0x%02X: %s",
                 reg_addr, dev_addr, esp_err_to_name(err));
        return false;
    }

    ESP_LOGV(TAG, "✅ Запись успешна");
    return true;
}

bool I2CMasterController::transfer(uint8_t dev_addr,
                                   const uint8_t* write_buffer, size_t write_size,
                                   uint8_t* read_buffer, size_t read_size) {
    if (!_initialized) return false;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Запись (если есть)
    if (write_size > 0 && write_buffer) {
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
        for (size_t i = 0; i < write_size; i++) {
            i2c_master_write_byte(cmd, write_buffer[i], true);
        }
    }

    // Чтение (если есть)
    if (read_size > 0 && read_buffer) {
        if (write_size > 0) {
            i2c_master_start(cmd);  // Повторный старт
        }
        i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

        if (read_size > 1) {
            i2c_master_read(cmd, read_buffer, read_size - 1, I2C_MASTER_ACK);
        }
        i2c_master_read_byte(cmd, read_buffer + read_size - 1, I2C_MASTER_NACK);
    }

    i2c_master_stop(cmd);

    esp_err_t err = _i2c_master_cmd_begin(cmd);
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ Ошибка передачи с устройством 0x%02X: %s",
                 dev_addr, esp_err_to_name(err));
        return false;
    }

    return true;
}

esp_err_t I2CMasterController::_i2c_master_cmd_begin(i2c_cmd_handle_t cmd_handle) {
    return i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle,
                               pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}
