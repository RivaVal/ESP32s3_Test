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
