/**
 * @file i2c_master.h
 * @brief Нативный контроллер I2C шины для ESP32-S3 (ESP-IDF 5.0+)
 * @details
 * - Полная замена Wire.h для максимальной производительности и надёжности
 * - Поддержка аппаратного Flow Control и таймаутов
 * - Интеграция с ESP_LOG для отладки
 *
 * @environment ESP32-S3-N16R8 | PlatformIO | ESP-IDF 5.0+
 * @version 1.0 (миграция из Arduino IDE)
 * @date 2026-05-26
 */

#pragma once

#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_err.h>
#include <cstdint>

#include "../../config/pins.h"

// ============================================================================
// Конфигурация I2C
// ============================================================================
#define I2C_MASTER_NUM          I2C_NUM_0           ///< Порт I2C (0 или 1)
#define I2C_MASTER_FREQ_HZ      400000              ///< Частота шины: 400 кГц (Fast Mode)
#define I2C_MASTER_TX_BUF_LEN   0                   ///< Буфер передачи: отключён
#define I2C_MASTER_RX_BUF_LEN   0                   ///< Буфер приёма: отключён
#define I2C_MASTER_TIMEOUT_MS   1000                ///< Таймаут операций (мс)

// ============================================================================
// Класс нативного I2C контроллера
// ============================================================================
class I2CMasterController {
public:
    /**
     * @brief Инициализация I2C шины
     * @return true при успешной инициализации
     * @details
     * - Настраивает пины SDA/SCL из Config::Pins
     * - Включает pull-up резисторы
     * - Устанавливает частоту 400 кГц
     */
    bool begin();

    /**
     * @brief Сканирование устройств на шине I2C
     * @details Выводит список всех обнаруженных устройств через ESP_LOG
     */
    void scanDevices();

    /**
     * @brief Проверка подключения устройства по адресу
     * @param address 7-bit I2C адрес устройства
     * @return true если устройство отвечает
     */
    bool isDeviceConnected(uint8_t address);

    /**
     * @brief Чтение данных из регистра устройства
     * @param dev_addr Адрес устройства (7-bit)
     * @param reg_addr Адрес регистра внутри устройства
     * @param data Буфер для чтения данных
     * @param len Количество байт для чтения
     * @return true при успехе
     */
    bool readRegister(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, size_t len);

    /**
     * @brief Запись данных в регистр устройства
     * @param dev_addr Адрес устройства (7-bit)
     * @param reg_addr Адрес регистра внутри устройства
     * @param data Буфер с данными для записи
     * @param len Количество байт для записи
     * @return true при успехе
     */
    bool writeRegister(uint8_t dev_addr, uint8_t reg_addr, const uint8_t* data, size_t len);

    /**
     * @brief Универсальная передача данных (burst read/write)
     * @param dev_addr Адрес устройства
     * @param write_buffer Буфер для записи (может быть nullptr)
     * @param write_size Размер буфера записи
     * @param read_buffer Буфер для чтения (может быть nullptr)
     * @param read_size Размер буфера чтения
     * @return true при успехе
     */
    bool transfer(uint8_t dev_addr,
                  const uint8_t* write_buffer, size_t write_size,
                  uint8_t* read_buffer, size_t read_size);

    /**
     * @brief Проверка успешности инициализации
     * @return true если begin() был успешно выполнен
     */
    bool isInitialized() const { return _initialized; }

private:
    static const char* TAG;          ///< Тег для ESP_LOG
    bool _initialized = false;       ///< Флаг инициализации

    /**
     * @brief Внутренняя функция выполнения I2C команды с таймаутом
     * @param cmd_handle Обработчик команды I2C
     * @return esp_err_t результат выполнения
     */
    esp_err_t _i2c_master_cmd_begin(i2c_cmd_handle_t cmd_handle);
};
