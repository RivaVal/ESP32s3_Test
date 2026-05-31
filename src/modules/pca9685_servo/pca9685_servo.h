/**
 * @file pca9685_servo.h
 * @brief Контроллер сервоприводов через PCA9685 (Adafruit-style)
 * @details
 * - Управление 7 сервоприводами через один чип PCA9685
 * - Подключение по шине I2C: SDA=GPIO8, SCL=GPIO9 (Config::Pins)
 * - Частота PWM: 50 Гц (стандарт для сервоприводов)
 * - Разрешение: 12 бит (4096 тиков, точность ~0.044°)
 * - Углы безопасности: ±75° макс, буфер ±5° вокруг нейтрали
 * - Предстартовый тест: 3 цикла, все каналы одновременно
 *
 * @environment ESP32-S3-N16R8 | PlatformIO | Arduino Framework
 * @version 1.0 (миграция из Arduino IDE)
 * @date 2026-05-26
 */

#pragma once

#include <Arduino.h>
#include <esp_log.h>
#include <cstdint>

#include "config/pins.h"
#include "common/types.h"
#include "modules/i2c_master/i2c_master.h"

// ============================================================================
// Регистры PCA9685 (совместимость с Adafruit)
// ============================================================================
#define PCA9685_MODE1           0x00    ///< Register: Mode 1
#define PCA9685_MODE2           0x01    ///< Register: Mode 2
#define PCA9685_PRESCALE        0xFE    ///< Register: Prescale for PWM frequency
#define PCA9685_LED0_ON_L       0x06    ///< Register: LED0 On Tick Low Byte
#define PCA9685_ALL_LED_ON_L    0xFA    ///< Register: All LEDs On Tick Low
#define PCA9685_ALL_LED_OFF_L   0xFC    ///< Register: All LEDs Off Tick Low

// ============================================================================
// Битовые маски режимов
// ============================================================================
#define PCA9685_MODE1_RESTART   0x80    ///< Restart bit
#define PCA9685_MODE1_AUTO_INC  0x20    ///< Auto-Increment (КРИТИЧЕСКИ!)
#define PCA9685_MODE1_SLEEP     0x10    ///< Sleep mode
#define PCA9685_MODE2_OUTDRV    0x04    ///< Totem pole output drive (для серво)

// ============================================================================
// Константы PCA9685
// ============================================================================
#define PCA9685_I2C_ADDRESS     0x40            ///< Стандартный адрес PCA9685
#define PCA9685_OSC_CLOCK       25000000UL      ///< Частота осциллятора (25 МГц)
#define PCA9685_PWM_RESOLUTION  4096            ///< 12-bit разрешение
#define PCA9685_PWM_FREQUENCY   50              ///< Частота PWM для серво (50 Гц)

// ============================================================================
// Калибровка сервоприводов (MG90S/SG90)
// ============================================================================
#define SERVO_PULSE_MIN_US      500     ///< Мин. импульс (0°)
#define SERVO_PULSE_MAX_US      2500    ///< Макс. импульс (180°)
#define SERVO_PULSE_NEUTRAL_US  1500    ///< Нейтраль (90°)

// ============================================================================
// Углы безопасности (ТЗ п.2.1-2.4)
// ============================================================================
#define SERVO_SAFE_MAX_ANGLE    75      ///< Макс. рабочий угол (градусы)
#define SERVO_NEUTRAL_BUFFER    5       ///< Буферная зона вокруг нейтрали

// ============================================================================
// Параметры предстартового теста
// ============================================================================
#define SERVO_TEST_CYCLES           3           ///< Количество циклов теста
#define SERVO_TEST_PAUSE_MS         1000        ///< Пауза между циклами
#define SERVO_TEST_STEP_DELAY_MS    600         ///< Задержка между шагами

// Последовательность углов для теста (логический диапазон -90...+90)
extern const int16_t SERVO_TEST_SEQUENCE[];
extern const uint8_t SERVO_TEST_SEQUENCE_SIZE;

// ============================================================================
// Класс контроллера сервоприводов
// ============================================================================
class PCA9685ServoController {
public:
    /**
     * @brief Конструктор контроллера
     * @param i2c_address I2C адрес PCA9685 (по умолчанию 0x40)
     */
    explicit PCA9685ServoController(uint8_t i2c_address = PCA9685_I2C_ADDRESS);

    /**
     * @brief Инициализация PCA9685
     * @param i2c_manager Ссылка на инициализированный I2C контроллер
     * @return true при успешной инициализации
     */
    bool begin(I2CMasterController& i2c_manager);

    /**
     * @brief Установка угла сервопривода (логический диапазон -90°...+90°)
     * @param servo_index Индекс сервопривода (0-6)
     * @param logical_angle Угол в градусах (-90...+90, где 0 = нейтраль)
     * @return true при успехе
     */
    bool setLogicalAngle(uint8_t servo_index, int16_t logical_angle);

    /**
     * @brief Установка угла (физический диапазон 0°...180°)
     * @param servo_index Индекс сервопривода (0-6)
     * @param physical_angle Угол в градусах (0-180)
     * @return true при успехе
     */
    bool setPhysicalAngle(uint8_t servo_index, uint16_t physical_angle);

    /**
     * @brief Прямая установка ширины импульса
     * @param servo_index Индекс сервопривода (0-6)
     * @param pulse_us Ширина импульса в микросекундах (500-2500)
     * @return true при успехе
     */
    bool setPulseWidthUs(uint8_t servo_index, uint16_t pulse_us);

    /**
     * @brief Установка калибровочной коррекции для канала
     * @param servo_index Индекс сервопривода (0-6)
     * @param trim_us Коррекция в микросекундах (-200...+200)
     */
    void setServoTrim(uint8_t servo_index, int16_t trim_us);

    /**
     * @brief Сброс всех сервоприводов в нейтральное положение
     */
    void resetToNeutral();

    /**
     * @brief 🔑 Предстартовый тест сервоприводов
     * @return true при успешном завершении теста
     * @details
     * - Все 7 каналов двигаются ОДНОВРЕМЕННО
     * - Последовательность: 0° → +30° → +60° → 0° → -30° → -60° → 0°
     * - Задержка между шагами: 600 мс
     * - Циклов: 3
     */
    bool runServoTest();

    /**
     * @brief Обработка команд пульта с распределением по каналам
     * @param com_up Команда тангажа (0-180)
     * @param com_left Команда рыскания (0-180)
     * @details Реализует распределение согласно ТЗ:
     *   - Каналы 0,1: Рули высоты (синхронно, реагируют на com_up)
     *   - Каналы 2,3: Рули направления (инверсно, реагируют на com_left)
     *   - Каналы 4,5,6: Крышки/парашют (реагируют на com_up)
     */
    void processFlightCommands(uint8_t com_up, uint8_t com_left);

    /**
     * @brief Проверка подключения PCA9685 по I2C
     * @return true если устройство отвечает на заданном адресе
     */
    bool isChipConnected() const { return _chip_connected; }

    /**
     * @brief Проверка инициализации контроллера
     * @return true если begin() был успешно выполнен
     */
    bool isInitialized() const { return _initialized; }

    /**
     * @brief Получение текущего I2C адреса устройства
     * @return Адрес PCA9685 (0x40 по умолчанию)
     */
    uint8_t getI2CAddress() const { return _i2c_address; }

private:
    static const char* TAG;  ///< Тег для ESP_LOG

    I2CMasterController* _i2c_manager = nullptr;  ///< Указатель на I2C контроллер
    uint8_t _i2c_address;                           ///< I2C адрес PCA9685
    uint8_t _servo_channels[7];                     ///< Номера каналов (0-6)
    int16_t _servo_trim[7] = {0};                   ///< Калибровка для каждого канала
    bool _initialized = false;                      ///< Флаг успешной инициализации
    bool _chip_connected = false;                   ///< Флаг подключения по I2C

    // Параметры сервоприводов
    struct ServoParams {
        uint16_t frequency = PCA9685_PWM_FREQUENCY;
        uint16_t min_pulse_us = SERVO_PULSE_MIN_US;
        uint16_t max_pulse_us = SERVO_PULSE_MAX_US;
        uint16_t neutral_pulse_us = SERVO_PULSE_NEUTRAL_US;
    } _servo_params;

    // ========================================================================
    // Приватные методы
    // ========================================================================

    /**
     * @brief Преобразование ширины импульса (мкс) в тики PCA9685
     * @param pulse_us Ширина импульса в микросекундах
     * @return Количество тиков (0-4095)
     */
    uint16_t _pulseUsToTicks(uint16_t pulse_us) const;

    /**
     * @brief Преобразование логического угла в ширину импульса
     * @param logical_angle Угол в градусах (-90...+90)
     * @return Ширина импульса в микросекундах
     */
    uint16_t _logicalAngleToPulseUs(int16_t logical_angle) const;

    /**
     * @brief Проверка угла на соответствие безопасности
     * @param angle Угол для проверки
     * @return Безопасный угол с учётом ограничений
     */
    int16_t _validateSafetyAngle(int16_t angle) const;

    /**
     * @brief Установка PWM тиков напрямую (низкоуровневый метод)
     * @param channel Номер канала PCA9685 (0-15)
     * @param on_tick Тик включения (обычно 0)
     * @param off_tick Тик выключения (0-4095)
     * @return true при успехе
     */
    bool _setPWM(uint8_t channel, uint16_t on_tick, uint16_t off_tick);

    /**
     * @brief Чтение регистра PCA9685 через I2C
     */
    bool _readRegister(uint8_t reg_addr, uint8_t* data, size_t len);

    /**
     * @brief Запись в регистр PCA9685 через I2C
     */
    bool _writeRegister(uint8_t reg_addr, const uint8_t* data, size_t len);

    /**
     * @brief 🔑 Отладочная печать PWM параметров
     */
    void _printPWMDebug(uint8_t servo_index, int16_t angle,
                       uint16_t pulse_us, uint16_t ticks);
};
