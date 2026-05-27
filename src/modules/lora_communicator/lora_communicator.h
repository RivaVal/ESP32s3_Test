/**
 * @file lora_communicator.h
 * @brief Неблокирующий LoRa приёмник (SX1278) с FSM и ACK
 * @details 
 * - Адаптирован под RadioLib 7.6.0 и ESP32-S3
 * - Использует прерывания DIO0/DIO1 для пробуждения
 * - Конечный автомат (FSM): INIT -> LISTENING -> PROCESSING -> SENDING_ACK -> LISTENING
 * - Подробная отладка через ESP_LOG
 * @version 2.0 (PlatformIO Migration)
 */
#pragma once
#include <Arduino.h>
#include <esp_log.h>
#include <RadioLib.h>
#include "../../config/pins.h"
#include "../../common/types.h"

#define LORA_TAG "LORA_RX"

enum class LoRaState : uint8_t {
    INIT = 0,
    LISTENING = 1,
    PROCESSING = 2,
    SENDING_ACK = 3,
    ERROR = 4
};

class LoRaReceiver {
public:
    LoRaReceiver();
    ~LoRaReceiver();

    /** @brief Инициализация радио, настройка прерываний, запуск приёма */
    bool begin();
    /** @brief Обновление FSM. Вызывать регулярно в loop() */
    void update();
    /** @brief Проверка готовности нового пакета */
    bool packetAvailable() const;
    /** @brief Чтение пакета. Возвращает размер данных или 0 */
    size_t readPacket(DataComSet_t& outPacket);
    /** @brief Отправка ACK для указанного ID пакета */
    bool sendAck(uint16_t packetId);
    /** @brief Получение копии статистики */
    ReceiverStats getStats() const;
    /** @brief Сброс статистики */
    void resetStats();
    /** @brief Проверка активности соединения (< 5 сек) */
    bool isConnected() const;

private:
    SX1278* _radio = nullptr;
    Module* _module = nullptr;
    
    LoRaState _state = LoRaState::INIT;
    bool _initialized = false;
    
    DataComSet_t _rxBuffer;
    volatile bool _dataReady = false;
    volatile uint16_t _ackIdPending = 0;
    
    ReceiverStats _stats;
    mutable portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;

    void handleRxDone();
    bool validatePacket(const DataComSet_t& pkt);
    void returnToRx();
    
    friend void IRAM_ATTR onLoraDio0ISR();
    friend void IRAM_ATTR onLoraDio1ISR();
    static LoRaReceiver* _instance;
};

// Глобальные ISR
void IRAM_ATTR onLoraDio0ISR();
void IRAM_ATTR onLoraDio1ISR();
