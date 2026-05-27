/**
 * @file lora_communicator.cpp
 * @brief Реализация LoRa приёмника для ESP32-S3 (RadioLib 7.6.0)
 * @details 
 * - Адаптированы вызовы API: transmit(data, len), clearIrqFlags(), getIrqFlags()
 * - Добавлена потокобезопасность (мьютекс) для ISR и loop()
 * - Вся отладка переведена на ESP_LOG
 */
#include "lora_communicator.h"

// Таблица CRC8 (полином 0x07)
static const uint8_t CRC8_TABLE[256] = {
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};

uint8_t calculateCRC8(const uint8_t* data, size_t length) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < length; i++) crc = CRC8_TABLE[crc ^ data[i]];
    return crc;
}

LoRaReceiver* LoRaReceiver::_instance = nullptr;

LoRaReceiver::LoRaReceiver() {
    _instance = this; // Привязка для ISR
    ESP_LOGI(LORA_TAG, "Constructor: Instance bound to %p", this);
}

LoRaReceiver::~LoRaReceiver() {
    if (_radio) delete _radio;
    if (_module) delete _module;
    if (_instance == this) _instance = nullptr;
    ESP_LOGI(LORA_TAG, "Destructor: Cleanup complete");
}

bool LoRaReceiver::begin() {
    ESP_LOGI(LORA_TAG, "📡 Initializing SX1278...");
    
    // Создаём объекты RadioLib
    _module = new Module(Config::Pins::LORA_CS, Config::Pins::LORA_DIO0, 
                         Config::Pins::LORA_RST, Config::Pins::LORA_DIO1, SPI);
    _radio = new SX1278(_module);
    if (!_radio || !_module) {
        ESP_LOGE(LORA_TAG, "❌ Failed to allocate RadioLib objects");
        return false;
    }

    // Инициализация радио (RadioLib 7.6.0 API)
    int state = _radio->begin(Config::Pins::FREQUENCY, Config::Pins::BANDWIDTH,
                              Config::Pins::SPREADING, Config::Pins::CODING_RATE,
                              Config::Pins::SYNC_WORD, Config::Pins::TX_POWER,
                              Config::Pins::PREAMBLE_LEN);
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(LORA_TAG, "❌ begin() failed: %d", state);
        return false;
    }
    _radio->setCRC(true);

    // Настройка прерываний
    _radio->setDio0Action(onLoraDio0ISR, RISING);
    _radio->setDio1Action(onLoraDio1ISR, RISING);

    // Запуск приёма
    state = _radio->startReceive();
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(LORA_TAG, "❌ startReceive() failed: %d", state);
        return false;
    }

    _initialized = true;
    _state = LoRaState::LISTENING;
    ESP_LOGI(LORA_TAG, "✅ Receiver READY | Freq:%.1f MHz SF%d", Config::Pins::FREQUENCY, Config::Pins::SPREADING);
    return true;
}

void LoRaReceiver::update() {
    if (!_initialized || !_radio) return;

    switch (_state) {
        case LoRaState::INIT:
            _state = LoRaState::LISTENING;
            break;
        case LoRaState::LISTENING:
            // Проверка флагов из ISR
            if (_dataReady) {
                _dataReady = false;
                _state = LoRaState::PROCESSING;
            }
            break;
        case LoRaState::PROCESSING:
            handleRxDone();
            break;
        case LoRaState::SENDING_ACK:
            // ACK отправляется асинхронно внутри handleRxDone при необходимости
            _state = LoRaState::LISTENING;
            returnToRx();
            break;
        case LoRaState::ERROR:
            ESP_LOGW(LORA_TAG, "⚠️ Error state, attempting recovery...");
            _radio->reset();
            delay(50);
            if (_radio->startReceive() == RADIOLIB_ERR_NONE) _state = LoRaState::LISTENING;
            break;
    }
}

void LoRaReceiver::handleRxDone() {
    uint8_t buf[sizeof(DataComSet_t)];
    int16_t len = _radio->readData(buf, sizeof(buf));
    
    if (len <= 0 || static_cast<size_t>(len) < sizeof(DataComSet_t)) {
        ESP_LOGW(LORA_TAG, "⚠️ Read failed/short packet (len=%d)", len);
        _state = LoRaState::LISTENING;
        returnToRx();
        return;
    }

    memcpy(&_rxBuffer, buf, sizeof(DataComSet_t));
    
    if (validatePacket(_rxBuffer)) {
        portENTER_CRITICAL(&_mux);
        _stats.packetsReceived++;
        _stats.packetsSuccess++;
        _stats.lastRssi = _radio->getRSSI();
        _stats.lastPacketTime = millis();
        portEXIT_CRITICAL(&_mux);
        
        ESP_LOGI(LORA_TAG, "✅ Valid Packet | ID:%u | RSSI:%d dBm", _rxBuffer.packet_id, _stats.lastRssi);
        
        if (_rxBuffer.comSetAll & ACK_REQUEST_FLAG) {
            ESP_LOGD(LORA_TAG, "📤 ACK requested for ID %u", _rxBuffer.packet_id);
            sendAck(_rxBuffer.packet_id);
        }
    } else {
        portENTER_CRITICAL(&_mux);
        _stats.crcErrors++;
        portEXIT_CRITICAL(&_mux);
    }
    
    _state = LoRaState::LISTENING;
    returnToRx();
}

bool LoRaReceiver::packetAvailable() const {
    bool ready;
    portENTER_CRITICAL((portMUX_TYPE*)&_mux);
    ready = _dataReady;
    portEXIT_CRITICAL((portMUX_TYPE*)&_mux);
    return ready;
}

size_t LoRaReceiver::readPacket(DataComSet_t& outPacket) {
    if (!_dataReady) return 0;
    portENTER_CRITICAL(&_mux);
    memcpy(&outPacket, &_rxBuffer, sizeof(DataComSet_t));
    _dataReady = false;
    portEXIT_CRITICAL(&_mux);
    return sizeof(DataComSet_t);
}

bool LoRaReceiver::sendAck(uint16_t packetId) {
    AckPacket_t ack;
    ack.packet_id = packetId;
    ack.timestamp = millis();
    ack.status = 1;
    ack.crc8 = calculateCRC8((uint8_t*)&ack.packet_id, sizeof(AckPacket_t) - 3);

    int state = _radio->standby();
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(LORA_TAG, "❌ standby() failed: %d", state);
        return false;
    }
    delayMicroseconds(100);

    // RadioLib 7.6.0: transmit требует данные и длину
    state = _radio->transmit((uint8_t*)&ack, sizeof(AckPacket_t));
    
    if (state == RADIOLIB_ERR_NONE) {
        portENTER_CRITICAL(&_mux);
        _stats.acksSent++;
        _stats.acksSuccess++;
        portEXIT_CRITICAL(&_mux);
        ESP_LOGI(LORA_TAG, "✅ ACK sent for ID %u", packetId);
        return true;
    }
    ESP_LOGE(LORA_TAG, "❌ ACK transmit failed: %d", state);
    return false;
}

bool LoRaReceiver::validatePacket(const DataComSet_t& pkt) {
    if (pkt.preamble[0] != 0xAA || pkt.preamble[1] != 0x55) return false;
    uint8_t calc = calculateCRC8((uint8_t*)&pkt.packet_id, sizeof(DataComSet_t) - 3);
    return pkt.crc8 == calc;
}

void LoRaReceiver::returnToRx() {
    if (_radio->startReceive() != RADIOLIB_ERR_NONE) {
        ESP_LOGW(LORA_TAG, "⚠️ Failed to return to RX mode");
        _state = LoRaState::ERROR;
    }
}

ReceiverStats LoRaReceiver::getStats() const {
    ReceiverStats s;
    portENTER_CRITICAL((portMUX_TYPE*)&_mux);
    s = _stats;
    portEXIT_CRITICAL((portMUX_TYPE*)&_mux);
    return s;
}

void LoRaReceiver::resetStats() {
    portENTER_CRITICAL(&_mux);
    memset(&_stats, 0, sizeof(_stats));
    portEXIT_CRITICAL(&_mux);
}

bool LoRaReceiver::isConnected() const {
    return (millis() - _stats.lastPacketTime) < Config::Timing::CONNECTION_TIMEOUT_MS;
}

// ISR функции (должны быть глобальными для RadioLib)
void IRAM_ATTR onLoraDio0ISR() {
    if (LoRaReceiver::_instance) {
        portENTER_CRITICAL_ISR(&LoRaReceiver::_instance->_mux);
        LoRaReceiver::_instance->_dataReady = true;
        portEXIT_CRITICAL_ISR(&LoRaReceiver::_instance->_mux);
    }
}

void IRAM_ATTR onLoraDio1ISR() {
    if (LoRaReceiver::_instance) {
        // Таймаут приёма: просто перезапускаем приём
        _instance->_radio->startReceive();
    }
}
