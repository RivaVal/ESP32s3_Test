
/**
 * @file TelemetryUARTBridge.cpp
 * @brief Реализация бинарного моста UART2 для телеметрии
 * @details
 * • Алгоритм отправки:
 *   1. Формирование payload (44 байта)
 *   2. Расчёт CRC8 по payload
 *   3. Добавление фрейминга: [0xAA 0x55][payload][CRC8][0xCC 0x33]
 *   4. Отправка через HardwareSerial с проверкой доступности буфера
 * 
 * • Алгоритм приёма команд:
 *   1. Проверка available()
 *   2. Поиск заголовка 0xAA 0x55
 *   3. Чтение payload + CRC + футера
 *   4. Валидация CRC и футера
 *   5. Обработка команды
 * 
 * @note Все операции неблокирующие для работы в loop()
 */
#include "TelemetryUARTBridge.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "UART_TELEM";

HardwareSerial* TelemetryUARTBridge::_uart = nullptr;
bool TelemetryUARTBridge::_initialized = false;
uint32_t TelemetryUARTBridge::_packets_sent = 0;
uint32_t TelemetryUARTBridge::_send_errors = 0;
uint64_t TelemetryUARTBridge::_total_latency_us = 0;

// ============================================================================
// 🔧 CRC8 CALCULATION (Poly 0x07, Init 0x00) — идентично LoRaCommunicator.cpp
// ============================================================================
uint8_t TelemetryUARTBridge::calculateCRC8(const uint8_t* data, size_t length) {
    // 🔑 Таблица CRC8 (Poly 0x07) — та же, что в LoRaCommunicator.cpp
    static const uint8_t CRC8_TABLE[256] = {
        0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
        // ... (полная таблица из LoRaCommunicator.cpp)
        0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
    };
    
    uint8_t crc = 0x00;
    for (size_t i = 0; i < length; i++) {
        crc = CRC8_TABLE[crc ^ data[i]];
    }
    return crc;
}

// ============================================================================
// 🔧 ИНИЦИАЛИЗАЦИЯ UART2 С FLOW CONTROL
// ============================================================================
bool TelemetryUARTBridge::begin(uint32_t baudrate, uint8_t tx_pin, uint8_t rx_pin, 
                                int8_t rts_pin, int8_t cts_pin) {
    if (_initialized) {
        ESP_LOGW(TAG, "⚠️ UART уже инициализирован");
        return true;
    }
    
    ESP_LOGI(TAG, "🚀 Инициализация UART2: %d бод, TX=%d, RX=%d", 
             baudrate, tx_pin, rx_pin);
    
    ESP_LOGI(TAG, "🚀 Инициализация UART2: %d бод | TX=%d, RX=%d | RTS=%d, CTS=%d",
             baudrate, tx_pin, rx_pin, rts_pin, cts_pin);

    // 🔑 Создаём объект HardwareSerial для UART2 (UART_NUM_2 на ESP32-S3)
    _uart = new HardwareSerial(2);  // UART2
    
    // 🔑 ВКЛЮЧЕНИЕ АППАРАТНОГО FLOW CONTROL
    // 5-й параметр (true) включает RTS/CTS. Пины передаются напрямую.
    _uart->begin(baudrate, SERIAL_8N1, rx_pin, tx_pin, true, 
                 static_cast<gpio_num_t>(rts_pin), 
                 static_cast<gpio_num_t>(cts_pin));

    _uart->setRxBufferSize(4096);  // Увеличен для пиков видеопотока
    _uart->setTxBufferSize(4096);
    delay(10);  // 🔧 Задержка для стабилизации

    // 🔧 Диагностика состояния линий RTS/CTS при старте
    ESP_LOGD(TAG, "📊 Состояние Flow Control: RTS(GPIO%d)=%s, CTS(GPIO%d)=%s",
             rts_pin, digitalRead(rts_pin) ? "HIGH" : "LOW",
             cts_pin, digitalRead(cts_pin) ? "HIGH" : "LOW");

            //                    // 🔧 Настройка пинов с аппаратным Flow Control (если заданы)
            //                    if (rts_pin >= 0 && cts_pin >= 0) {
            //                        ESP_LOGI(TAG, "✅ Flow Control: RTS=%d, CTS=%d", rts_pin, cts_pin);
            //                        _uart->begin(baudrate, SERIAL_8N1, rx_pin, tx_pin, true, 
            //                                    static_cast<gpio_num_t>(rts_pin), 
            //                                    static_cast<gpio_num_t>(cts_pin));
            //                    } else {
            //                        ESP_LOGW(TAG, "⚠️ Flow Control отключён (рекомендуется для 921600 бод!)");
            //                        _uart->begin(baudrate, SERIAL_8N1, rx_pin, tx_pin);
            //                    }
                                
            //                    // 🔧 Настройка буферов для высокой скорости
            //                    _uart->setRxBufferSize(2048);  // Увеличенный буфер приёма
            //                    _uart->setTxBufferSize(2048);  // Увеличенный буфер передачи
                                
                                    // 🔧 Задержка для стабилизации
                                    //delay(10);
    
    // 🔧 Проверка связи (опционально)
    if (_uart->available()) {
        ESP_LOGI(TAG, "✅ UART2 готов, в буфере: %d байт", _uart->available());
    }
    
    _initialized = true;
    ESP_LOGI(TAG, "✅ UART2 инициализирован успешно");
    return true;
}

// ============================================================================
// 🔧 ОТПРАВКА ТЕЛЕМЕТРИИ (с фреймингом и CRC)
// ============================================================================
bool TelemetryUARTBridge::sendTelemetry(const TelemetryPacket_t& pkt) {
    if (!_initialized || !_uart) {
        ESP_LOGE(TAG, "❌ sendTelemetry: UART не инициализирован");
        return false;
    }
    
    // 🔑 1. Подготовка буфера: [header][payload][crc][footer]
    uint8_t frame[2 + TELEMETRY_PAYLOAD_SIZE + 1 + 2];  // 2+44+1+2 = 49 байт
    size_t idx = 0;
    
    // Заголовок
    frame[idx++] = UART_HEADER_1;  // 0xAA
    frame[idx++] = UART_HEADER_2;  // 0x55
    
    // Payload (копирование структуры)
    memcpy(&frame[idx], &pkt, TELEMETRY_PAYLOAD_SIZE);
    idx += TELEMETRY_PAYLOAD_SIZE;
    
    // CRC8 (по payload, без заголовка/футера)
    frame[idx++] = calculateCRC8(&frame[2], TELEMETRY_PAYLOAD_SIZE);
    
    // Футер
    frame[idx++] = UART_FOOTER_1;  // 0xCC
    frame[idx++] = UART_FOOTER_2;  // 0x33
    
    // 🔑 2. Отправка (неблокирующая с проверкой буфера)
    return _sendRaw(frame, idx);
}

// ============================================================================
// 🔧 ВНУТРЕННЯЯ ОТПРАВКА С ПРОВЕРКОЙ БУФЕРА
// ============================================================================
bool TelemetryUARTBridge::_sendRaw(const uint8_t* payload, size_t length) {
    // 🔧 Проверка доступности места в буфере передачи
    if (_uart->availableForWrite() < static_cast<int>(length)) {
        ESP_LOGV(TAG, "⏳ Буфер TX занят (%d/%d свободных)", 
                 _uart->availableForWrite(), length);
        _send_errors++;
        return false;  // Не блокируем, просто пропускаем кадр
    }
    
    // 🔧 Замер времени для статистики
    uint32_t start_us = micros();
    
    // 🔧 Отправка
    size_t written = _uart->write(payload, length);
    _uart->flush();  // 🔑 Дождаться отправки (важно для надёжности!)
    
    // 🔧 Статистика
    uint32_t latency_us = micros() - start_us;
    _total_latency_us += latency_us;
    
    if (written == length) {
        _packets_sent++;
        ESP_LOGV(TAG, "✅ Отправлено %d байт за %d мкс", written, latency_us);
        return true;
    } else {
        ESP_LOGE(TAG, "❌ Ошибка отправки: %d/%d байт", written, length);
        _send_errors++;
        return false;
    }
}

// ============================================================================
// 🔧 ПРОВЕРКА ВХОДЯЩИХ КОМАНД (обратный канал)
// ============================================================================
bool TelemetryUARTBridge::checkCommand(uint8_t& cmd) {
    if (!_initialized || !_uart) return false;
    
    // 🔧 Быстрая проверка наличия данных
    if (_uart->available() < 4) return false;  // Мин. размер команды: [hdr][cmd][crc][ftr]
    
    // 🔧 Поиск заголовка (простой алгоритм, можно оптимизировать)
    while (_uart->available() >= 2) {
        uint8_t h1 = _uart->read();
        uint8_t h2 = _uart->peek();  // Смотрим следующий байт без удаления
        
        if (h1 == UART_HEADER_1 && h2 == UART_HEADER_2) {
            _uart->read();  // Прочитать второй байт заголовка
            
            // 🔧 Чтение payload (1 байт команды + 1 байт CRC + 2 байта футера)
            if (_uart->available() >= 4) {
                uint8_t command = _uart->read();
                uint8_t crc_recv = _uart->read();
                uint8_t f1 = _uart->read();
                uint8_t f2 = _uart->read();
                
                // 🔧 Валидация футера
                if (f1 != UART_FOOTER_1 || f2 != UART_FOOTER_2) {
                    ESP_LOGW(TAG, "⚠️ Ошибка футера: 0x%02X 0x%02X", f1, f2);
                    continue;  // Пропустить битый пакет
                }
                
                // 🔧 Валидация CRC
                uint8_t crc_calc = calculateCRC8(&command, 1);
                if (crc_calc != crc_recv) {
                    ESP_LOGW(TAG, "⚠️ CRC mismatch: recv=0x%02X, calc=0x%02X", 
                             crc_recv, crc_calc);
                    continue;
                }
                
                // ✅ Команда валидна
                cmd = command;
                ESP_LOGD(TAG, "✅ Команда получена: 0x%02X", command);
                return true;
            }
        }
    }
    
    return false;
}

// ============================================================================
// 🔧 СТАТИСТИКА
// ============================================================================
void TelemetryUARTBridge::getStats(uint32_t& sent, uint32_t& errors, float& avg_latency_ms) {
    sent = _packets_sent;
    errors = _send_errors;
    avg_latency_ms = (_packets_sent > 0) 
        ? (_total_latency_us / _packets_sent) / 1000.0f 
        : 0.0f;
}

// ============================================================================
// 🔧 СТАТИСТИКА
// ============================================================================
void TelemetryUARTBridge::processRPiCommands(void (*handler)(RPiCommand cmd)) {
    uint8_t cmdByte;
    // Используем существующий checkCommand() для валидации CRC/футера
    while (checkCommand(cmdByte)) {
        if (handler != nullptr) {
            ESP_LOGI(TAG, "📥 RPi команда: 0x%02X", cmdByte);
            handler(static_cast<RPiCommand>(cmdByte));
        }
    }
}

