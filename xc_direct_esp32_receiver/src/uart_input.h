#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>
#include <stdlib.h>

// rlogiacco/CircularBuffer v1.3.3 (template)
#include <CircularBuffer.hpp>

#include "config.h"

using SampleFn = void (*)(uint32_t rel_ms, uint16_t tag_id, int8_t rssi);

class UartInput {
public:
    explicit UartInput(HardwareSerial& s) : m_ser(s) {}

    void begin() {
        m_ser.setRxBufferSize(EXT_UART_RX_BUF);
        m_ser.begin(EXT_UART_BAUD, SERIAL_8N1, EXT_UART_RXPIN, EXT_UART_TXPIN);
        clear_line();
    }

    void set_callback(SampleFn fn) { m_cb = fn; }

    void service() {
        while (m_ser.available()) {
            char c = (char)m_ser.read();
            if (c == '\r') continue;

            if (c == '\n') {
                if (!m_line.isEmpty()) {
                    parse_and_emit();
                    clear_line();
                }
            } else {
                if (m_line.isFull()) {
                    // drop overlong line
                    clear_line();
                } else {
                    m_line.push(c);
                }
            }
        }
    }

private:
    HardwareSerial& m_ser;
    SampleFn m_cb = nullptr;

    // This MUST compile as a template. If it doesn't, you're not using rlogiacco.
    CircularBuffer<char, MAX_LINE_LEN> m_line;

    inline void clear_line() {
        while (!m_line.isEmpty()) m_line.shift();
    }

    void parse_and_emit() {
        char tmp[MAX_LINE_LEN];
        size_t n = 0;
        while (!m_line.isEmpty() && n < (MAX_LINE_LEN - 1)) {
            tmp[n++] = m_line.shift();
        }
        tmp[n] = '\0';

        const char* p = tmp;
        char* end = nullptr;

        long rel = strtol(p, &end, 10);
        if (!end || *end != ',') return;
        p = end + 1;

        long tag = strtol(p, &end, 10);
        if (!end || *end != ',') return;
        p = end + 1;

        long rssi = strtol(p, &end, 10);

        if (m_cb) {
            m_cb((uint32_t)rel, (uint16_t)tag, (int8_t)rssi);
        }
    }
};
