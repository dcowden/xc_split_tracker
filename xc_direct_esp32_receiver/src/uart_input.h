#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>
#include <stdlib.h>

// rlogiacco/CircularBuffer v1.3.3 (template)
#include <CircularBuffer.hpp>

#include "config.h"

// NEW: include seq
using SampleFn = void (*)(uint32_t rel_ms, uint16_t tag_id, uint8_t seq, int8_t rssi);

class UartInput {
public:
    explicit UartInput(HardwareSerial& s) : m_ser(s) {}

    void begin() {
        m_ser.setRxBufferSize(EXT_UART_RX_BUF);
        m_ser.begin(EXT_UART_BAUD, SERIAL_8N1, EXT_UART_RXPIN, EXT_UART_TXPIN);
        clear_line();

#if UARTINPUT_DEBUG
        Serial.printf("[UART] begin baud=%lu rxbuf=%u pin_rx=%d pin_tx=%d fmt=t_ms,tag,seq,rssi\n",
                      (unsigned long)EXT_UART_BAUD,
                      (unsigned)EXT_UART_RX_BUF,
                      (int)EXT_UART_RXPIN,
                      (int)EXT_UART_TXPIN);
#endif
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
                    note_overflow();
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

    CircularBuffer<char, MAX_LINE_LEN> m_line;

    inline void clear_line() {
        while (!m_line.isEmpty()) m_line.shift();
    }

    // -----------------------------
    // Debug / instrumentation
    // -----------------------------
#ifndef UARTINPUT_DEBUG
#define UARTINPUT_DEBUG 1
#endif

#ifndef UARTINPUT_RSSI_MIN
#define UARTINPUT_RSSI_MIN (-110)
#endif

#ifndef UARTINPUT_RSSI_MAX
#define UARTINPUT_RSSI_MAX (-20)
#endif

#ifndef UARTINPUT_BAD_BURST_PRINTS
#define UARTINPUT_BAD_BURST_PRINTS 10
#endif

#ifndef UARTINPUT_BAD_PRINT_COOLDOWN_MS
#define UARTINPUT_BAD_PRINT_COOLDOWN_MS 250
#endif

    uint32_t m_total_lines = 0;
    uint32_t m_good_lines  = 0;
    uint32_t m_bad_lines   = 0;
    uint32_t m_overflows   = 0;

    uint32_t m_bad_burst_left = UARTINPUT_BAD_BURST_PRINTS;
    uint32_t m_last_bad_print_ms = 0;

    inline bool rssi_sane(int r) const {
        return (r >= UARTINPUT_RSSI_MIN && r <= UARTINPUT_RSSI_MAX);
    }

    void note_overflow() {
        m_overflows++;
#if UARTINPUT_DEBUG
        rate_limited_print("[UART] overflow: dropped overlong line\n");
#endif
    }

    void rate_limited_print(const char* msg) {
#if UARTINPUT_DEBUG
        const uint32_t now = millis();
        const bool allow = (m_bad_burst_left > 0) || (now - m_last_bad_print_ms >= UARTINPUT_BAD_PRINT_COOLDOWN_MS);
        if (!allow) return;
        if (m_bad_burst_left > 0) m_bad_burst_left--;
        m_last_bad_print_ms = now;
        Serial.print(msg);
#else
        (void)msg;
#endif
    }

    void rate_limited_print_line(const char* prefix, const char* line) {
#if UARTINPUT_DEBUG
        const uint32_t now = millis();
        const bool allow = (m_bad_burst_left > 0) || (now - m_last_bad_print_ms >= UARTINPUT_BAD_PRINT_COOLDOWN_MS);
        if (!allow) return;
        if (m_bad_burst_left > 0) m_bad_burst_left--;
        m_last_bad_print_ms = now;

        Serial.print(prefix);
        Serial.print(line);
        Serial.print('\n');
#else
        (void)prefix; (void)line;
#endif
    }

    // Parse integer and require the next char to be a delimiter (',' or '\0')
    static inline bool parse_long_field(const char*& p, long& out, char delim_expected) {
        char* end = nullptr;
        out = strtol(p, &end, 10);
        if (!end) return false;
        if (delim_expected == ',') {
            if (*end != ',') return false;
            p = end + 1;
            return true;
        } else { // '\0'
            if (*end != '\0') return false;
            p = end;
            return true;
        }
    }

    void parse_and_emit() {
        // Pull line into tmp buffer
        char tmp[MAX_LINE_LEN];
        size_t n = 0;
        while (!m_line.isEmpty() && n < (MAX_LINE_LEN - 1)) {
            tmp[n++] = m_line.shift();
        }
        tmp[n] = '\0';

        m_total_lines++;

        // Expected format: t_ms,tag,seq,rssi
        const char* p = tmp;
        long rel_l = 0, tag_l = 0, seq_l = 0, rssi_l = 0;

        // rel_ms,
        if (!parse_long_field(p, rel_l, ',')) {
            m_bad_lines++;
            rate_limited_print_line("[UART] bad line (rel): ", tmp);
            return;
        }
        // tag,
        if (!parse_long_field(p, tag_l, ',')) {
            m_bad_lines++;
            rate_limited_print_line("[UART] bad line (tag): ", tmp);
            return;
        }
        // seq,
        if (!parse_long_field(p, seq_l, ',')) {
            m_bad_lines++;
            rate_limited_print_line("[UART] bad line (seq): ", tmp);
            return;
        }
        // rssi\0
        if (!parse_long_field(p, rssi_l, '\0')) {
            m_bad_lines++;
            rate_limited_print_line("[UART] bad line (rssi): ", tmp);
            return;
        }

        // Range / sanity checks
        if (rel_l < 0 || rel_l > 0xFFFFFFFFL) {
            m_bad_lines++;
            rate_limited_print_line("[UART] bad rel range: ", tmp);
            return;
        }
        if (tag_l < 0 || tag_l > 0xFFFFL) {
            m_bad_lines++;
            rate_limited_print_line("[UART] bad tag range: ", tmp);
            return;
        }
        if (seq_l < 0 || seq_l > 255) {
            m_bad_lines++;
            rate_limited_print_line("[UART] bad seq range: ", tmp);
            return;
        }
        if (rssi_l < -127 || rssi_l > 0) { // RSSI should be negative; 0 is "possible" but suspicious
            m_bad_lines++;
            rate_limited_print_line("[UART] bad rssi range: ", tmp);
            return;
        }

        const int rssi_i = (int)rssi_l;
        if (!rssi_sane(rssi_i)) {
            // Don’t reject by default — just print so you can see if the wire data is weird.
            // If you want to hard-drop these, uncomment the return.
            m_bad_lines++;
            rate_limited_print_line("[UART] rssi out of sane range: ", tmp);
            // return;
        }

        m_good_lines++;

        if (m_cb) {
            m_cb((uint32_t)rel_l, (uint16_t)tag_l, (uint8_t)seq_l, (int8_t)rssi_l);
        }
    }
};
