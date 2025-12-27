#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>
#include <stdlib.h>

// rlogiacco/CircularBuffer v1.3.3 (template)
#include <CircularBuffer.hpp>

#include "config.h"

// callback now includes computed unix_ms
using SampleFn = void (*)(uint32_t rel_ms, int64_t unix_ms, uint16_t tag_id, uint8_t seq, int8_t rssi);

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

    // Provide RTC/NTP-backed Unix-ms clock
    using UnixNowFn = int64_t (*)();
    void set_unix_now_fn(UnixNowFn fn) { m_unix_now = fn; }

    bool timebase_valid() const { return m_time_base_valid; }

    // Convert a receiver rel_ms into unix_ms using the captured base.
    // If base isn't set yet, returns 0 (caller can decide what to do).
    int64_t unix_from_rel(uint32_t rel_ms) const {
        if (!m_time_base_valid) return 0;
        return m_base_unix_ms + (int64_t)((int32_t)(rel_ms - m_base_rel_ms));
    }


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
    SampleFn  m_cb = nullptr;
    UnixNowFn m_unix_now = nullptr;

    CircularBuffer<char, MAX_LINE_LEN> m_line;

    // one-time timebase
    bool     m_time_base_valid = false;
    uint32_t m_base_rel_ms = 0;
    int64_t  m_base_unix_ms = 0;

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

    static inline bool parse_long_field(const char*& p, long& out, char delim_expected) {
        char* end = nullptr;
        out = strtol(p, &end, 10);
        if (!end) return false;
        if (delim_expected == ',') {
            if (*end != ',') return false;
            p = end + 1;
            return true;
        } else {
            if (*end != '\0') return false;
            p = end;
            return true;
        }
    }

    void parse_and_emit() {
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

        if (!parse_long_field(p, rel_l, ',')) { m_bad_lines++; rate_limited_print_line("[UART] bad line (rel): ", tmp); return; }
        if (!parse_long_field(p, tag_l, ',')) { m_bad_lines++; rate_limited_print_line("[UART] bad line (tag): ", tmp); return; }
        if (!parse_long_field(p, seq_l, ',')) { m_bad_lines++; rate_limited_print_line("[UART] bad line (seq): ", tmp); return; }
        if (!parse_long_field(p, rssi_l, '\0')) { m_bad_lines++; rate_limited_print_line("[UART] bad line (rssi): ", tmp); return; }

        if (rel_l < 0 || rel_l > 0xFFFFFFFFL) { m_bad_lines++; rate_limited_print_line("[UART] bad rel range: ", tmp); return; }
        if (tag_l < 0 || tag_l > 0xFFFFL)     { m_bad_lines++; rate_limited_print_line("[UART] bad tag range: ", tmp); return; }
        if (seq_l < 0 || seq_l > 255)         { m_bad_lines++; rate_limited_print_line("[UART] bad seq range: ", tmp); return; }
        if (rssi_l < -127 || rssi_l > 0)      { m_bad_lines++; rate_limited_print_line("[UART] bad rssi range: ", tmp); return; }

        const int rssi_i = (int)rssi_l;
        if (!rssi_sane(rssi_i)) {
            m_bad_lines++;
            rate_limited_print_line("[UART] rssi out of sane range: ", tmp);
        }

        m_good_lines++;

        const uint32_t rel_ms = (uint32_t)rel_l;
        const int64_t unix_now = (m_unix_now) ? m_unix_now() : 0;

        // Set base once (single offset)
        if (!m_time_base_valid && unix_now != 0) {
            m_time_base_valid = true;
            m_base_rel_ms = rel_ms;
            m_base_unix_ms = unix_now;
#if UARTINPUT_DEBUG
            Serial.printf("[UART] time base set: base_rel=%lu base_unix=%lld\n",
                          (unsigned long)m_base_rel_ms, (long long)m_base_unix_ms);
#endif
        }

        int64_t unix_ms = unix_now; // fallback
        if (m_time_base_valid) {
            unix_ms = m_base_unix_ms + (int64_t)((int32_t)(rel_ms - m_base_rel_ms));
        }

        // Optional: handle receiver reboot (rel_ms jump backwards)
        if (m_time_base_valid && rel_ms + 5000UL < m_base_rel_ms) {
#if UARTINPUT_DEBUG
            Serial.printf("[UART] rel_ms jumped backwards (reboot?) rel=%lu old_base_rel=%lu -> rebase\n",
                          (unsigned long)rel_ms, (unsigned long)m_base_rel_ms);
#endif
            m_base_rel_ms  = rel_ms;
            m_base_unix_ms = unix_now;
            unix_ms = m_base_unix_ms;
        }

        if (m_cb) {
            m_cb(rel_ms, unix_ms, (uint16_t)tag_l, (uint8_t)seq_l, (int8_t)rssi_l);
        }
    }
};
