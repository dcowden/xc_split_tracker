#pragma once
#include <Arduino.h>

// RSSI / pass behaviour
struct RuntimeConfig {
    float    ema_alpha;          // 0..1
    uint32_t pass_timeout_ms;    // no "good" samples for this long => finish pass
    int8_t   approach_threshold; // EMA above this => inside pass
};

extern RuntimeConfig g_cfg;  // defined in main.cpp

// Max concurrent tags we care about
constexpr uint8_t  MAX_TAGS = 10;

// 45 s window at 10 ms effective resolution = 4500 samples max
constexpr uint16_t MAX_SAMPLES_PER_PASS = 4500;

// Time quantization used for stored deltas (dt = ticks * SAMPLE_DT_MS).
// Radio might be ~5 ms, but we store at ~10 ms resolution and allow gaps.
constexpr uint16_t SAMPLE_DT_MS = 10;

// Radio receive ring buffer capacity (packets)
constexpr size_t RX_RING_CAPACITY = 1024;

// SD buffering
constexpr size_t   RAW_BUFFER_SIZE      = 8192;   // bytes
constexpr size_t   EVENTS_BUFFER_SIZE   = 2048;   // bytes
constexpr uint32_t SD_FLUSH_INTERVAL_MS = 2000;

// SD wiring (XIAO nRF52840)
constexpr uint8_t SD_CS_PIN = D3;

// I2C wiring (RTC + OLED)
constexpr uint8_t I2C_SDA_PIN = D4;
constexpr uint8_t I2C_SCL_PIN = D5;

// UART1 wiring (external device / downstream)
constexpr uint8_t  EXT_UART_TX_PIN = D6;
constexpr uint8_t  EXT_UART_RX_PIN = D7;
constexpr uint32_t EXT_UART_BAUD   = 115200;

// OLED I2C address
constexpr uint8_t OLED_I2C_ADDR = 0x3C;
