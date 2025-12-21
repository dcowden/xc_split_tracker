#pragma once
#include <Arduino.h>

// =====================================================
// MIN-RAM ESP32 UART->SD logger configuration
// =====================================================
// Goals (matches the late nRF52 approach):
//   - Do NOT store samples in RAM.
//   - Compute EMA online per tag.
//   - Peak = peak EMA during the pass.
//   - Start/stop recording via configured RSSI threshold (with optional hysteresis).
//   - Stream raw samples to SD via low-latency buffering + best-effort pre-expand.
//   - Use Ticker for periodic display updates + SD flush.
// =====================================================

// ---------- UART (external nRF52840 -> ESP32) ----------
constexpr uint32_t EXT_UART_BAUD   = 460800;
constexpr size_t   EXT_UART_RX_BUF = 8192;

constexpr int EXT_UART_NUM   = 2;
constexpr int EXT_UART_RXPIN = 16;
constexpr int EXT_UART_TXPIN = -1;   // not used

// ---------- I2C (DS3231 + OLED) ----------
constexpr int I2C_SDA_PIN = 21;
constexpr int I2C_SCL_PIN = 22;

constexpr int SD_CS_PIN   = 5;
constexpr int SD_MISO_PIN = 19;
constexpr int SD_SCK_PIN  = 18;
constexpr int SD_MOSI_PIN = 23;


constexpr uint8_t SD_SPI_MHZ = 25;

// Raw RingBuf capacity in sectors (512 bytes each).
// 8 sectors = 4KB, 16 sectors = 8KB, 32 sectors = 16KB.
// If you ever see RingBuf overruns, bump this.
constexpr size_t RAW_RB_SECTORS = 16;


// ---------- Pass / RSSI behavior ----------
struct RuntimeConfig {
    float    ema_alpha;           // 0..1
    int8_t   threshold_dbm;       // START when EMA >= threshold
    int8_t   hysteresis_db;       // STOP when EMA < (threshold - hysteresis); 0 disables
    uint32_t idle_end_ms;         // end pass if no samples for this long (0 disables)
};

extern RuntimeConfig g_cfg;  // defined in main.cpp
constexpr uint8_t MAX_TAGS = 10;

// ---------- UART parsing ----------
constexpr size_t MAX_LINE_LEN = 96;  // rel_ms,tag_id,rssi (including sign)

// ---------- SD buffering ----------
// Must be multiples of 512 for best performance.
constexpr size_t RAW_BUF_BYTES = 8 * 1024;
constexpr size_t EVT_BUF_BYTES = 2 * 1024;

// Periodic policies
constexpr uint32_t SD_SYNC_INTERVAL_MS   = 2000;
constexpr uint32_t DISPLAY_INTERVAL_MS   = 500;

// Best-effort pre-expand of /raw.csv (reduces metadata churn). 0 disables.
#ifndef RAW_LOG_FILE_SIZE
#define RAW_LOG_FILE_SIZE (32UL * 1024UL * 1024UL)  // 32 MB
#endif

// How often to flush/sync the events file (ms).
constexpr uint32_t EVENTS_FLUSH_INTERVAL_MS = 1000;