#pragma once

// Use SdFat - Adafruit Fork with RingBuf, same pattern as RingBufLogger/LowLatencyLogger.
#ifndef DISABLE_FS_H_WARNING
#define DISABLE_FS_H_WARNING  // Avoid File-vs-FsFile warning.
#endif

#include <SdFat.h>
#include <RingBuf.h>
#include <ArduinoLog.h>

#include "config.h"
#include "types.h"     // for Event

namespace SdLogger {

// ========================
// SD + RingBuf config
// ========================

// Dedicated SPI, same style as SdFat examples.
#ifndef SD_CONFIG
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(25))
#endif

// Raw RingBuf capacity in sectors (512 bytes each).
// 8 sectors = 4 KB; bump if you see RingBuf overruns.
constexpr size_t RAW_RB_SECTORS = 8;

// Pre-allocated size for /raw.csv in bytes.
// You can override RAW_LOG_FILE_SIZE in config.h if you want.
#ifndef RAW_LOG_FILE_SIZE
#define RAW_LOG_FILE_SIZE (8UL * 1024UL * 1024UL)  // 16 MB
#endif

// How often to flush/sync the events file (ms).
constexpr uint32_t EVENTS_FLUSH_INTERVAL_MS = 1000;

// ========================
// Internal state
// ========================

static SdFs   g_sd;
static FsFile g_raw_file;
static FsFile g_events_file;

// Ring buffer for raw samples (backed by /raw.csv).
static RingBuf<FsFile, 512 * RAW_RB_SECTORS> g_raw_rb;

static bool     g_sd_ok                = false;
static bool     g_raw_ok               = false;
static bool     g_events_ok            = false;
static bool     g_raw_prealloc_ok      = false;
static uint32_t g_last_events_flush_ms = 0;

// ========================
// Helpers
// ========================

inline void write_raw_header_if_empty() {
    if (!g_raw_ok) return;
    if (g_raw_file.size() == 0) {
        // unix_ts,tag_id,pass_id,rssi (unix may be seconds or ms; your choice)
        g_raw_file.println(F("unix_ts,tag_id,pass_id,rssi"));
        g_raw_file.flush();
    }
}

inline void write_events_header_if_empty() {
    if (!g_events_ok) return;
    if (g_events_file.size() == 0) {
        // unix_ts,tag_id,pass_id,event_type,rssi
        g_events_file.println(F("unix_ts,tag_id,pass_id,event_type,rssi"));
        g_events_file.flush();
    }
}

// ========================
// Public API
// ========================

// Call once in setup() after SPI pins are configured.
inline bool init() {
    Log.noticeln(F("SdLogger: init SdFs..."));

    if (!g_sd.begin(SD_CONFIG)) {
        Log.errorln(F("SdLogger: sd.begin() FAILED"));
        g_sd_ok = false;
        return false;
    }
    g_sd_ok = true;

    // -----------------------------
    // /raw.csv – fresh file + preallocation
    // -----------------------------
    if (!g_raw_file.open("/raw.csv", O_RDWR | O_CREAT | O_TRUNC)) {
        Log.errorln(F("SdLogger: failed to open /raw.csv"));
        g_raw_ok = false;
    } else {
        g_raw_ok = true;

        // Header first.
        write_raw_header_if_empty();

        // Pre-allocate contiguous space for low-latency logging.
        if (RAW_LOG_FILE_SIZE > 0 && g_raw_file.size() < RAW_LOG_FILE_SIZE) {
            if (!g_raw_file.preAllocate(RAW_LOG_FILE_SIZE)) {
                Log.warningln(F("SdLogger: preAllocate(/raw.csv) FAILED – continuing without prealloc"));
                g_raw_prealloc_ok = false;
            } else {
                g_raw_prealloc_ok = true;
                Log.noticeln(F("SdLogger: preAllocated /raw.csv"));
            }
        }

        // Attach RingBuf to this file.
        g_raw_rb.begin(&g_raw_file);
        Log.noticeln(F("SdLogger: raw.csv open + RingBuf ready"));
    }

    // -----------------------------
    // /events.csv – append-only, no prealloc (low rate)
    // -----------------------------
    if (!g_events_file.open("/events.csv", O_RDWR | O_CREAT | O_AT_END)) {
        Log.errorln(F("SdLogger: failed to open /events.csv"));
        g_events_ok = false;
    } else {
        g_events_ok = true;
        write_events_header_if_empty();
        g_last_events_flush_ms = millis();
        Log.noticeln(F("SdLogger: events.csv open"));
    }

    return g_sd_ok && g_raw_ok && g_events_ok;
}

// Log a *single RAW sample* into the RingBuf (non-blocking).
// unix_ts  - full Unix timestamp (seconds or ms, your choice)
// tag_id   - tag ID
// pass_id  - current pass ID (or 0 if none)
// rssi     - RSSI dBm
inline void log_raw_sample(uint32_t unix_ts,
                           uint8_t  tag_id,
                           uint32_t pass_id,
                           int8_t   rssi) {
    if (!g_sd_ok || !g_raw_ok) return;

    // Write CSV line into the RingBuf, not directly to SD.
    // Format: unix_ts,tag_id,pass_id,rssi
    g_raw_rb.print(unix_ts);
    g_raw_rb.write(',');
    g_raw_rb.print(tag_id);
    g_raw_rb.write(',');
    g_raw_rb.print(pass_id);
    g_raw_rb.write(',');
    g_raw_rb.println((int)rssi);

    if (g_raw_rb.getWriteError()) {
        // This only happens if RingBuf runs out of space (service() not draining fast enough).
        Log.errorln(F("SdLogger: RingBuf write error (raw)"));
        g_raw_rb.clearWriteError();
    }
}

// Log an *event* line (start/peak/finished).
// We assume your Event struct has: tag_id, pass_id, type, rssi.
inline void log_event(uint32_t unix_ts, const Event &e) {
    if (!g_sd_ok || !g_events_ok) return;

    // Direct write to events.csv is fine – event rate is low.
    // Format: unix_ts,tag_id,pass_id,event_type,rssi
    g_events_file.print(unix_ts);
    g_events_file.write(',');
    g_events_file.print(e.tag_id);
    g_events_file.write(',');
    g_events_file.print(e.pass_id);
    g_events_file.write(',');
    g_events_file.print((uint8_t)e.type);
    g_events_file.write(',');
    g_events_file.println((int)e.rssi);
}

// Call this frequently from loop():
//
//  - Push 512-byte chunks from RingBuf to SD when the card is not busy.
//  - Periodically flush the events file.
//
inline void service() {
    if (!g_sd_ok) return;

    // 1) Drain raw RingBuf in 512-byte blocks, when SD isn't busy.
    if (g_raw_ok) {
        while (g_raw_rb.bytesUsed() >= 512 && !g_raw_file.isBusy()) {
            // Optional: guard against writing beyond preallocated region.
            if (g_raw_prealloc_ok &&
                (g_raw_file.curPosition() + 512 > RAW_LOG_FILE_SIZE)) {
                Log.errorln(F("SdLogger: raw preallocated file full; dropping further raw samples"));
                break;
            }

            if (512 != g_raw_rb.writeOut(512)) {
                Log.errorln(F("SdLogger: raw_rb.writeOut(512) failed"));
                break;
            }
        }
    }

    // 2) Periodically flush events.csv so it’s visible on the card.
    if (g_events_ok) {
        uint32_t now = millis();
        if ((now - g_last_events_flush_ms) >= EVENTS_FLUSH_INTERVAL_MS) {
            g_last_events_flush_ms = now;
            g_events_file.flush();
        }
    }
}

// Optional: explicit sync, e.g. on a graceful shutdown path.
inline void sync() {
    if (!g_sd_ok) return;

    if (g_raw_ok) {
        g_raw_rb.sync();     // flush remaining bytes in RingBuf to file
        g_raw_file.flush();
    }
    if (g_events_ok) {
        g_events_file.flush();
    }
}

} // namespace SdLogger
