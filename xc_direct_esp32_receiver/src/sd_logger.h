#pragma once

#ifndef DISABLE_FS_H_WARNING
#define DISABLE_FS_H_WARNING
#endif

#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>
#include <RingBuf.h>
#include <ArduinoLog.h>

#include "config.h"
#include "types.h"

// FreeRTOS (ESP32)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

namespace SdLogger {

// -------------------------
// SD config
// -------------------------
#ifndef SD_CONFIG
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(SD_SPI_MHZ))
#endif

#ifndef RAW_RB_SECTORS
constexpr size_t RAW_RB_SECTORS = 16;
#endif

#ifndef RAW_LOG_FILE_SIZE
#define RAW_LOG_FILE_SIZE (128UL * 1024UL * 1024UL)
#endif

#ifndef EVENTS_FLUSH_INTERVAL_MS
constexpr uint32_t EVENTS_FLUSH_INTERVAL_MS = 1000;
#endif

// -------------------------
// Internal state
// -------------------------
static SdFs   g_sd;
static FsFile g_raw_file;
static FsFile g_evt_file;

static RingBuf<FsFile, 512 * RAW_RB_SECTORS> g_raw_rb;

static bool g_sd_ok           = false;
static bool g_raw_ok          = false;
static bool g_evt_ok          = false;
static bool g_raw_prealloc_ok = false;

static uint32_t g_last_evt_flush_ms = 0;

// Task + lock
static TaskHandle_t      g_task = nullptr;
static volatile bool     g_run  = false;
static SemaphoreHandle_t g_lock = nullptr;

// Optional stats
static volatile uint32_t g_drop_raw_lock = 0;
static volatile uint32_t g_drop_raw_full = 0;

// -------------------------
// Helpers
// -------------------------
static inline void lock_take(TickType_t ticks) {
  if (g_lock) xSemaphoreTake(g_lock, ticks);
}
static inline void lock_give() {
  if (g_lock) xSemaphoreGive(g_lock);
}

static inline void write_raw_header_if_empty() {
  if (!g_raw_ok) return;
  if (g_raw_file.size() == 0) {
    g_raw_file.println(F("unix_ms,rel_ms,tag_id,pass_id,rssi"));
    g_raw_file.flush();
  }
}

static inline void write_evt_header_if_empty() {
  if (!g_evt_ok) return;
  if (g_evt_file.size() == 0) {
    g_evt_file.println(F("unix_ms,rel_ms,tag_id,pass_id,event_type,rssi"));
    g_evt_file.flush();
  }
}

// -------------------------
// Public API
// -------------------------
inline bool init() {
  Log.noticeln(F("SdLogger: init SdFat + RingBuf..."));

  if (!g_lock) g_lock = xSemaphoreCreateMutex();

  // Explicit VSPI pins on ESP32-DevKitC-32D
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

  if (!g_sd.begin(SD_CONFIG)) {
    Log.errorln(F("SdLogger: sd.begin FAILED"));
    g_sd_ok = false;
    return false;
  }
  g_sd_ok = true;

  // /raw.csv (fresh each boot)
  if (!g_raw_file.open("/raw.csv", O_RDWR | O_CREAT | O_TRUNC)) {
    Log.errorln(F("SdLogger: open /raw.csv FAILED"));
    g_raw_ok = false;
  } else {
    g_raw_ok = true;
    write_raw_header_if_empty();

    if (RAW_LOG_FILE_SIZE > 0 && g_raw_file.size() < RAW_LOG_FILE_SIZE) {
      if (!g_raw_file.preAllocate(RAW_LOG_FILE_SIZE)) {
        Log.warningln(F("SdLogger: preAllocate(/raw.csv) FAILED; continuing without prealloc"));
        g_raw_prealloc_ok = false;
      } else {
        g_raw_prealloc_ok = true;
        Log.noticeln(F("SdLogger: preAllocated /raw.csv"));
      }
    }

    g_raw_rb.begin(&g_raw_file);
    Log.noticeln(F("SdLogger: RingBuf ready"));
  }

  // /events.csv (append)
  if (!g_evt_file.open("/events.csv", O_RDWR | O_CREAT | O_AT_END)) {
    Log.errorln(F("SdLogger: open /events.csv FAILED"));
    g_evt_ok = false;
  } else {
    g_evt_ok = true;
    write_evt_header_if_empty();
    g_last_evt_flush_ms = millis();
  }

  return g_sd_ok && g_raw_ok && g_evt_ok;
}

// HOT PATH: write into RingBuf, don’t block
inline void log_raw_sample(uint64_t unix_ms,
                           uint32_t rel_ms,
                           uint16_t tag_id,
                           uint32_t pass_id,
                           int8_t rssi) {
  if (!g_sd_ok || !g_raw_ok) return;

  if (g_lock && xSemaphoreTake(g_lock, 0) != pdTRUE) {
    g_drop_raw_lock++;
    return;
  }

  g_raw_rb.print((unsigned long long)unix_ms);
  g_raw_rb.write(',');
  g_raw_rb.print(rel_ms);
  g_raw_rb.write(',');
  g_raw_rb.print(tag_id);
  g_raw_rb.write(',');
  g_raw_rb.print(pass_id);
  g_raw_rb.write(',');
  g_raw_rb.println((int)rssi);

  if (g_raw_rb.getWriteError()) {
    g_drop_raw_full++;
    g_raw_rb.clearWriteError();
  }

  lock_give();
}

// Events are low-rate; direct file write is fine
inline void log_event(uint64_t unix_ms, const Event &e) {
  if (!g_sd_ok || !g_evt_ok) return;

  lock_take(pdMS_TO_TICKS(5));

  g_evt_file.print((unsigned long long)unix_ms);
  g_evt_file.write(',');
  g_evt_file.print(e.rel_ms);
  g_evt_file.write(',');
  g_evt_file.print(e.tag_id);
  g_evt_file.write(',');
  g_evt_file.print(e.pass_id);
  g_evt_file.write(',');
  g_evt_file.print((uint8_t)e.type);
  g_evt_file.write(',');
  g_evt_file.println((int)e.rssi);

  lock_give();
}

// Drain RingBuf in 512-byte chunks (SdFat “good path”)
inline void service() {
  if (!g_sd_ok) return;

  lock_take(pdMS_TO_TICKS(2));

  if (g_raw_ok) {
    while (g_raw_rb.bytesUsed() >= 512 && !g_raw_file.isBusy()) {

      // Optional guard if you care about not exceeding prealloc
      if (g_raw_prealloc_ok && (g_raw_file.curPosition() + 512 > RAW_LOG_FILE_SIZE)) {
        break;
      }

      if (512 != g_raw_rb.writeOut(512)) {
        Log.errorln(F("SdLogger: writeOut(512) failed"));
        break;
      }
    }
  }

  if (g_evt_ok) {
    uint32_t now = millis();
    if ((now - g_last_evt_flush_ms) >= EVENTS_FLUSH_INTERVAL_MS) {
      g_last_evt_flush_ms = now;
      g_evt_file.flush();
    }
  }

  lock_give();
}

inline void sync() {
  if (!g_sd_ok) return;

  lock_take(pdMS_TO_TICKS(100));

  if (g_raw_ok) {
    g_raw_rb.sync();
    g_raw_file.flush();
  }
  if (g_evt_ok) {
    g_evt_file.flush();
  }

  lock_give();
}

// Button finalize: drain + truncate to actual length + close
inline void finalize_and_close() {
  if (g_task) {
    g_run = false;
    vTaskDelay(pdMS_TO_TICKS(50));
    g_task = nullptr;
  }

  lock_take(pdMS_TO_TICKS(200));

  if (g_raw_ok) {
    g_raw_rb.sync();
    g_raw_file.truncate(g_raw_file.curPosition());
    g_raw_file.flush();
    g_raw_file.close();
    g_raw_ok = false;
  }

  if (g_evt_ok) {
    g_evt_file.truncate(g_evt_file.curPosition());
    g_evt_file.flush();
    g_evt_file.close();
    g_evt_ok = false;
  }

  lock_give();
}

inline uint32_t dropped_due_to_lock() { return g_drop_raw_lock; }
inline uint32_t dropped_due_to_full() { return g_drop_raw_full; }

// -------------------------
// Background task
// -------------------------
static void sd_task(void *arg) {
  (void)arg;
  while (g_run) {
    service();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  vTaskDelete(nullptr);
}

inline void start_background_task(uint32_t stack_words = 4096, UBaseType_t prio = 1) {
  if (g_task) return;
  g_run = true;

  // Pin to core 0 (Arduino loop usually runs on core 1)
  BaseType_t ok = xTaskCreatePinnedToCore(sd_task,
                                         "sd_service",
                                         stack_words,
                                         nullptr,
                                         prio,
                                         &g_task,
                                         0);
  if (ok != pdPASS) {
    Log.errorln(F("SdLogger: failed to start background task"));
    g_task = nullptr;
    g_run = false;
  }
}

} // namespace SdLogger
