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

namespace SdLogger {

// ============================================================
// Config
// ============================================================

#ifndef SD_SPI_MHZ
#define SD_SPI_MHZ 25
#endif

#ifndef RAW_RB_SECTORS
constexpr size_t RAW_RB_SECTORS = 16; // RingBuf size = 512 * sectors
#endif

#ifndef RAW_LOG_FILE_SIZE
#define RAW_LOG_FILE_SIZE (128UL * 1024UL * 1024UL)
#endif

#ifndef EVENTS_FLUSH_INTERVAL_MS
constexpr uint32_t EVENTS_FLUSH_INTERVAL_MS = 1000;
#endif

#ifndef EVT_Q_LEN
constexpr uint16_t EVT_Q_LEN = 64;
#endif

#ifndef EVT_ENQUEUE_SPIN_MS
constexpr uint32_t EVT_ENQUEUE_SPIN_MS = 10;
#endif

#ifndef SD_REMOUNT_PERIOD_MS
constexpr uint32_t SD_REMOUNT_PERIOD_MS = 1000; // how often we *start* a new mount sequence
#endif

#ifndef SD_MOUNT_GRACE_MS
constexpr uint32_t SD_MOUNT_GRACE_MS = 200;     // after boot / after a failure, wait a bit before trying
#endif

#ifndef SD_SERVICE_EVENT_BUDGET
constexpr uint8_t SD_SERVICE_EVENT_BUDGET = 8;
#endif

#ifndef SD_SERVICE_MAX_RAW_BLOCKS
constexpr uint8_t SD_SERVICE_MAX_RAW_BLOCKS = 8;
#endif

#ifndef SDLOG_DEBUG
#define SDLOG_DEBUG 1
#endif

#ifndef SDLOG_HEARTBEAT_MS
#define SDLOG_HEARTBEAT_MS 1000
#endif

#ifndef SDLOG_RSSI_MIN
#define SDLOG_RSSI_MIN (-110)
#endif

#ifndef SDLOG_RSSI_MAX
#define SDLOG_RSSI_MAX (-20)
#endif

// ============================================================
// Public API
// ============================================================

inline bool  is_mounted();     // UI mounted: sd_ok + raw_ok + prealloc_ok
inline bool  mount();          // request mount (non-blocking)
inline void  unmount();        // immediate unmount
inline void  service();        // call often from loop()
inline void  sync();           // bounded flush/drain

inline void  log_raw_sample(uint64_t unix_ms, uint32_t rel_ms, uint16_t tag_id, uint32_t pass_id, int8_t rssi);
inline void  log_event(uint64_t unix_ms, const Event &e);

inline uint32_t dropped_due_to_full();
inline uint32_t dropped_events_due_to_queue_full();
inline uint32_t event_enqueue_spins();

// ============================================================
// State machine
// ============================================================

enum class SdState : uint8_t {
  UNMOUNTED = 0,   // not trying
  WANT_MOUNT,      // user/system requested mount
  MOUNTING,        // stepper is executing
  LOGGING,         // mounted & writing
  FAULTED,         // failed; will retry periodically
};

static SdState  g_state = SdState::UNMOUNTED;
static uint32_t g_state_since_ms = 0;

static inline void set_state(SdState s) {
  g_state = s;
  g_state_since_ms = millis();
#if SDLOG_DEBUG
  const char* name =
    (s == SdState::UNMOUNTED) ? "UNMOUNTED" :
    (s == SdState::WANT_MOUNT)? "WANT_MOUNT" :
    (s == SdState::MOUNTING)  ? "MOUNTING" :
    (s == SdState::LOGGING)   ? "LOGGING" :
    (s == SdState::FAULTED)   ? "FAULTED" : "?";
  Serial.printf("[SDLOG] state -> %s\n", name);
#endif
}

// ============================================================
// Shared objects / flags
// ============================================================

static SdFs   g_sd;
static FsFile g_raw_file;
static FsFile g_evt_file;

static RingBuf<FsFile, 512 * RAW_RB_SECTORS> g_raw_rb;

static bool g_sd_ok           = false;
static bool g_raw_ok          = false;
static bool g_evt_ok          = false;
static bool g_raw_prealloc_ok = false;

static uint32_t g_last_evt_flush_ms   = 0;
static uint32_t g_next_retry_ms       = 0;     // when we may start/continue mounting
static uint32_t g_mount_grace_until_ms= 0;     // small grace window before first attempt

// ============================================================
// Event queue (RAM)
// ============================================================

struct EvtItem {
  uint64_t unix_ms;
  Event    e;
};

static EvtItem  g_evt_q[EVT_Q_LEN];
static volatile uint16_t g_evt_q_head  = 0;
static volatile uint16_t g_evt_q_tail  = 0;
static volatile uint16_t g_evt_q_count = 0;

static inline bool evt_q_full()  { return g_evt_q_count >= EVT_Q_LEN; }
static inline bool evt_q_empty() { return g_evt_q_count == 0; }

static inline bool evt_q_push(uint64_t unix_ms, const Event &e) {
  if (evt_q_full()) return false;
  g_evt_q[g_evt_q_tail].unix_ms = unix_ms;
  g_evt_q[g_evt_q_tail].e       = e;
  g_evt_q_tail = (uint16_t)((g_evt_q_tail + 1) % EVT_Q_LEN);
  g_evt_q_count++;
  return true;
}

static inline bool evt_q_pop(EvtItem &out) {
  if (evt_q_empty()) return false;
  out = g_evt_q[g_evt_q_head];
  g_evt_q_head = (uint16_t)((g_evt_q_head + 1) % EVT_Q_LEN);
  g_evt_q_count--;
  return true;
}

// ============================================================
// Stats
// ============================================================

static volatile uint32_t g_drop_raw_full     = 0;
static volatile uint32_t g_drop_evt_q_full   = 0;
static volatile uint32_t g_evt_enqueue_spins = 0;

#if SDLOG_DEBUG
static volatile uint32_t g_total_raw      = 0;
static volatile uint32_t g_total_evt_in   = 0;
static volatile uint32_t g_total_evt_out  = 0;
static uint32_t g_last_heartbeat_ms = 0;

static volatile uint32_t g_last_rel_ms  = 0;
static volatile uint16_t g_last_tag_id  = 0;
static volatile uint32_t g_last_pass_id = 0;
static volatile int16_t  g_last_rssi    = 0;

static inline bool rssi_sane(int r) { return (r >= SDLOG_RSSI_MIN && r <= SDLOG_RSSI_MAX); }
#endif

// ============================================================
// Helpers
// ============================================================

static inline void sd_bus_quiet() {
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  delay(0);
}

static inline void sd_print_fail_detail(const char* stage) {
  Serial.printf("[SDLOG] %s failed\n", stage);
  if (g_sd.card()) {
    Serial.printf("[SDLOG] card errCode=0x%02X errData=0x%02X\n",
                  (unsigned)g_sd.card()->errorCode(),
                  (unsigned)g_sd.card()->errorData());
  }
}

static inline void write_raw_header_and_position() {
  g_raw_file.seekSet(0);
  g_raw_file.println(F("unix_ms,rel_ms,tag_id,pass_id,rssi"));
  g_raw_file.flush();
}

static inline void write_evt_header_if_empty() {
  if (!g_evt_ok) return;
  if (g_evt_file.size() == 0) {
    g_evt_file.println(F("unix_ms,rel_ms,tag_id,pass_id,event_type,rssi"));
    g_evt_file.flush();
  }
}

static inline void close_files_best_effort() {
  if (g_raw_ok) {
    g_raw_rb.sync();
    g_raw_file.flush();
    g_raw_file.truncate(g_raw_file.curPosition());
    g_raw_file.flush();
    g_raw_file.close();
    g_raw_ok = false;
  }

  if (g_evt_ok) {
    const uint32_t t0 = millis();
    while (!evt_q_empty() && (millis() - t0) < 50 && !g_evt_file.isBusy()) {
      EvtItem it;
      if (!evt_q_pop(it)) break;

      g_evt_file.print((unsigned long long)it.unix_ms); g_evt_file.write(',');
      g_evt_file.print(it.e.rel_ms);                    g_evt_file.write(',');
      g_evt_file.print(it.e.tag_id);                    g_evt_file.write(',');
      g_evt_file.print(it.e.pass_id);                   g_evt_file.write(',');
      g_evt_file.print((uint8_t)it.e.type);             g_evt_file.write(',');
      g_evt_file.println((int)it.e.rssi);
#if SDLOG_DEBUG
      g_total_evt_out++;
#endif
    }

    g_evt_file.flush();
    g_evt_file.truncate(g_evt_file.curPosition());
    g_evt_file.flush();
    g_evt_file.close();
    g_evt_ok = false;
  }

  g_raw_prealloc_ok = false;
}

static inline void sd_end_everything() {
  close_files_best_effort();
  if (g_sd_ok) g_sd.end();
  g_sd_ok = false;
  g_evt_ok = false;
  g_raw_ok = false;
  g_raw_prealloc_ok = false;

  sd_bus_quiet();
  SPI.end();
}

// ============================================================
// Mount stepper: one action per service() call
// ============================================================

struct TryCfg { uint8_t mhz; uint8_t mode; };
static constexpr TryCfg kTries[] = {
  { (uint8_t)SD_SPI_MHZ, SHARED_SPI },
  { 12, SHARED_SPI },
  { 4,  SHARED_SPI },
  { 1,  SHARED_SPI },
  { (uint8_t)SD_SPI_MHZ, DEDICATED_SPI },
  { 12, DEDICATED_SPI },
  { 4,  DEDICATED_SPI },
};

enum class MountStep : uint8_t {
  START = 0,
  SPI_BEGIN,
  TRY_BEGIN,       // try one sd.begin cfg per call
  OPEN_FILES,
  DONE_OK,
  DONE_FAIL,
};

static MountStep g_mstep = MountStep::START;
static uint8_t   g_try_i = 0;

static inline void mount_step_reset() {
  g_mstep = MountStep::START;
  g_try_i = 0;
}

static inline void mount_begin_sequence(uint32_t now) {
  // reset flags but DO NOT clear queued events
  g_sd_ok = g_raw_ok = g_evt_ok = false;
  g_raw_prealloc_ok = false;

  sd_bus_quiet();
  SPI.end();
  mount_step_reset();

  g_mount_grace_until_ms = now + SD_MOUNT_GRACE_MS;
  set_state(SdState::MOUNTING);
}

static inline bool mount_stepper_once(uint32_t now) {
  // returns true only if sequence completed successfully and we are LOGGING

  // honor grace time
  if ((int32_t)(now - g_mount_grace_until_ms) < 0) {
    return false;
  }

  switch (g_mstep) {
    case MountStep::START: {
      g_mstep = MountStep::SPI_BEGIN;
      return false;
    }

    case MountStep::SPI_BEGIN: {
      SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
      g_mstep = MountStep::TRY_BEGIN;
      g_try_i = 0;
      return false;
    }

    case MountStep::TRY_BEGIN: {
      if (g_try_i >= (sizeof(kTries)/sizeof(kTries[0]))) {
        g_mstep = MountStep::DONE_FAIL;
        return false;
      }

      const TryCfg cfg0 = kTries[g_try_i++];

      SdSpiConfig cfg(SD_CS_PIN,
                      (cfg0.mode == SHARED_SPI) ? SHARED_SPI : DEDICATED_SPI,
                      SD_SCK_MHZ(cfg0.mhz));

      sd_bus_quiet();

      if (g_sd.begin(cfg)) {
        g_sd_ok = true;
#if SDLOG_DEBUG
        Serial.printf("[SDLOG] sd.begin OK at %u MHz (%s)\n",
                      (unsigned)cfg0.mhz,
                      (cfg0.mode == SHARED_SPI) ? "SHARED" : "DEDICATED");
#endif
        g_mstep = MountStep::OPEN_FILES;
      } else {
#if SDLOG_DEBUG
        Serial.printf("[SDLOG] sd.begin failed at %u MHz (%s)\n",
                      (unsigned)cfg0.mhz,
                      (cfg0.mode == SHARED_SPI) ? "SHARED" : "DEDICATED");
#endif
        sd_print_fail_detail("sd.begin");
      }

      // IMPORTANT: only one try per service() call
      return false;
    }

    case MountStep::OPEN_FILES: {
      // raw.csv: fresh
      if (!g_raw_file.open("/raw.csv", O_RDWR | O_CREAT | O_TRUNC)) {
        sd_print_fail_detail("open raw.csv");
        g_raw_ok = false;
      } else {
        g_raw_ok = true;

        g_raw_prealloc_ok = false;
        if (RAW_LOG_FILE_SIZE > 0) {
          if (!g_raw_file.preAllocate(RAW_LOG_FILE_SIZE)) {
            Log.warningln(F("SdLogger: preAllocate(/raw.csv) FAILED (SD UI will show N)"));
            g_raw_prealloc_ok = false;
          } else {
            g_raw_prealloc_ok = true;
          }
        } else {
          g_raw_prealloc_ok = true;
        }

        write_raw_header_and_position();
        g_raw_rb.begin(&g_raw_file);
      }

      // events.csv: append
      if (!g_evt_file.open("/events.csv", O_RDWR | O_CREAT | O_AT_END)) {
        sd_print_fail_detail("open events.csv");
        g_evt_ok = false;
      } else {
        g_evt_ok = true;
        write_evt_header_if_empty();
        g_last_evt_flush_ms = millis();
      }

      if (g_sd_ok && g_raw_ok && g_evt_ok) {
        g_mstep = MountStep::DONE_OK;
      } else {
        g_mstep = MountStep::DONE_FAIL;
      }
      return false;
    }

    case MountStep::DONE_OK: {
#if SDLOG_DEBUG
      Serial.printf("[SDLOG] mount done. sd_ok=%u raw_ok=%u evt_ok=%u prealloc=%u rb=%uB evtQ=%u/%u\n",
                    (unsigned)g_sd_ok, (unsigned)g_raw_ok, (unsigned)g_evt_ok, (unsigned)g_raw_prealloc_ok,
                    (unsigned)(512 * RAW_RB_SECTORS), (unsigned)g_evt_q_count, (unsigned)EVT_Q_LEN);
#endif
      set_state(SdState::LOGGING);
      return true;
    }

    case MountStep::DONE_FAIL: {
      // stop touching SD immediately; schedule retry later
      sd_end_everything();
      set_state(SdState::FAULTED);
      g_next_retry_ms = now + SD_REMOUNT_PERIOD_MS;
      return false;
    }
  }

  return false;
}

// ============================================================
// Public mount/unmount
// ============================================================

inline bool is_mounted() {
  return g_sd_ok && g_raw_ok && g_raw_prealloc_ok;
}

inline bool mount() {
  // request mount; actual work happens in service()
  if (g_state == SdState::LOGGING) return is_mounted();
  set_state(SdState::WANT_MOUNT);
  return false;
}

inline void unmount() {
  sd_end_everything();
  set_state(SdState::UNMOUNTED);
  // after manual unmount, don't auto-retry unless mount() called again
  g_next_retry_ms = 0;
  mount_step_reset();
}

// ============================================================
// Writers
// ============================================================

inline void log_raw_sample(uint64_t unix_ms,
                           uint32_t rel_ms,
                           uint16_t tag_id,
                           uint32_t pass_id,
                           int8_t rssi) {
  if (g_state != SdState::LOGGING || !g_sd_ok || !g_raw_ok) return;

#if SDLOG_DEBUG
  g_total_raw++;
  g_last_rel_ms  = rel_ms;
  g_last_tag_id  = tag_id;
  g_last_pass_id = pass_id;
  g_last_rssi    = (int16_t)rssi;

  const int r = (int)rssi;
  if (!rssi_sane(r)) {
    Serial.printf("[SDLOG] WARN: insane rssi=%d tag=%u rel=%lu\n",
                  r, (unsigned)tag_id, (unsigned long)rel_ms);
  }
#endif

  g_raw_rb.print((unsigned long long)unix_ms); g_raw_rb.write(',');
  g_raw_rb.print(rel_ms);                      g_raw_rb.write(',');
  g_raw_rb.print(tag_id);                      g_raw_rb.write(',');
  g_raw_rb.print(pass_id);                     g_raw_rb.write(',');
  g_raw_rb.println((int)rssi);

  if (g_raw_rb.getWriteError()) {
    g_drop_raw_full++;
    g_raw_rb.clearWriteError();
  }
}

inline void log_event(uint64_t unix_ms, const Event &e) {
  // buffer regardless of SD state
  if (!evt_q_push(unix_ms, e)) {
    const uint32_t start = millis();
    g_evt_enqueue_spins++;

    while (evt_q_full() && (millis() - start) < EVT_ENQUEUE_SPIN_MS) {
      service();  // try to drain if SD is up
      delay(0);
    }

    if (!evt_q_push(unix_ms, e)) {
      g_drop_evt_q_full++;
#if SDLOG_DEBUG
      Serial.printf("[SDLOG] ERROR: event queue FULL -> drop (tag=%u pass=%lu type=%u rel=%lu) q=%u/%u\n",
                    (unsigned)e.tag_id, (unsigned long)e.pass_id, (unsigned)(uint8_t)e.type,
                    (unsigned long)e.rel_ms,
                    (unsigned)g_evt_q_count, (unsigned)EVT_Q_LEN);
#endif
      return;
    }
  }

#if SDLOG_DEBUG
  g_total_evt_in++;
#endif
}

// ============================================================
// service() + sync()
// ============================================================

inline void service() {
  const uint32_t now = millis();

  // ---------- State machine: MOUNT sequencing (one step per call) ----------
  if (g_state == SdState::WANT_MOUNT) {
    mount_begin_sequence(now);
    // no return; let it immediately proceed if grace already elapsed (usually not)
  }

  if (g_state == SdState::FAULTED) {
    if (g_next_retry_ms != 0 && (int32_t)(now - g_next_retry_ms) >= 0) {
      mount_begin_sequence(now);
    }
  }

  if (g_state == SdState::MOUNTING) {
    // IMPORTANT: this does ONE small step per call (at most one sd.begin attempt)
    mount_stepper_once(now);
  }

  // ---------- If not logging, we can still print heartbeat and return ----------
  if (g_state != SdState::LOGGING || !g_sd_ok) {
#if SDLOG_DEBUG
    if (SDLOG_HEARTBEAT_MS > 0 && (now - g_last_heartbeat_ms) >= SDLOG_HEARTBEAT_MS) {
      g_last_heartbeat_ms = now;
      Serial.printf("[SDLOG] hb state=%u raw=%lu evtQ=%u/%u evtIn=%lu evtOut=%lu drops(rawFull=%lu evtDrop=%lu)\n",
                    (unsigned)g_state,
                    (unsigned long)g_total_raw,
                    (unsigned)g_evt_q_count, (unsigned)EVT_Q_LEN,
                    (unsigned long)g_total_evt_in,
                    (unsigned long)g_total_evt_out,
                    (unsigned long)g_drop_raw_full,
                    (unsigned long)g_drop_evt_q_full);
    }
#endif
    return;
  }

  // ---------- Drain raw RingBuf (bounded) ----------
  if (g_raw_ok) {
    uint8_t blocks = SD_SERVICE_MAX_RAW_BLOCKS;
    while (blocks-- && g_raw_rb.bytesUsed() >= 512) {
      if (g_raw_file.isBusy()) break;

      if (g_raw_prealloc_ok) {
        if (g_raw_file.curPosition() + 512 > (uint32_t)RAW_LOG_FILE_SIZE) break;
      }

      const int n = g_raw_rb.writeOut(512);
      if (n != 512) {
        Log.errorln(F("SdLogger: writeOut(512) failed => FAULT"));
        // Fault -> stop SD and retry later
        sd_end_everything();
        set_state(SdState::FAULTED);
        g_next_retry_ms = now + SD_REMOUNT_PERIOD_MS;
        return;
      }
    }
  }

  // ---------- Drain event queue (bounded) ----------
  if (g_evt_ok) {
    uint8_t budget = SD_SERVICE_EVENT_BUDGET;
    while (budget-- && !evt_q_empty()) {
      if (g_evt_file.isBusy()) break;

      EvtItem it;
      if (!evt_q_pop(it)) break;

      g_evt_file.print((unsigned long long)it.unix_ms); g_evt_file.write(',');
      g_evt_file.print(it.e.rel_ms);                    g_evt_file.write(',');
      g_evt_file.print(it.e.tag_id);                    g_evt_file.write(',');
      g_evt_file.print(it.e.pass_id);                   g_evt_file.write(',');
      g_evt_file.print((uint8_t)it.e.type);             g_evt_file.write(',');
      g_evt_file.println((int)it.e.rssi);

#if SDLOG_DEBUG
      g_total_evt_out++;
#endif
    }

    if ((now - g_last_evt_flush_ms) >= EVENTS_FLUSH_INTERVAL_MS) {
      g_last_evt_flush_ms = now;
      g_evt_file.flush();
    }
  }

#if SDLOG_DEBUG
  if (SDLOG_HEARTBEAT_MS > 0 && (now - g_last_heartbeat_ms) >= SDLOG_HEARTBEAT_MS) {
    g_last_heartbeat_ms = now;
    Serial.printf("[SDLOG] hb state=%u raw=%lu evtQ=%u/%u evtIn=%lu evtOut=%lu drops(rawFull=%lu evtDrop=%lu)\n",
                  (unsigned)g_state,
                  (unsigned long)g_total_raw,
                  (unsigned)g_evt_q_count, (unsigned)EVT_Q_LEN,
                  (unsigned long)g_total_evt_in,
                  (unsigned long)g_total_evt_out,
                  (unsigned long)g_drop_raw_full,
                  (unsigned long)g_drop_evt_q_full);
  }
#endif
}

inline void sync() {
  if (g_state != SdState::LOGGING || !g_sd_ok) return;

  const uint32_t t0 = millis();
  while (!evt_q_empty() && (millis() - t0) < 200) {
    service();
    delay(0);
  }

  if (g_raw_ok) {
    g_raw_rb.sync();
    g_raw_file.flush();
  }
  if (g_evt_ok) {
    g_evt_file.flush();
  }
}

// ============================================================
// Stats accessors
// ============================================================

inline uint32_t dropped_due_to_full() { return g_drop_raw_full; }
inline uint32_t dropped_events_due_to_queue_full() { return g_drop_evt_q_full; }
inline uint32_t event_enqueue_spins() { return g_evt_enqueue_spins; }

} // namespace SdLogger
