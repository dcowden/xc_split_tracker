#pragma once
#include <Arduino.h>
#include <math.h>

#include "config.h"
#include "types.h"

extern RuntimeConfig g_cfg;
extern TagContext g_tags[MAX_TAGS];
extern volatile uint32_t g_total_events;

using EventSinkFn = void(*)(const Event&);

static EventSinkFn g_sink = nullptr;
static uint32_t g_next_pass_id = 1;

// =====================================================
// Debug controls
// =====================================================
#ifndef PASS_DEBUG
#define PASS_DEBUG 1
#endif

#ifndef PASS_DEBUG_PERIOD_MS
#define PASS_DEBUG_PERIOD_MS 1000
#endif

#ifndef PASS_DEBUG_CROSSING_COOLDOWN_MS
#define PASS_DEBUG_CROSSING_COOLDOWN_MS 250
#endif

#if PASS_DEBUG
  #define PASS_DBG_PRINTF(...) do { Serial.printf(__VA_ARGS__); } while (0)
#else
  #define PASS_DBG_PRINTF(...) do {} while (0)
#endif

// =====================================================
// Anti-flap controls
// =====================================================
#ifndef PASS_START_CONFIRM_SAMPLES
#define PASS_START_CONFIRM_SAMPLES 3
#endif

#ifndef PASS_STOP_CONFIRM_SAMPLES
#define PASS_STOP_CONFIRM_SAMPLES 3
#endif

#ifndef PASS_MIN_PASS_MS
#define PASS_MIN_PASS_MS 400
#endif

#ifndef PASS_MIN_GAP_MS
#define PASS_MIN_GAP_MS 250
#endif

// =====================================================
// Per-slot debug / state (kept outside TagContext)
// =====================================================
static uint32_t s_dbg_last_print_wall[MAX_TAGS] = {0};
static uint32_t s_dbg_last_cross_wall[MAX_TAGS] = {0};
static uint32_t s_dbg_samples[MAX_TAGS] = {0};
static int8_t   s_dbg_prev_above_start[MAX_TAGS] = {0}; // 0 unknown, 1 above, 2 below

static uint8_t  s_start_up_cnt[MAX_TAGS]    = {0};
static uint8_t  s_stop_down_cnt[MAX_TAGS]   = {0};
static uint32_t s_pass_start_wall[MAX_TAGS] = {0};
static uint32_t s_last_end_wall[MAX_TAGS]   = {0};

inline void pass_set_event_sink(EventSinkFn fn) { g_sink = fn; }

// Clamp helper
static inline int8_t clamp_i8(int16_t v) {
  if (v < -127) v = -127;
  if (v >  127) v =  127;
  return (int8_t)v;
}

// TRUE hysteresis band:
// start_thr = thr + hyst (harder to start)
// stop_thr  = thr - hyst (harder to stop)
static inline int8_t start_threshold_i8() {
  return clamp_i8((int16_t)g_cfg.threshold_dbm + (int16_t)g_cfg.hysteresis_db);
}
static inline int8_t stop_threshold_i8() {
  return clamp_i8((int16_t)g_cfg.threshold_dbm - (int16_t)g_cfg.hysteresis_db);
}

inline void pass_init() {
  for (uint8_t i = 0; i < MAX_TAGS; ++i) {
    g_tags[i] = TagContext{};

    s_dbg_last_print_wall[i] = 0;
    s_dbg_last_cross_wall[i] = 0;
    s_dbg_samples[i] = 0;
    s_dbg_prev_above_start[i] = 0;

    s_start_up_cnt[i] = 0;
    s_stop_down_cnt[i] = 0;
    s_pass_start_wall[i] = 0;
    s_last_end_wall[i] = 0;
  }
  g_next_pass_id = 1;
}

inline uint8_t pass_get_tag_count() {
  uint8_t c = 0;
  for (uint8_t i = 0; i < MAX_TAGS; ++i) if (g_tags[i].used) c++;
  return c;
}

// Returns ctx and slot index.
inline TagContext* pass_get_ctx(uint16_t tag_id, uint8_t* out_idx = nullptr) {
  for (uint8_t i = 0; i < MAX_TAGS; ++i) {
    if (g_tags[i].used && g_tags[i].tag_id == tag_id) {
      if (out_idx) *out_idx = i;
      return &g_tags[i];
    }
  }
  for (uint8_t i = 0; i < MAX_TAGS; ++i) {
    if (!g_tags[i].used) {
      g_tags[i].used = true;
      g_tags[i].tag_id = tag_id;
      if (out_idx) *out_idx = i;

#if PASS_DEBUG
      PASS_DBG_PRINTF("[PASS] new tag slot=%u tag=%u thr=%d hyst=%d start=%d stop=%d alpha=%.2f startN=%u stopN=%u minpass=%lums mingap=%lums\n",
                      (unsigned)i, (unsigned)tag_id,
                      (int)g_cfg.threshold_dbm, (int)g_cfg.hysteresis_db,
                      (int)start_threshold_i8(), (int)stop_threshold_i8(),
                      (double)g_cfg.ema_alpha,
                      (unsigned)PASS_START_CONFIRM_SAMPLES,
                      (unsigned)PASS_STOP_CONFIRM_SAMPLES,
                      (unsigned long)PASS_MIN_PASS_MS,
                      (unsigned long)PASS_MIN_GAP_MS);
#endif

      // reset per-slot
      s_dbg_last_print_wall[i] = 0;
      s_dbg_last_cross_wall[i] = 0;
      s_dbg_samples[i] = 0;
      s_dbg_prev_above_start[i] = 0;

      s_start_up_cnt[i] = 0;
      s_stop_down_cnt[i] = 0;
      s_pass_start_wall[i] = 0;
      s_last_end_wall[i] = 0;

      return &g_tags[i];
    }
  }
  return nullptr;
}

inline void emit(TagContext &ctx, uint32_t rel_ms, EventType t, int8_t rssi) {
  Event e{rel_ms, ctx.pass_id, ctx.tag_id, t, rssi};
  if (g_sink) g_sink(e);
  g_total_events++;
}

static inline void dbg_periodic(uint8_t idx, const TagContext& ctx, uint32_t now_wall_ms) {
#if PASS_DEBUG
  if (PASS_DEBUG_PERIOD_MS == 0) return;
  if (s_dbg_last_print_wall[idx] == 0 || (now_wall_ms - s_dbg_last_print_wall[idx]) >= PASS_DEBUG_PERIOD_MS) {
    s_dbg_last_print_wall[idx] = now_wall_ms;

    const int thr      = (int)g_cfg.threshold_dbm;
    const int start_thr = (int)start_threshold_i8();
    const int stop_thr  = (int)stop_threshold_i8();
    const uint32_t age  = (ctx.in_pass && s_pass_start_wall[idx]) ? (now_wall_ms - s_pass_start_wall[idx]) : 0;

    PASS_DBG_PRINTF("[PASS] tag=%u samp=%lu in=%u pass=%lu rssi=%d ema=%.2f thr=%d start=%d stop=%d peak=%.2f@%lu rel=%lu wall=%lu upN=%u dnN=%u age=%lums\n",
                    (unsigned)ctx.tag_id,
                    (unsigned long)s_dbg_samples[idx],
                    (unsigned)ctx.in_pass,
                    (unsigned long)ctx.pass_id,
                    (int)ctx.last_rssi,
                    (double)ctx.ema,
                    thr, start_thr, stop_thr,
                    (double)ctx.peak_ema,
                    (unsigned long)ctx.peak_rel_ms,
                    (unsigned long)ctx.last_rel_ms,
                    (unsigned long)ctx.last_wall_ms,
                    (unsigned)s_start_up_cnt[idx],
                    (unsigned)s_stop_down_cnt[idx],
                    (unsigned long)age);
  }
#endif
}

static inline void dbg_crossing_start(uint8_t idx, const TagContext& ctx, uint32_t now_wall_ms, bool now_above_start) {
#if PASS_DEBUG
  if (s_dbg_prev_above_start[idx] == 0) {
    s_dbg_prev_above_start[idx] = now_above_start ? 1 : 2;
    return;
  }
  const bool was_above = (s_dbg_prev_above_start[idx] == 1);
  if (was_above != now_above_start) {
    if (s_dbg_last_cross_wall[idx] == 0 ||
        (now_wall_ms - s_dbg_last_cross_wall[idx]) >= PASS_DEBUG_CROSSING_COOLDOWN_MS) {
      s_dbg_last_cross_wall[idx] = now_wall_ms;
      PASS_DBG_PRINTF("[PASS] tag=%u %s start=%d rssi=%d ema=%.2f in=%u pass=%lu rel=%lu\n",
                      (unsigned)ctx.tag_id,
                      now_above_start ? "CROSS_UP" : "CROSS_DOWN",
                      (int)start_threshold_i8(),
                      (int)ctx.last_rssi,
                      (double)ctx.ema,
                      (unsigned)ctx.in_pass,
                      (unsigned long)ctx.pass_id,
                      (unsigned long)ctx.last_rel_ms);
    }
    s_dbg_prev_above_start[idx] = now_above_start ? 1 : 2;
  }
#endif
}

static inline void pass_start(uint8_t idx, TagContext &ctx, uint32_t rel_ms, int8_t rssi, uint32_t now_wall_ms) {
  ctx.in_pass     = true;
  ctx.pass_id     = g_next_pass_id++;
  ctx.peak_ema    = ctx.ema;
  ctx.peak_rel_ms = rel_ms;

  ctx.last_rel_ms  = rel_ms;
  ctx.last_rssi    = rssi;
  ctx.last_wall_ms = now_wall_ms;

  s_pass_start_wall[idx] = now_wall_ms;
  s_stop_down_cnt[idx] = 0;

#if PASS_DEBUG
  PASS_DBG_PRINTF("[PASS] START tag=%u pass=%lu rel=%lu rssi=%d ema=%.2f start=%d (upcnt=%u)\n",
                  (unsigned)ctx.tag_id,
                  (unsigned long)ctx.pass_id,
                  (unsigned long)rel_ms,
                  (int)rssi,
                  (double)ctx.ema,
                  (int)start_threshold_i8(),
                  (unsigned)s_start_up_cnt[idx]);
#endif

  emit(ctx, rel_ms, EventType::Start, rssi);
}

static inline void pass_finish(uint8_t idx, TagContext &ctx) {
  if (!ctx.in_pass) return;

#if PASS_DEBUG
  const uint32_t age = (s_pass_start_wall[idx] ? (ctx.last_wall_ms - s_pass_start_wall[idx]) : 0);
  PASS_DBG_PRINTF("[PASS] END tag=%u pass=%lu last_rel=%lu last_rssi=%d peak=%.2f@%lu stop=%d start=%d (downcnt=%u age=%lums)\n",
                  (unsigned)ctx.tag_id,
                  (unsigned long)ctx.pass_id,
                  (unsigned long)ctx.last_rel_ms,
                  (int)ctx.last_rssi,
                  (double)ctx.peak_ema,
                  (unsigned long)ctx.peak_rel_ms,
                  (int)stop_threshold_i8(),
                  (int)start_threshold_i8(),
                  (unsigned)s_stop_down_cnt[idx],
                  (unsigned long)age);
#endif

  emit(ctx, ctx.peak_rel_ms, EventType::Peak, (int8_t)lroundf(ctx.peak_ema));
  emit(ctx, ctx.last_rel_ms, EventType::End, ctx.last_rssi);

  ctx.in_pass = false;
  ctx.pass_id = 0;

  s_last_end_wall[idx] = ctx.last_wall_ms;
  s_start_up_cnt[idx] = 0;
  s_stop_down_cnt[idx] = 0;
}

inline void pass_process_sample(uint32_t rel_ms, uint16_t tag_id, int8_t rssi, uint32_t now_wall_ms) {
  uint8_t idx = 0;
  TagContext *ctx = pass_get_ctx(tag_id, &idx);
  if (!ctx) return;

  s_dbg_samples[idx]++;

  ctx->last_rel_ms  = rel_ms;
  ctx->last_rssi    = rssi;
  ctx->last_wall_ms = now_wall_ms;

  // EMA
  if (!ctx->ema_init) {
    ctx->ema = (float)rssi;
    ctx->ema_init = true;
  } else {
    const float a = g_cfg.ema_alpha;
    ctx->ema = a * (float)rssi + (1.0f - a) * ctx->ema;
  }

  const float start_thr = (float)start_threshold_i8();
  const float stop_thr  = (float)stop_threshold_i8();

  const bool now_above_start = (ctx->ema >= start_thr);
  dbg_crossing_start(idx, *ctx, now_wall_ms, now_above_start);
  dbg_periodic(idx, *ctx, now_wall_ms);

  // -------------------------
  // NOT in pass: start logic
  // -------------------------
  if (!ctx->in_pass) {
    if (PASS_MIN_GAP_MS > 0) {
      const uint32_t since_end = now_wall_ms - s_last_end_wall[idx];
      if (s_last_end_wall[idx] != 0 && since_end < PASS_MIN_GAP_MS) {
        s_start_up_cnt[idx] = 0;
        return;
      }
    }

    if (now_above_start) {
      if (s_start_up_cnt[idx] < 255) s_start_up_cnt[idx]++;
    } else {
      s_start_up_cnt[idx] = 0;
    }

    if (s_start_up_cnt[idx] >= PASS_START_CONFIRM_SAMPLES) {
      pass_start(idx, *ctx, rel_ms, rssi, now_wall_ms);
      s_start_up_cnt[idx] = 0;
    }
    return;
  }

  // -------------------------
  // IN pass: peak tracking
  // -------------------------
  if (ctx->ema > ctx->peak_ema) {
    ctx->peak_ema    = ctx->ema;
    ctx->peak_rel_ms = rel_ms;
  }

  // Minimum pass duration
  if (PASS_MIN_PASS_MS > 0 && s_pass_start_wall[idx] != 0) {
    const uint32_t age = now_wall_ms - s_pass_start_wall[idx];
    if (age < PASS_MIN_PASS_MS) {
      s_stop_down_cnt[idx] = 0;
      return;
    }
  }

  // -------------------------
  // IN pass: stop logic
  // -------------------------
  if (ctx->ema <= stop_thr) {
    if (s_stop_down_cnt[idx] < 255) s_stop_down_cnt[idx]++;
  } else {
    s_stop_down_cnt[idx] = 0;
  }

  if (s_stop_down_cnt[idx] >= PASS_STOP_CONFIRM_SAMPLES) {
    pass_finish(idx, *ctx);
  }
}

inline void pass_check_idle_timeouts(uint32_t now_wall_ms) {
  if (g_cfg.idle_end_ms == 0) return;

  for (uint8_t i = 0; i < MAX_TAGS; ++i) {
    TagContext &ctx = g_tags[i];
    if (!ctx.used || !ctx.in_pass) continue;

    if ((now_wall_ms - ctx.last_wall_ms) >= g_cfg.idle_end_ms) {
#if PASS_DEBUG
      PASS_DBG_PRINTF("[PASS] IDLE_END tag=%u pass=%lu idle_ms=%lu cfg=%lu\n",
                      (unsigned)ctx.tag_id,
                      (unsigned long)ctx.pass_id,
                      (unsigned long)(now_wall_ms - ctx.last_wall_ms),
                      (unsigned long)g_cfg.idle_end_ms);
#endif
      pass_finish(i, ctx);
    }
  }
}
