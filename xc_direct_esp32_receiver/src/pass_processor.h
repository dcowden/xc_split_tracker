// pass_processor.h
#pragma once
#include <Arduino.h>
#include <math.h>

#include "config.h"
#include "types.h"
#include "display.h"   // UI updates only when a pass is FINISHED

// =====================================================
// External globals (as before)
// =====================================================
extern RuntimeConfig g_cfg;
extern TagContext g_tags[MAX_TAGS];
extern volatile uint32_t g_total_events;

using EventSinkFn = void(*)(const Event&);

static EventSinkFn g_sink = nullptr;
static uint32_t    g_next_pass_id = 1;
static volatile uint32_t g_passes_completed = 0;

// =====================================================
// Debug controls
// =====================================================
#ifndef PASS_DEBUG
#define PASS_DEBUG 1
#endif

#ifndef PASS_DEBUG_PERIOD_MS
#define PASS_DEBUG_PERIOD_MS 1000
#endif

#if PASS_DEBUG
  #define PASS_DBG_PRINTF(...) do { Serial.printf(__VA_ARGS__); } while (0)
#else
  #define PASS_DBG_PRINTF(...) do {} while (0)
#endif

// =====================================================
// Streaming "prominence / drop" peak-pass detector params
// Mirrors the Python defaults/behavior closely
// =====================================================
#ifndef PASS_PROMINENCE_DB
#define PASS_PROMINENCE_DB (10.0f)      // rise above valley to begin tracking
#endif

#ifndef PASS_DROP_DB
#define PASS_DROP_DB (10.0f)            // drop from peak to end
#endif

#ifndef PASS_MAX_PASS_TIME_MS
#define PASS_MAX_PASS_TIME_MS (20000u) // timeout from pass start (valley time)
#endif

#ifndef PASS_MIN_SEP_MS
#define PASS_MIN_SEP_MS (0u)           // optional extra separation between passes
#endif

// =====================================================
// Per-tag detector state (ONE instance per tag slot)
// Kept outside TagContext to avoid duplicating TagContext.
// =====================================================
struct PeakPassState {
  // EMA
  bool  ema_init = false;
  float ema      = 0.0f;

  // tracking state
  bool tracking = false;

  // valley (idle minimum)
  float    valley        = 0.0f;
  uint32_t valley_wall   = 0;     // "now_wall_ms" at valley
  uint32_t valley_rel    = 0;     // rel_ms at valley
  int8_t   valley_rssi   = 0;     // raw at valley (best effort)

  // pass start (defined at valley)
  uint32_t start_wall = 0;
  uint32_t start_rel  = 0;
  int8_t   start_rssi = 0;

  // peak (while tracking)
  float    peak        = 0.0f;
  uint32_t peak_wall   = 0;
  uint32_t peak_rel    = 0;
  int8_t   peak_rssi   = 0;       // rounded from peak EMA (like Python)

  // last finish (for min separation)
  uint32_t last_finish_wall = 0;
};

// One state per slot
static PeakPassState s_pp[MAX_TAGS];

// Optional periodic debug timing per slot
static uint32_t s_dbg_last_print_wall[MAX_TAGS] = {0};

// =====================================================
// API
// =====================================================
inline void pass_set_event_sink(EventSinkFn fn) { g_sink = fn; }

inline uint32_t pass_get_completed_count() { return (uint32_t)g_passes_completed; }
inline void pass_reset_completed_count()  { g_passes_completed = 0; }

// =====================================================
// Helpers
// =====================================================
static inline int8_t clamp_i8(int16_t v) {
  if (v < -127) v = -127;
  if (v >  127) v =  127;
  return (int8_t)v;
}

// "round away from zero" to match python _round_away_from_zero
static inline int8_t round_away_from_zero_i8(float x) {
  int16_t v = (x >= 0.0f) ? (int16_t)(x + 0.5f) : (int16_t)(x - 0.5f);
  return clamp_i8(v);
}

static inline void emit(TagContext &ctx, uint32_t rel_ms, EventType t, int8_t rssi) {
  Event e{rel_ms, ctx.pass_id, ctx.tag_id, t, rssi};
  if (g_sink) g_sink(e);
  g_total_events++;
}

// =====================================================
// Tag slot management (same style as your existing code)
// =====================================================
inline void pass_init() {
  for (uint8_t i = 0; i < MAX_TAGS; ++i) {
    g_tags[i] = TagContext{};
    s_pp[i]   = PeakPassState{};
    s_dbg_last_print_wall[i] = 0;
  }
  g_next_pass_id = 1;
  g_passes_completed = 0;
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

      // reset state for this slot
      g_tags[i].ema_init = false;
      g_tags[i].ema      = 0.0f;
      g_tags[i].in_pass  = false;
      g_tags[i].pass_id  = 0;
      g_tags[i].peak_ema = 0.0f;
      g_tags[i].peak_rel_ms = 0;

      s_pp[i] = PeakPassState{};
      s_dbg_last_print_wall[i] = 0;

#if PASS_DEBUG
      PASS_DBG_PRINTF("[PASS] new tag slot=%u tag=%u ema_alpha=%.6f prom=%.2f drop=%.2f maxPass=%lums minSep=%lums\n",
                      (unsigned)i, (unsigned)tag_id,
                      (double)g_cfg.ema_alpha,
                      (double)PASS_PROMINENCE_DB,
                      (double)PASS_DROP_DB,
                      (unsigned long)PASS_MAX_PASS_TIME_MS,
                      (unsigned long)PASS_MIN_SEP_MS);
#endif
      return &g_tags[i];
    }
  }
  return nullptr;
}

// =====================================================
// Core per-tag processing (mirror Python PeakPass1Py)
// This is the key: pass_process_sample does "lookup + delegate"
// =====================================================
static inline void pass_finish_and_emit(uint8_t idx, TagContext& ctx, PeakPassState& st,
                                       uint32_t end_rel_ms, int8_t end_rssi) {
  // "pass" identity
  ctx.in_pass = false;

  // Emit events
  emit(ctx, st.peak_rel, EventType::Peak, st.peak_rssi);
  emit(ctx, end_rel_ms,  EventType::End,  end_rssi);

  // UI update + counter
  StatusDisplay::note_pass(ctx.tag_id);
  g_passes_completed++;

  // clear TagContext pass fields
  ctx.pass_id = 0;

  // min separation bookkeeping
  st.last_finish_wall = ctx.last_wall_ms;

  // reset valley at current sample (Python does this at end)
  st.tracking      = false;
  st.valley        = st.ema;
  st.valley_wall   = ctx.last_wall_ms;
  st.valley_rel    = end_rel_ms;
  st.valley_rssi   = end_rssi;
}

static inline void process_one_tag_sample(uint8_t idx, TagContext& ctx, PeakPassState& st,
                                         uint32_t rel_ms, int8_t raw_rssi, uint32_t now_wall_ms) {
  // keep latest samples in ctx (same as before)
  ctx.last_rel_ms  = rel_ms;
  ctx.last_rssi    = raw_rssi;
  ctx.last_wall_ms = now_wall_ms;

  // --- EMA (must match Python exactly) ---
  const float x = (float)raw_rssi;
  if (!st.ema_init) {
    st.ema_init = true;
    st.ema = x;

    // mirror TagContext EMA fields too (handy for logging/inspection)
    ctx.ema_init = true;
    ctx.ema = st.ema;

    // init valley to first EMA
    st.valley      = st.ema;
    st.valley_wall = now_wall_ms;
    st.valley_rel  = rel_ms;
    st.valley_rssi = raw_rssi;
    return;
  }

  const float a = (float)g_cfg.ema_alpha;
  st.ema = a * x + (1.0f - a) * st.ema;

  ctx.ema_init = true;
  ctx.ema = st.ema;

  const float e = st.ema;

  // --- MIN_SEP block (but still track valley) ---
  if (!st.tracking && PASS_MIN_SEP_MS > 0 && st.last_finish_wall != 0) {
    const uint32_t since = now_wall_ms - st.last_finish_wall;
    if (since < (uint32_t)PASS_MIN_SEP_MS) {
      if (e < st.valley) {
        st.valley      = e;
        st.valley_wall = now_wall_ms;
        st.valley_rel  = rel_ms;
        st.valley_rssi = raw_rssi;
      }
      return;
    }
  }

  // --- IDLE: update valley, start tracking on prominence ---
  if (!st.tracking) {
    if (e < st.valley) {
      st.valley      = e;
      st.valley_wall = now_wall_ms;
      st.valley_rel  = rel_ms;
      st.valley_rssi = raw_rssi;
    }

    if (e >= (st.valley + (float)PASS_PROMINENCE_DB)) {
      st.tracking = true;

      // Start time is valley time (Python definition)
      st.start_wall = st.valley_wall;
      st.start_rel  = st.valley_rel;
      st.start_rssi = st.valley_rssi;

      // Assign a new pass id NOW (when tracking begins)
      ctx.in_pass = true;
      ctx.pass_id = g_next_pass_id++;

      // peak init at current sample
      st.peak      = e;
      st.peak_wall = now_wall_ms;
      st.peak_rel  = rel_ms;
      st.peak_rssi = round_away_from_zero_i8(e);

      // also store in ctx for debug/telemetry
      ctx.peak_ema    = st.peak;
      ctx.peak_rel_ms = st.peak_rel;

#if PASS_DEBUG
      PASS_DBG_PRINTF("[PASS] START(tag=%u pass=%lu) valley=%.2f@%lu ema=%.2f rel=%lu\n",
                      (unsigned)ctx.tag_id,
                      (unsigned long)ctx.pass_id,
                      (double)st.valley, (unsigned long)st.start_rel,
                      (double)e, (unsigned long)rel_ms);
#endif
      emit(ctx, st.start_rel, EventType::Start, st.start_rssi);
    }
    return;
  }

  // --- TRACKING: update peak ---
  if (e > st.peak) {
    st.peak      = e;
    st.peak_wall = now_wall_ms;
    st.peak_rel  = rel_ms;
    st.peak_rssi = round_away_from_zero_i8(e);

    ctx.peak_ema    = st.peak;
    ctx.peak_rel_ms = st.peak_rel;
  }

  // --- End conditions: drop from peak OR timeout ---
  const bool dropped   = (e <= (st.peak - (float)PASS_DROP_DB));
  const bool timed_out = (PASS_MAX_PASS_TIME_MS > 0) &&
                         ((now_wall_ms - st.start_wall) >= (uint32_t)PASS_MAX_PASS_TIME_MS);

  if (dropped || timed_out) {
#if PASS_DEBUG
    const uint32_t age = now_wall_ms - st.start_wall;
    PASS_DBG_PRINTF("[PASS] END(tag=%u pass=%lu) peak=%.2f@%lu endEma=%.2f rel=%lu age=%lums %s\n",
                    (unsigned)ctx.tag_id,
                    (unsigned long)ctx.pass_id,
                    (double)st.peak, (unsigned long)st.peak_rel,
                    (double)e, (unsigned long)rel_ms,
                    (unsigned long)age,
                    dropped ? "DROP" : "TIMEOUT");
#endif
    pass_finish_and_emit(idx, ctx, st, rel_ms, raw_rssi);
  }

#if PASS_DEBUG
  if (PASS_DEBUG_PERIOD_MS > 0) {
    if (s_dbg_last_print_wall[idx] == 0 || (now_wall_ms - s_dbg_last_print_wall[idx]) >= (uint32_t)PASS_DEBUG_PERIOD_MS) {
      s_dbg_last_print_wall[idx] = now_wall_ms;
      PASS_DBG_PRINTF("[PASS] tag=%u in=%u pass=%lu rssi=%d ema=%.2f valley=%.2f@%lu peak=%.2f@%lu rel=%lu\n",
                      (unsigned)ctx.tag_id,
                      (unsigned)ctx.in_pass,
                      (unsigned long)ctx.pass_id,
                      (int)raw_rssi,
                      (double)e,
                      (double)st.valley, (unsigned long)st.valley_rel,
                      (double)st.peak, (unsigned long)st.peak_rel,
                      (unsigned long)rel_ms);
    }
  }
#endif
}

// =====================================================
// Top-level entry: find tag slot and delegate.
// No loops other than tag slot lookup (same as your current layout).
// =====================================================
inline void pass_process_sample(uint32_t rel_ms, uint16_t tag_id, int8_t rssi, uint32_t now_wall_ms) {
  uint8_t idx = 0;
  TagContext* ctx = pass_get_ctx(tag_id, &idx);
  if (!ctx) return;

  process_one_tag_sample(idx, *ctx, s_pp[idx], rel_ms, rssi, now_wall_ms);
}

// No longer needed in this algorithm, but keep stub for compatibility
inline void pass_check_idle_timeouts(uint32_t now_wall_ms) {
  (void)now_wall_ms;
  // This detector ends passes by drop-from-peak or timeout inside process_one_tag_sample().
}
