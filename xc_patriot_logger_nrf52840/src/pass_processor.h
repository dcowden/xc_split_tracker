#pragma once
#include <Arduino.h>
#include <ArduinoLog.h>
#include "config.h"
#include "types.h"

// ======================================================================
// GLOBALS / CONFIG
// ======================================================================

// Comes from main.cpp
extern volatile uint32_t g_total_events;

// Tag table from types.h (allocated in main.cpp)
extern TagContext g_tags[MAX_TAGS];

// We assume an effective sampling interval of ~5 ms
static constexpr uint32_t SAMPLE_PERIOD_MS = 5;

// EMA group delay approximation: delay_samples ≈ (1 - alpha) / alpha
inline uint32_t ema_lag_correction_ms() {
    const float a = g_cfg.ema_alpha;
    if (a <= 0.0f) return 0;
    float samples = (1.0f - a) / a;
    if (samples < 0.0f) samples = 0.0f;
    return (uint32_t)lroundf(samples * SAMPLE_PERIOD_MS);
}

// ======================================================================
// EVENT SINK
// ======================================================================

using EventSinkFn = void(*)(const Event&);

static EventSinkFn g_event_sink = nullptr;
static uint32_t    g_next_pass_id = 1;

inline void pass_set_event_sink(EventSinkFn fn) {
    g_event_sink = fn;
}

inline void pass_init() {
    for (uint8_t i = 0; i < MAX_TAGS; ++i) {
        g_tags[i] = TagContext{};
    }
    g_next_pass_id = 1;
}

// For the OLED
inline uint8_t pass_get_tag_count() {
    uint8_t count = 0;
    for (uint8_t i = 0; i < MAX_TAGS; ++i) {
        if (g_tags[i].used) count++;
    }
    return count;
}

inline TagContext* pass_get_ctx(uint8_t tag_id) {
    // find existing
    for (uint8_t i = 0; i < MAX_TAGS; ++i) {
        if (g_tags[i].used && g_tags[i].tag_id == tag_id) {
            return &g_tags[i];
        }
    }
    // allocate new
    for (uint8_t i = 0; i < MAX_TAGS; ++i) {
        if (!g_tags[i].used) {
            g_tags[i].used   = true;
            g_tags[i].tag_id = tag_id;
            return &g_tags[i];
        }
    }
    Log.errorln(F("No free TagContext slots!"));
    return nullptr;
}

inline void pass_emit_event(TagContext &ctx,
                            uint32_t    rel_ms,
                            EventType   type,
                            int8_t      rssi)
{
    Event e;
    e.rel_ms = rel_ms;
    e.pass_id = ctx.pass_id;
    e.tag_id  = ctx.tag_id;
    e.type    = type;
    e.rssi    = rssi;

    if (g_event_sink) {
        g_event_sink(e);
    }
    g_total_events++;
}

// ======================================================================
// PASS LIFECYCLE
// ======================================================================

inline void pass_finish(TagContext &ctx) {
    if (!ctx.in_pass) return;

    Log.notice(F("Tag %u: finish pass %lu (peak_ema=%d at %lu ms)\n"),
               ctx.tag_id,
               (unsigned long)ctx.pass_id,
               (int)lroundf(ctx.peak_ema),
               (unsigned long)ctx.peak_rel_ms);

    // Apply EMA lag correction so peak timestamp is closer to "true" TCA.
    uint32_t corr_ms   = ema_lag_correction_ms();
    uint32_t peak_time = ctx.peak_rel_ms;
    if (peak_time > corr_ms) {
        peak_time -= corr_ms;
    }

    // Peak event
    pass_emit_event(ctx, peak_time,
                    EventType::Peak,
                    (int8_t)lroundf(ctx.peak_ema));

    // End event at last sample time
    pass_emit_event(ctx, ctx.last_rel_ms,
                    EventType::End,
                    ctx.last_rssi);

    ctx.in_pass      = false;
    ctx.pass_id      = 0;
    ctx.ema_init     = false;
    ctx.ema          = 0.0f;
    ctx.peak_ema     = 0.0f;
    ctx.peak_rel_ms  = 0;
}

// Start a new pass
inline void pass_start(TagContext &ctx,
                       const Measurement &m,
                       uint32_t now_ms)
{
    (void)now_ms;  // currently unused but kept for future heuristics

    ctx.in_pass      = true;
    ctx.pass_id      = g_next_pass_id++;
    ctx.ema          = (float)m.rssi;
    ctx.ema_init     = true;

    ctx.last_rssi    = m.rssi;
    ctx.last_rel_ms  = m.rel_ms;
    ctx.last_good_ms = now_ms;

    ctx.peak_ema     = ctx.ema;
    ctx.peak_rel_ms  = m.rel_ms;

    pass_emit_event(ctx, m.rel_ms,
                    EventType::Start,
                    m.rssi);

    Log.notice(F("Tag %u: start pass %lu at %lu ms (rssi=%d)\n"),
               ctx.tag_id,
               (unsigned long)ctx.pass_id,
               (unsigned long)m.rel_ms,
               (int)m.rssi);
}

// ======================================================================
// SAMPLE PROCESSING (CALL PER RADIO SAMPLE IN LOOP CONTEXT)
// ======================================================================

inline void pass_process_sample(const Measurement &m, uint32_t now_ms) {
    TagContext *ctx = pass_get_ctx(m.tag_id);
    if (!ctx) return;

    ctx->last_rel_ms = m.rel_ms;
    ctx->last_rssi   = m.rssi;

    // EMA update
    if (!ctx->ema_init) {
        ctx->ema      = (float)m.rssi;
        ctx->ema_init = true;
    } else {
        const float a  = g_cfg.ema_alpha;
        const float om = 1.0f - a;
        ctx->ema = a * (float)m.rssi + om * ctx->ema;
    }

    // Track last time EMA was "good" (above threshold)
    if (ctx->ema >= g_cfg.approach_threshold) {
        ctx->last_good_ms = now_ms;
    }

    if (!ctx->in_pass) {
        // Start a new pass when EMA crosses above threshold
        if (ctx->ema >= g_cfg.approach_threshold) {
            pass_start(*ctx, m, now_ms);
        }
        return;
    }

    // In a pass: streaming peak tracking
    if (ctx->ema > ctx->peak_ema) {
        ctx->peak_ema    = ctx->ema;
        ctx->peak_rel_ms = m.rel_ms;
    }

    // End condition is handled by timeout, not by threshold flicker
}

// ======================================================================
// TIMEOUT-BASED ENDING
// ======================================================================

inline void pass_check_timeouts(uint32_t now_ms) {
    for (uint8_t i = 0; i < MAX_TAGS; ++i) {
        TagContext &ctx = g_tags[i];
        if (!ctx.used || !ctx.in_pass) continue;

        uint32_t since_good = now_ms - ctx.last_good_ms;
        if (since_good > g_cfg.pass_timeout_ms) {
            Log.notice(F("Tag %u: timeout (%lu ms since good), finishing pass %lu\n"),
                       ctx.tag_id,
                       (unsigned long)since_good,
                       (unsigned long)ctx.pass_id);
            pass_finish(ctx);
        }
    }
}
