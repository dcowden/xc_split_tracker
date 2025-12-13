#pragma once
#include <Arduino.h>

#include "config.h"
#include "types.h"

extern RuntimeConfig g_cfg;
extern TagContext g_tags[MAX_TAGS];
extern volatile uint32_t g_total_events;

using EventSinkFn = void(*)(const Event&);

static EventSinkFn g_sink = nullptr;
static uint32_t g_next_pass_id = 1;

inline void pass_set_event_sink(EventSinkFn fn) { g_sink = fn; }

inline void pass_init() {
    for (uint8_t i = 0; i < MAX_TAGS; ++i) g_tags[i] = TagContext{};
    g_next_pass_id = 1;
}

inline uint8_t pass_get_tag_count() {
    uint8_t c = 0;
    for (uint8_t i = 0; i < MAX_TAGS; ++i) if (g_tags[i].used) c++;
    return c;
}

inline TagContext* pass_get_ctx(uint16_t tag_id) {
    for (uint8_t i = 0; i < MAX_TAGS; ++i) {
        if (g_tags[i].used && g_tags[i].tag_id == tag_id) return &g_tags[i];
    }
    for (uint8_t i = 0; i < MAX_TAGS; ++i) {
        if (!g_tags[i].used) {
            g_tags[i].used = true;
            g_tags[i].tag_id = tag_id;
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

inline int8_t stop_threshold() {
    int16_t thr = (int16_t)g_cfg.threshold_dbm - (int16_t)g_cfg.hysteresis_db;
    if (thr < -127) thr = -127;
    if (thr > 127) thr = 127;
    return (int8_t)thr;
}

inline void pass_start(TagContext &ctx, uint32_t rel_ms, int8_t rssi, uint32_t now_wall_ms) {
    ctx.in_pass     = true;
    ctx.pass_id     = g_next_pass_id++;
    ctx.peak_ema    = ctx.ema;
    ctx.peak_rel_ms = rel_ms;

    ctx.last_rel_ms  = rel_ms;
    ctx.last_rssi    = rssi;
    ctx.last_wall_ms = now_wall_ms;

    emit(ctx, rel_ms, EventType::Start, rssi);
}

inline void pass_finish(TagContext &ctx) {
    if (!ctx.in_pass) return;

    emit(ctx, ctx.peak_rel_ms, EventType::Peak, (int8_t)lroundf(ctx.peak_ema));
    emit(ctx, ctx.last_rel_ms, EventType::End, ctx.last_rssi);

    ctx.in_pass = false;
    ctx.pass_id = 0;
}

inline void pass_process_sample(uint32_t rel_ms, uint16_t tag_id, int8_t rssi, uint32_t now_wall_ms) {
    TagContext *ctx = pass_get_ctx(tag_id);
    if (!ctx) return;

    ctx->last_rel_ms  = rel_ms;
    ctx->last_rssi    = rssi;
    ctx->last_wall_ms = now_wall_ms;

    if (!ctx->ema_init) {
        ctx->ema = (float)rssi;
        ctx->ema_init = true;
    } else {
        float a = g_cfg.ema_alpha;
        ctx->ema = a * (float)rssi + (1.0f - a) * ctx->ema;
    }

    if (!ctx->in_pass) {
        if (ctx->ema >= g_cfg.threshold_dbm) pass_start(*ctx, rel_ms, rssi, now_wall_ms);
        return;
    }

    if (ctx->ema > ctx->peak_ema) {
        ctx->peak_ema    = ctx->ema;
        ctx->peak_rel_ms = rel_ms;
    }

    if (ctx->ema < stop_threshold()) {
        pass_finish(*ctx);
    }
}

inline void pass_check_idle_timeouts(uint32_t now_wall_ms) {
    if (g_cfg.idle_end_ms == 0) return;
    for (uint8_t i = 0; i < MAX_TAGS; ++i) {
        TagContext &ctx = g_tags[i];
        if (!ctx.used || !ctx.in_pass) continue;
        if ((now_wall_ms - ctx.last_wall_ms) >= g_cfg.idle_end_ms) pass_finish(ctx);
    }
}
