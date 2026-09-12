#pragma once
#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "types.h"
#include "display.h"
#include "session_detector.h"

using EventSinkFn = void(*)(const Event&);
static EventSinkFn g_sink = nullptr;
static uint32_t g_next_pass_id = 1;
static uint32_t g_passes_completed = 0;
static SessionDetector s_sessions[MAX_TAGS];

inline void pass_set_event_sink(EventSinkFn fn) { g_sink = fn; }
inline uint32_t pass_get_completed_count() { return g_passes_completed; }
inline void pass_reset_completed_count() { g_passes_completed = 0; }
inline void pass_init() {
    for (uint8_t i = 0; i < MAX_TAGS; ++i) {
        g_tags[i] = TagContext{};
        s_sessions[i] = SessionDetector{};
    }
    g_next_pass_id = 1;
    g_passes_completed = 0;
}
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
            return &g_tags[i];
        }
    }
    return nullptr;
}
static inline void pass_emit(TagContext& ctx, uint32_t rel_ms, EventType type, int8_t rssi) {
    if (g_sink) g_sink(Event{rel_ms, ctx.pass_id, ctx.tag_id, type, rssi});
}
static inline void pass_finish(uint8_t idx) {
    auto& session = s_sessions[idx];
    auto& ctx = g_tags[idx];
    pass_emit(ctx, session.peak_rel_ms, EventType::Peak, static_cast<int8_t>(lround(session.peak_rssi)));
    // End marks the last observed presence, not the later timeout check.
    pass_emit(ctx, session.last_rel_ms, EventType::End, session.last_rssi);
    session.finish();
    ctx.in_pass = false;
    ctx.pass_id = 0;
    StatusDisplay::note_pass(ctx.tag_id);
    ++g_passes_completed;
}
static inline void pass_finish_if_expired(uint8_t idx, uint32_t now_wall_ms) {
    auto& session = s_sessions[idx];
    if (session.timeout_reason(now_wall_ms, g_cfg) != SessionEnd::None) pass_finish(idx);
    if (session.silent(now_wall_ms, g_cfg)) session = SessionDetector{};
}
inline void pass_process_sample(uint32_t rel_ms, uint16_t tag_id, int8_t rssi, uint32_t now_wall_ms) {
    uint8_t idx = 0;
    TagContext* ctx = pass_get_ctx(tag_id, &idx);
    if (!ctx) return;
    // Close before a packet at/after the deadline can extend the old session.
    pass_finish_if_expired(idx, now_wall_ms);
    auto& session = s_sessions[idx];
    const SessionUpdate update = session.sample(rel_ms, now_wall_ms, rssi, g_cfg);
    ctx->last_rel_ms = rel_ms;
    ctx->last_wall_ms = now_wall_ms;
    ctx->last_rssi = rssi;
    ctx->peak_rel_ms = session.peak_rel_ms;
    if (update == SessionUpdate::Started) {
        ctx->in_pass = true;
        ctx->pass_id = g_next_pass_id++;
        pass_emit(*ctx, session.start_rel_ms, EventType::Start, session.start_rssi);
    }
    if (update == SessionUpdate::Dropped) pass_finish(idx);
}
inline void pass_check_idle_timeouts(uint32_t now_wall_ms) {
    for (uint8_t i = 0; i < MAX_TAGS; ++i) pass_finish_if_expired(i, now_wall_ms);
}
