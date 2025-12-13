#pragma once
#include <Arduino.h>
#include "config.h"

// ======================================================================
// BASIC TYPES
// ======================================================================

// Single radio measurement pushed by ISR and consumed in loop().
// rel_ms is millis() at the time the packet was received.
struct Measurement {
    uint32_t rel_ms;   // device-relative ms since boot (from millis())
    uint8_t  tag_id;   // from radio packet
    int8_t   rssi;     // dBm (negative)
};

// Events: 1=start, 2=peak, 3=end
enum class EventType : uint8_t {
    Start = 1,
    Peak  = 2,
    End   = 3,
};

struct Event {
    uint32_t  rel_ms;   // event time (same rel_ms base as Measurement)
    uint32_t  pass_id;
    uint8_t   tag_id;
    EventType type;
    int8_t    rssi;
};

// ======================================================================
// PER-TAG CONTEXT
// ======================================================================

struct TagContext {
    bool     used        = false;
    uint8_t  tag_id      = 0;

    bool     in_pass     = false;
    uint32_t pass_id     = 0;

    // EMA state
    float    ema         = 0.0f;
    bool     ema_init    = false;

    // Last sample
    int8_t   last_rssi   = 0;
    uint32_t last_rel_ms = 0;

    // For timeout logic: last time EMA was "good" (>= threshold), in ms
    uint32_t last_good_ms = 0;

    // Streaming peak tracking: best EMA seen so far this pass
    float    peak_ema    = 0.0f;
    uint32_t peak_rel_ms = 0;
};

// Global tag table (allocated in main.cpp)
extern TagContext g_tags[MAX_TAGS];
