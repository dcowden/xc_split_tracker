#pragma once
#include <Arduino.h>
#include "config.h"

enum class EventType : uint8_t {
    Start = 1,
    Peak  = 2,
    End   = 3,
};

struct Event {
    uint32_t  rel_ms;   // sender-relative time (ms)
    uint32_t  pass_id;
    uint16_t  tag_id;
    EventType type;
    int8_t    rssi;     // Start/End: raw RSSI; Peak: rounded median + EMA (raw fallback)
};

struct TagContext {
    bool     used        = false;
    uint16_t tag_id      = 0;

    bool     in_pass     = false;
    uint32_t pass_id     = 0;

    // Peak timestamp during session
    uint32_t peak_rel_ms = 0;

    // Last sample bookkeeping
    int8_t   last_rssi   = 0;
    uint32_t last_rel_ms = 0;
    uint32_t last_wall_ms= 0;
};

extern TagContext g_tags[MAX_TAGS];
extern volatile uint32_t g_total_events;
extern volatile uint32_t g_total_samples;
