#pragma once
#include <stdint.h>

struct SessionConfig {
    double ema_alpha = 0.05;
    double drop_db = 4.0;
    uint32_t max_session_ms = 35000;
    uint32_t drop_hold_ms = 2000;
    uint32_t idle_end_ms = 5000;
    double rearm_db = 4.0;
};

enum class SessionEnd { None, Drop, Silence, MaxDuration };
enum class SessionUpdate { None, Started, Dropped };

// Streaming port of plotter/main5.py. Doubles preserve Python EMA precision.
struct SessionDetector {
    bool active = false;
    bool waiting = false;
    bool filtered_peak = false;
    double peak_rssi = -127;
    uint32_t peak_rel_ms = 0;
    int8_t last_rssi = -127;
    uint32_t last_rel_ms = 0;
    uint32_t last_wall_ms = 0;
    uint32_t start_rel_ms = 0;
    uint32_t start_wall_ms = 0;
    int8_t start_rssi = -127;
    bool dropping = false;
    uint32_t drop_since = 0;
    double valley = -127;
    double last_signal = -127;
    double ema = 0;
    bool ema_init = false;
    uint8_t count = 0;
    int8_t readings[3] = {};
    uint32_t times[3] = {};
    uint32_t walls[3] = {};

    bool silent(uint32_t now, const SessionConfig& cfg) const {
        return count != 0 && uint32_t(now - last_wall_ms) >= cfg.idle_end_ms;
    }

    SessionEnd timeout_reason(uint32_t now, const SessionConfig& cfg) const {
        if (!active) return SessionEnd::None;
        const uint32_t age = now - start_wall_ms;
        const uint32_t gap = now - last_wall_ms;
        const bool silence = gap >= cfg.idle_end_ms;
        const bool maximum = age >= cfg.max_session_ms;
        // If both deadlines passed between polls, select the earlier one.
        if (silence && (!maximum || gap - cfg.idle_end_ms >= age - cfg.max_session_ms))
            return SessionEnd::Silence;
        return maximum ? SessionEnd::MaxDuration : SessionEnd::None;
    }

    void finish() {
        active = false;
        waiting = true;
        valley = last_signal;
        dropping = false;
    }

    // Call timeout_reason()/finish() and clear silence before feeding a packet.
    SessionUpdate sample(uint32_t rel, uint32_t now, int8_t rssi, const SessionConfig& cfg) {
        if (count && uint32_t(now - last_wall_ms) >= 1000u) {
            count = 0;
            ema_init = false;
            dropping = false;
        }
        last_rssi = rssi;
        last_rel_ms = rel;
        last_wall_ms = now;
        if (count == 3) {
            for (uint8_t i = 0; i < 2; ++i) {
                readings[i] = readings[i + 1];
                times[i] = times[i + 1];
                walls[i] = walls[i + 1];
            }
            count = 2;
        }
        readings[count] = rssi;
        times[count] = rel;
        walls[count++] = now;
        const bool filtered = count == 3;
        double value = rssi;
        uint32_t peak_time = rel;
        uint32_t peak_wall = now;
        if (filtered) {
            int8_t a = readings[0], b = readings[1], c = readings[2];
            if (a > b) { const int8_t t = a; a = b; b = t; }
            if (b > c) { const int8_t t = b; b = c; c = t; }
            if (a > b) { const int8_t t = a; a = b; b = t; }
            ema = ema_init ? cfg.ema_alpha * b + (1.0 - cfg.ema_alpha) * ema : b;
            ema_init = true;
            value = ema;
            peak_time = times[1];
            peak_wall = walls[1];
        }
        last_signal = value;
        if (!active) {
            if (waiting) {
                if (!filtered) return SessionUpdate::None;
                if (value < valley) valley = value;
                if (value < valley + cfg.rearm_db) return SessionUpdate::None;
            }
            active = true;
            waiting = false;
            start_rel_ms = peak_time;
            start_wall_ms = peak_wall;
            start_rssi = rssi;
            peak_rssi = value;
            peak_rel_ms = peak_time;
            filtered_peak = filtered;
            dropping = false;
            return SessionUpdate::Started;
        }
        if (filtered || !filtered_peak) {
            if ((filtered && !filtered_peak) || value > peak_rssi) {
                peak_rssi = value;
                peak_rel_ms = peak_time;
            }
            filtered_peak = filtered_peak || filtered;
        }
        if (filtered && value <= peak_rssi - cfg.drop_db) {
            if (!dropping) {
                dropping = true;
                drop_since = now;
            }
            if (uint32_t(now - drop_since) >= cfg.drop_hold_ms)
                return SessionUpdate::Dropped;
        } else {
            dropping = false;
        }
        return SessionUpdate::None;
    }
};
