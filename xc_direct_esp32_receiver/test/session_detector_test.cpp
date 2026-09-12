#include <assert.h>
#include <stdio.h>
#include "../src/session_detector.h"

int main() {
    SessionConfig cfg;
    assert(cfg.ema_alpha == 0.05 && cfg.drop_db == 4 && cfg.rearm_db == 4);
    assert(cfg.max_session_ms == 35000 && cfg.drop_hold_ms == 2000 && cfg.idle_end_ms == 5000);
    SessionDetector s;
    assert(s.sample(0, 0, -100, cfg) == SessionUpdate::Started);
    assert(s.active); // No absolute RSSI gate.
    assert(s.timeout_reason(4999, cfg) == SessionEnd::None);
    assert(s.timeout_reason(5000, cfg) == SessionEnd::Silence);
    s.finish();
    assert(!s.active && s.waiting);

    cfg.ema_alpha = 1;
    s = SessionDetector{};
    s.sample(0, 0, -50, cfg);
    s.sample(500, 500, -50, cfg);
    s.sample(1000, 1000, -50, cfg);
    for (uint32_t t = 1500; t < 4000; t += 500)
        assert(s.sample(t, t, -80, cfg) == SessionUpdate::None);
    assert(s.sample(4000, 4000, -80, cfg) == SessionUpdate::Dropped);
    assert(s.peak_rssi == -50 && s.peak_rel_ms == 500);
    s.finish();
    for (uint32_t t = 4500; t <= 10000; t += 500)
        assert(s.sample(t, t, -80, cfg) == SessionUpdate::None);
    assert(s.sample(10500, 10500, -60, cfg) == SessionUpdate::None);
    assert(s.sample(11000, 11000, -60, cfg) == SessionUpdate::Started);
    assert(s.start_rel_ms == 10500);

    // Independent clocks: UART rel timestamps are not local millis().
    s = SessionDetector{};
    s.sample(100, UINT32_MAX - 1000, -80, cfg);
    assert(s.timeout_reason(3998, cfg) == SessionEnd::None);
    assert(s.timeout_reason(3999, cfg) == SessionEnd::Silence);
    assert(s.peak_rel_ms == 100);
    cfg.max_session_ms = 2000;
    assert(s.timeout_reason(10000, cfg) == SessionEnd::MaxDuration);

    // A gap resets drop evidence and smoothing, but not an active peak.
    cfg = SessionConfig{};
    cfg.ema_alpha = 1;
    s = SessionDetector{};
    for (uint32_t t = 0; t <= 200; t += 100) s.sample(t, t, -50, cfg);
    s.sample(300, 300, -80, cfg);
    s.sample(400, 400, -80, cfg);
    assert(s.dropping);
    assert(s.sample(4500, 4500, -80, cfg) == SessionUpdate::None);
    assert(!s.dropping && s.peak_rssi == -50);
    puts("Session detector tests passed");
}
