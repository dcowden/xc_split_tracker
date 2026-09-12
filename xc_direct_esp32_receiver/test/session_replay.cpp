// Host replay harness for comparison with plotter/main5.py (no hardware needed).
#include "../src/session_detector.h"
#include <iostream>
#include <iomanip>
#include <map>

int main() {
    const SessionConfig cfg;
    std::map<unsigned, SessionDetector> tags;
    std::cout << std::setprecision(17);
    auto finish = [&](unsigned tag, SessionDetector& s, uint32_t confirmed, const char* reason) {
        std::cout << tag << ',' << s.start_rel_ms << ',' << int(s.start_rssi) << ','
                  << s.peak_rel_ms << ',' << s.peak_rssi << ',' << s.last_rel_ms << ','
                  << int(s.last_rssi) << ',' << confirmed << ',' << s.filtered_peak << ',' << reason << '\n';
        s.finish();
    };
    auto advance = [&](uint32_t now) {
        for (auto& entry : tags) {
            auto& s = entry.second;
            const auto reason = s.timeout_reason(now, cfg);
            if (reason == SessionEnd::Silence)
                finish(entry.first, s, s.last_rel_ms + cfg.idle_end_ms, "silence");
            else if (reason == SessionEnd::MaxDuration)
                finish(entry.first, s, s.start_rel_ms + cfg.max_session_ms, "max duration");
            if (s.silent(now, cfg)) s = SessionDetector{};
        }
    };
    uint32_t rel = 0, last = 0;
    unsigned tag;
    int rssi;
    bool any = false;
    while (std::cin >> rel >> tag >> rssi) {
        advance(rel);
        auto& s = tags[tag];
        if (s.sample(rel, rel, static_cast<int8_t>(rssi), cfg) == SessionUpdate::Dropped)
            finish(tag, s, rel, "drop");
        last = rel;
        any = true;
    }
    if (any) advance(last + cfg.idle_end_ms);
}
