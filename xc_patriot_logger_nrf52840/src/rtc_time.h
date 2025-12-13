#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <ArduinoLog.h>

// Simple namespace wrapper around DS3231 + unix mapping
namespace RtcTime {

    // Internal state
    static RTC_DS3231 rtc;
    static bool       rtc_ok      = false;
    static uint32_t   boot_unix   = 0;   // unix seconds at baseline
    static uint32_t   boot_millis = 0;   // millis() at baseline

    // Call once from setup()
    inline void init() {
        Wire.begin();

        if (!rtc.begin()) {
            Log.errorln(F("RTC DS3231 not found"));
            rtc_ok      = false;
            boot_unix   = 0;
            boot_millis = millis();
            return;
        }

        if (rtc.lostPower()) {
            Log.warningln(F("RTC lost power – time not set"));
            // You should provide a one-time set mechanism externally.
        }

        DateTime now = rtc.now();
        boot_unix   = now.unixtime();
        boot_millis = millis();
        rtc_ok      = true;

        Log.notice(
            "RTC time at boot: %04d-%02d-%02d %02d:%02d:%02d (unix=%lu) baseline_ms=%lu\n",
            now.year(), now.month(), now.day(),
            now.hour(), now.minute(), now.second(),
            (unsigned long)boot_unix,
            (unsigned long)boot_millis
        );
    }

    inline bool is_ok() {
        return rtc_ok;
    }

    // Convert a millis()-based timestamp (same timebase as when init() ran)
    // into unix time in **milliseconds**.
    inline uint64_t ms_to_unix_ms(uint32_t now_ms) {
        if (!rtc_ok) return 0;

        uint32_t delta_ms = now_ms - boot_millis;  // handles wrap
        uint64_t base_ms  = (uint64_t)boot_unix * 1000ULL;
        return base_ms + (uint64_t)delta_ms;
    }

    // Optional helpers if you want current wall clock
    inline DateTime now() {
        return rtc.now();
    }

    inline void format_time(char *buf, size_t len) {
        if (!rtc_ok) {
            snprintf(buf, len, "NO_RTC");
            return;
        }
        DateTime dt = rtc.now();
        snprintf(buf, len, "%02d:%02d:%02d",
                 dt.hour(), dt.minute(), dt.second());
    }

// Set RTC and baseline from a unix seconds value.
inline void set_rtc_from_unix(uint32_t unix_seconds) {
    DateTime dt(unix_seconds);
    rtc.adjust(dt);

    boot_unix   = unix_seconds;
    boot_millis = millis();
    rtc_ok      = true;

    Log.notice(
        "RTC set from BLE: %04d-%02d-%02d %02d:%02d:%02d (unix=%lu) baseline_ms=%lu\n",
        dt.year(), dt.month(), dt.day(),
        dt.hour(), dt.minute(), dt.second(),
        (unsigned long)boot_unix,
        (unsigned long)boot_millis
    );
}

}
