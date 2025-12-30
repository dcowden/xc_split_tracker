#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <ArduinoLog.h>
#include <sys/time.h>   // gettimeofday, settimeofday
#include "config.h"

namespace RtcTime {

    static RTC_DS3231 rtc;
    static bool       rtc_ok      = false;
    static uint32_t   boot_unix   = 0;   // unix seconds at baseline
    static uint32_t   boot_millis = 0;   // millis() at baseline

    // "time is real" threshold
    static constexpr uint32_t MIN_VALID_UNIX_SEC = 1577836800UL; // 2020-01-01

    // ---------------------------
    // System time helpers
    // ---------------------------
    inline int64_t system_unix_now_ms() {
        struct timeval tv;
        gettimeofday(&tv, nullptr);
        return (int64_t)tv.tv_sec * 1000LL + (tv.tv_usec / 1000);
    }

    inline bool system_time_valid_now() {
        const int64_t ms = system_unix_now_ms();
        return ms >= (int64_t)MIN_VALID_UNIX_SEC * 1000LL;
    }

    inline bool apply_system_time_ms(int64_t unix_ms) {
        if (unix_ms < (int64_t)MIN_VALID_UNIX_SEC * 1000LL) return false;

        struct timeval tv;
        tv.tv_sec  = (time_t)(unix_ms / 1000LL);
        tv.tv_usec = (suseconds_t)((unix_ms % 1000LL) * 1000LL);
        return settimeofday(&tv, nullptr) == 0;
    }

    // ---------------------------
    // RTC init / validity
    // ---------------------------
    inline void init() {
        Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

        if (!rtc.begin()) {
            Log.errorln(F("RTC: DS3231 not found"));
            rtc_ok      = false;
            boot_unix   = 0;
            boot_millis = millis();
            return;
        }

        if (rtc.lostPower()) {
            Log.warningln(F("RTC: lost power; time may be invalid until set"));
        }

        DateTime now = rtc.now();
        boot_unix   = now.unixtime();
        boot_millis = millis();
        rtc_ok      = true;

        Log.notice("RTC: baseline unix=%lu baseline_ms=%lu\n",
                   (unsigned long)boot_unix,
                   (unsigned long)boot_millis);
    }

    inline bool is_ok() { return rtc_ok; }

    inline bool rtc_time_valid_now() {
        if (!rtc_ok) return false;
        uint32_t sec = rtc.now().unixtime();
        return sec >= MIN_VALID_UNIX_SEC;
    }

    // Seed system clock from RTC (only if RTC looks valid)
    inline bool seed_system_time_from_rtc() {
        if (!rtc_ok) return false;

        uint32_t sec = rtc.now().unixtime();
        if (sec < MIN_VALID_UNIX_SEC) {
            Log.warning("RTC: time looks invalid (unix=%lu); not seeding system time\n",
                        (unsigned long)sec);
            return false;
        }

        const bool ok = apply_system_time_ms((int64_t)sec * 1000LL);
        if (ok) {
            Log.notice("RTC: seeded system time (unix=%lu)\n", (unsigned long)sec);
        } else {
            Log.errorln(F("RTC: failed to seed system time"));
        }
        return ok;
    }

    // ---------------------------
    // RTC-relative unix time
    // ---------------------------
    inline uint64_t ms_to_unix_ms(uint32_t now_ms) {
        if (!rtc_ok) return 0;
        uint32_t delta_ms = now_ms - boot_millis;
        return (uint64_t)boot_unix * 1000ULL + (uint64_t)delta_ms;
    }

    // Best-available "now" in unix ms:
    //  - prefer system time if valid
    //  - else fall back to RTC baseline timeline (better than 1970)
    //  - else 0 (no reliable clock)
    inline int64_t unix_now_ms() {
        if (system_time_valid_now()) return system_unix_now_ms();

        uint64_t rtc_ms = ms_to_unix_ms(millis());
        if (rtc_ms != 0 && rtc_time_valid_now()) return (int64_t)rtc_ms;

        return 0;
    }

    // ---------------------------
    // Set RTC from unix ms
    // ---------------------------
    inline bool set_from_unix_ms(int64_t unix_ms) {
        if (unix_ms < (int64_t)MIN_VALID_UNIX_SEC * 1000LL) {
            Log.errorln(F("RTC: invalid unix_ms"));
            return false;
        }

        if (!rtc.begin()) {
            Log.errorln(F("RTC: DS3231 not responding during set"));
            rtc_ok = false;
            return false;
        }

        uint32_t unix_sec = (uint32_t)(unix_ms / 1000LL);
        rtc.adjust(DateTime(unix_sec));

        // Re-establish baseline immediately after setting
        boot_unix   = unix_sec;
        boot_millis = millis();
        rtc_ok      = true;

        Log.notice("RTC: set from unix_ms=%lld (sec=%lu)\n",
                   (long long)unix_ms,
                   (unsigned long)unix_sec);

        return true;
    }

    // ---------------------------
    // Formatting helpers (used by display)
    // ---------------------------
    inline void format_date(char *buf, size_t len) {
        if (!rtc_ok) { snprintf(buf, len, "NO_RTC"); return; }
        DateTime dt = rtc.now();
        snprintf(buf, len, "%02d/%02d/%04d", dt.month(), dt.day(), dt.year());
    }

    inline void format_time(char *buf, size_t len) {
        if (!rtc_ok) { snprintf(buf, len, "NO_RTC"); return; }
        DateTime dt = rtc.now();
        snprintf(buf, len, "%02d:%02d:%02d", dt.hour(), dt.minute(), dt.second());
    }

    inline void format_time_local(char *buf, size_t len) {
        if (!rtc_ok) { snprintf(buf, len, "NO_RTC"); return; }
        time_t utc = (time_t)rtc.now().unixtime();
        struct tm local_tm;
        localtime_r(&utc, &local_tm);
        snprintf(buf, len, "%02d:%02d:%02d", local_tm.tm_hour, local_tm.tm_min, local_tm.tm_sec);
    }

    inline void format_date_local(char *buf, size_t len) {
        if (!rtc_ok) { snprintf(buf, len, "NO_RTC"); return; }
        time_t utc = (time_t)rtc.now().unixtime();
        struct tm local_tm;
        localtime_r(&utc, &local_tm);
        snprintf(buf, len, "%02d/%02d/%04d",
                 local_tm.tm_mon + 1,
                 local_tm.tm_mday,
                 local_tm.tm_year + 1900);
    }

} // namespace RtcTime
