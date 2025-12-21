#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <ArduinoLog.h>
#include "config.h"

namespace RtcTime {
    static RTC_DS3231 rtc;
    static bool       rtc_ok      = false;
    static uint32_t   boot_unix   = 0;   // unix seconds at baseline
    static uint32_t   boot_millis = 0;   // millis() at baseline

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

    inline uint64_t ms_to_unix_ms(uint32_t now_ms) {
        if (!rtc_ok) return 0;
        uint32_t delta_ms = now_ms - boot_millis;
        return (uint64_t)boot_unix * 1000ULL + (uint64_t)delta_ms;
    }

    inline bool set_from_unix_ms(int64_t unix_ms) {
        if (unix_ms <= 0) {
            Log.errorln(F("RTC: invalid unix_ms"));
            return false;
        }

        if (!rtc.begin()) {
            Log.errorln(F("RTC: DS3231 not responding during set"));
            rtc_ok = false;
            return false;
        }

        uint32_t unix_sec = (uint32_t)(unix_ms / 1000LL);
        DateTime dt(unix_sec);

        rtc.adjust(dt);

        // Re-establish baseline immediately after setting
        boot_unix   = unix_sec;
        boot_millis = millis();
        rtc_ok      = true;

        Log.notice("RTC: set from unix_ms=%lld (sec=%lu)\n",
                   (long long)unix_ms,
                   (unsigned long)unix_sec);

        return true;
    }

    inline void format_date(char *buf, size_t len) {
        if (!rtc_ok) {
            snprintf(buf, len, "NO_RTC");
            return;
        }
        DateTime dt = rtc.now();
        snprintf(buf, len, "%02d/%02d/%04d",
                dt.month(),
                dt.day(),
                dt.year());
    }

    inline void format_time_local(char *buf, size_t len) {
        if (!rtc_ok) {
            snprintf(buf, len, "NO_RTC");
            return;
        }

        // Get current UTC time from RTC
        DateTime dt = rtc.now();
        time_t utc = dt.unixtime();

        // Convert to local time using TZ rules
        struct tm local_tm;
        localtime_r(&utc, &local_tm);

        snprintf(buf, len, "%02d:%02d:%02d",
                local_tm.tm_hour,
                local_tm.tm_min,
                local_tm.tm_sec);
    }

    inline void format_date_local(char *buf, size_t len) {
        if (!rtc_ok) {
            snprintf(buf, len, "NO_RTC");
            return;
        }

        // Get current UTC time from RTC
        DateTime dt = rtc.now();
        time_t utc = dt.unixtime();

        // Convert to local time using TZ rules
        struct tm local_tm;
        localtime_r(&utc, &local_tm);

        snprintf(buf, len, "%02d/%02d/%04d",
                local_tm.tm_mon + 1,
                local_tm.tm_mday,
                local_tm.tm_year + 1900);
    }


    inline void format_time(char *buf, size_t len) {
        if (!rtc_ok) {
            snprintf(buf, len, "NO_RTC");
            return;
        }
        DateTime dt = rtc.now();
        snprintf(buf, len, "%02d:%02d:%02d", dt.hour(), dt.minute(), dt.second());
    }
}
