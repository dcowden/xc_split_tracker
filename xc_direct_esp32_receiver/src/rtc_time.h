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

    inline void format_time(char *buf, size_t len) {
        if (!rtc_ok) {
            snprintf(buf, len, "NO_RTC");
            return;
        }
        DateTime dt = rtc.now();
        snprintf(buf, len, "%02d:%02d:%02d", dt.hour(), dt.minute(), dt.second());
    }
}
