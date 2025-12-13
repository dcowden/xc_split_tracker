#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
#include "rtc_time.h"

// Simple status display wrapper for 128x64 I2C OLED
// SDA/SCL use the board's default Wire pins (XIAO nRF52840: D4/D5)

namespace StatusDisplay {

    // Adjust if your OLED address is different
    static constexpr uint8_t OLED_I2C_ADDR = 0x3C;

    // Global OLED object (one instance)
    static SSD1306AsciiWire oled;

    inline void init() {
        // Use default Wire pins; for XIAO nRF52840 this is correct
        Wire.begin();

        oled.begin(&Adafruit128x64, OLED_I2C_ADDR);
        oled.setFont(Adafruit5x7);

        oled.clear();
        oled.setCursor(0, 0);
        oled.println(F("XC Logger Ready"));
        oled.setCursor(0, 1);
        oled.println(F("Waiting for data..."));
    }

    // Now includes total_samples in addition to total_events and tag_count
    inline void update(uint32_t total_events,
                       uint32_t total_samples,
                       uint8_t  tag_count)
    {
        char timebuf[16];
        RtcTime::format_time(timebuf, sizeof(timebuf));  // "HH:MM:SS" or "NO_RTC"

        oled.clear();

        // Line 0: time
        oled.setCursor(0, 0);
        oled.print(F("Time: "));
        oled.println(timebuf);

        // Line 1: events
        oled.setCursor(0, 1);
        oled.print(F("Events: "));
        oled.println(total_events);

        // Line 2: raw samples
        oled.setCursor(0, 2);
        oled.print(F("Samples: "));
        oled.println(total_samples);

        // Line 3: tags
        oled.setCursor(0, 3);
        oled.print(F("Tags: "));
        oled.println(tag_count);
    }

} // namespace StatusDisplay
