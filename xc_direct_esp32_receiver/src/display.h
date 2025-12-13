#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>

#include "config.h"
#include "rtc_time.h"

// Simple status display wrapper for 128x64 I2C OLED.
// On ESP32 we explicitly set SDA/SCL from config.h so you can share the I2C bus with DS3231.

namespace StatusDisplay {

    static constexpr uint8_t OLED_I2C_ADDR = 0x3C;
    static SSD1306AsciiWire oled;

    inline void init() {
        Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

        oled.begin(&Adafruit128x64, OLED_I2C_ADDR);
        oled.setFont(Adafruit5x7);

        oled.clear();
        oled.setCursor(0, 0);
        oled.println(F("XC Logger Ready"));
        oled.setCursor(0, 1);
        oled.println(F("Waiting for data..."));
    }

    inline void update(uint32_t total_events,
                       uint32_t total_samples,
                       uint8_t  tag_count)
    {
        char timebuf[16];
        RtcTime::format_time(timebuf, sizeof(timebuf));

        oled.clear();

        oled.setCursor(0, 0);
        oled.print(F("Time: "));
        oled.println(timebuf);

        oled.setCursor(0, 1);
        oled.print(F("Events: "));
        oled.println(total_events);

        oled.setCursor(0, 2);
        oled.print(F("Samples: "));
        oled.println(total_samples);

        oled.setCursor(0, 3);
        oled.print(F("Tags: "));
        oled.println(tag_count);
    }

} // namespace StatusDisplay
