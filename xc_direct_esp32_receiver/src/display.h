#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

#include "config.h"
#include "rtc_time.h"

namespace StatusDisplay {

    static constexpr uint8_t OLED_I2C_ADDR = 0x3C;

    // Full framebuffer = no flicker
    // Hardware I2C, no reset pin
    static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(
        U8G2_R0,
        /* reset=*/ U8X8_PIN_NONE
    );

    // ------------------------------------------------------------
    // Init
    // ------------------------------------------------------------
    inline void init() {
        Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
        u8g2.begin();

        // Pick a clean, readable font (roughly similar to Adafruit5x7)
        u8g2.setFont(u8g2_font_6x10_tf);

        char buf[16];

        u8g2.clearBuffer();

        u8g2.setCursor(0, 12);
        u8g2.print(F("XC Logger v0.1"));

        RtcTime::format_date_local(buf, sizeof(buf));
        u8g2.setCursor(0, 26);
        u8g2.print(buf);

        RtcTime::format_time_local(buf, sizeof(buf));
        u8g2.setCursor(0, 40);
        u8g2.print(buf);

        u8g2.sendBuffer();
    }

    // ------------------------------------------------------------
    // Update (called periodically)
    // ------------------------------------------------------------
    inline void update(uint32_t total_events,
                       uint32_t total_samples,
                       uint8_t  tag_count,
                       bool     sd_present)
    {
        char timebuf[16];

        RtcTime::format_time_local(timebuf, sizeof(timebuf));

        // Draw everything into RAM buffer
        u8g2.clearBuffer();

        // Line 1
        u8g2.setCursor(0, 12);
        u8g2.print(F("Time: "));
        u8g2.print(timebuf);

        // Line 2
        u8g2.setCursor(0, 24);
        u8g2.print(F("Events: "));
        u8g2.print(total_events);

        // Line 3
        u8g2.setCursor(0, 36);
        u8g2.print(F("Samples: "));
        u8g2.print(total_samples);

        // Line 4
        u8g2.setCursor(0, 48);
        u8g2.print(F("Tags: "));
        u8g2.print(tag_count);

        // Bottom line
        u8g2.setCursor(0, 62);
        u8g2.print(F("SD: "));
        u8g2.print(sd_present ? F("Y") : F("N"));

        // Single atomic screen update → no blink
        u8g2.sendBuffer();
    }

} // namespace StatusDisplay
