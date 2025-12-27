#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

#include "config.h"
#include "rtc_time.h"

namespace StatusDisplay {

    static constexpr uint8_t OLED_I2C_ADDR = 0x3C;

    // Full framebuffer = no flicker
    static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(
        U8G2_R0,
        /* reset=*/ U8X8_PIN_NONE
    );

    // We render a fixed 6x10 grid for tag IDs 1..60
    static constexpr uint8_t GRID_ROWS = 6;
    static constexpr uint8_t GRID_COLS = 10;
    static constexpr uint8_t GRID_TAGS = GRID_ROWS * GRID_COLS; // 60

    // -------------------------
    // INIT ("heard") tracking
    // -------------------------
    static bool     s_seen_ever[GRID_TAGS + 1];       // 1..60
    static uint32_t s_last_heard_ms[GRID_TAGS + 1];   // 1..60
    static uint32_t s_last_flash_ms[GRID_TAGS + 1];   // 1..60

    // -------------------------
    // RACE ("pass complete") tracking
    // -------------------------
    static bool     s_pass_done[GRID_TAGS + 1];       // 1..60
    static uint32_t s_pass_flash_ms[GRID_TAGS + 1];   // 1..60

    inline bool in_grid(uint16_t tag_id) {
        return tag_id >= 1 && tag_id <= GRID_TAGS;
    }

    inline void note_tag_heard(uint16_t tag_id, uint32_t now_ms) {
        if (!in_grid(tag_id)) return;
        s_seen_ever[tag_id]     = true;
        s_last_heard_ms[tag_id] = now_ms;
        s_last_flash_ms[tag_id] = now_ms;
    }

    inline void note_pass(uint16_t tag_id) {
        if (!in_grid(tag_id)) return;
        if (!s_pass_done[tag_id]) {
            s_pass_done[tag_id] = true;
        }
        s_pass_flash_ms[tag_id] = millis();
    }

    inline void reset_race() {
        for (uint8_t i = 1; i <= GRID_TAGS; i++) {
            s_pass_done[i] = false;
            s_pass_flash_ms[i] = 0;
        }
    }

    // Optional: if you want a "new meet" hard reset for self-check too
    inline void reset_selfcheck() {
        for (uint8_t i = 1; i <= GRID_TAGS; i++) {
            s_seen_ever[i] = false;
            s_last_heard_ms[i] = 0;
            s_last_flash_ms[i] = 0;
        }
    }

    inline void init() {
        Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
        u8g2.begin();

        u8g2.setFont(u8g2_font_5x7_tf);

        for (uint8_t i = 1; i <= GRID_TAGS; i++) {
            s_seen_ever[i] = false;
            s_last_heard_ms[i] = 0;
            s_last_flash_ms[i] = 0;
            s_pass_done[i] = false;
            s_pass_flash_ms[i] = 0;
        }

        u8g2.clearBuffer();
        u8g2.setCursor(0, 8);
        u8g2.print(F("XC Logger"));
        u8g2.sendBuffer();
    }

    inline bool is_recent(uint8_t tag_id, uint32_t now_ms, uint32_t window_ms) {
        if (!s_seen_ever[tag_id]) return false;
        const uint32_t age = now_ms - s_last_heard_ms[tag_id];
        return age <= window_ms;
    }

    inline bool is_flashing_heard(uint8_t tag_id, uint32_t now_ms) {
        if (!s_seen_ever[tag_id]) return false;
        return (now_ms - s_last_flash_ms[tag_id]) <= 250;
    }

    inline bool is_flashing_pass(uint8_t tag_id, uint32_t now_ms) {
        if (!s_pass_done[tag_id]) return false;
        return (now_ms - s_pass_flash_ms[tag_id]) <= 350;
    }

    inline void format_line1(char *out, size_t out_sz, char ntp_c, bool sd_present) {
        char datebuf[12];
        char timebuf[12];

        RtcTime::format_date_local(datebuf, sizeof(datebuf));
        RtcTime::format_time_local(timebuf, sizeof(timebuf));

        char mmdd[6] = {0};
        strncpy(mmdd, datebuf, 5);
        mmdd[5] = '\0';

        snprintf(out, out_sz, "%s %s N%c SD=%c", mmdd, timebuf, ntp_c, sd_present ? 'Y' : 'N');
    }

    inline void format_line2_selfcheck(char *out, size_t out_sz, uint32_t now_ms, uint8_t nn, uint32_t recent_window_ms) {
        uint8_t recent_cnt = 0;
        for (uint8_t i = 1; i <= nn; i++) {
            if (is_recent(i, now_ms, recent_window_ms)) recent_cnt++;
        }

        char miss[20];
        miss[0] = '\0';

        bool first = true;
        for (uint8_t i = 1; i <= nn; i++) {
            if (!s_seen_ever[i]) {
                char tmp[6];
                snprintf(tmp, sizeof(tmp), "%s%u", first ? "" : " ", i);
                strncat(miss, tmp, sizeof(miss) - strlen(miss) - 1);
                first = false;
            }
        }
        if (first) {
            strncpy(miss, "-", sizeof(miss));
            miss[sizeof(miss) - 1] = '\0';
        }

        snprintf(out, out_sz, "Tags:%2u/%u M=%s", recent_cnt, nn, miss);
    }

    inline void format_line2_race(char *out, size_t out_sz, uint32_t passes, uint32_t samples_total, uint8_t nn) {
        const uint32_t kpkts = samples_total / 1000UL;
        snprintf(out, out_sz, "Passes:%lu/%u kpkts=%lu",
                 (unsigned long)passes, (unsigned)nn, (unsigned long)kpkts);
    }

    inline void grid_layout(uint8_t& grid_x0, uint8_t& grid_y0, uint8_t& cell_w, uint8_t& cell_h) {
        static constexpr uint8_t LINE_H = 8;
        static constexpr uint8_t TOP_LINES = 2;

        grid_y0 = TOP_LINES * LINE_H;   // 16
        const uint8_t grid_h = 64 - grid_y0;          // 48
        cell_h = grid_h / GRID_ROWS;                  // 8

        cell_w = 12;                                  // 10*12 = 120
        grid_x0 = 4;                                  // left margin
    }

    inline void draw_grid_heard(uint32_t now_ms, uint32_t recent_window_ms) {
        uint8_t grid_x0, grid_y0, cell_w, cell_h;
        grid_layout(grid_x0, grid_y0, cell_w, cell_h);

        static constexpr uint8_t BOX_W = 8;
        static constexpr uint8_t BOX_H = 6;

        for (uint8_t r = 0; r < GRID_ROWS; r++) {
            for (uint8_t c = 0; c < GRID_COLS; c++) {
                const uint8_t id = (r * GRID_COLS) + c + 1;

                const uint8_t x_cell = grid_x0 + c * cell_w;
                const uint8_t y_cell = grid_y0 + r * cell_h;

                const uint8_t x = x_cell + 2;
                const uint8_t y = y_cell + 1;

                const bool ever   = s_seen_ever[id];
                const bool recent = is_recent(id, now_ms, recent_window_ms);
                const bool flash  = is_flashing_heard(id, now_ms);

                if (!ever) {
                    u8g2.drawFrame(x, y, BOX_W, BOX_H);
                } else if (recent) {
                    u8g2.drawBox(x, y, BOX_W, BOX_H);
                    if (flash) u8g2.drawFrame(x, y, BOX_W, BOX_H);
                } else {
                    u8g2.drawFrame(x, y, BOX_W, BOX_H);
                    u8g2.drawPixel(x + BOX_W/2, y + BOX_H/2);
                }
            }
        }
    }

    inline void draw_grid_pass(uint32_t now_ms) {
        uint8_t grid_x0, grid_y0, cell_w, cell_h;
        grid_layout(grid_x0, grid_y0, cell_w, cell_h);

        static constexpr uint8_t BOX_W = 8;
        static constexpr uint8_t BOX_H = 6;

        for (uint8_t r = 0; r < GRID_ROWS; r++) {
            for (uint8_t c = 0; c < GRID_COLS; c++) {
                const uint8_t id = (r * GRID_COLS) + c + 1;

                const uint8_t x_cell = grid_x0 + c * cell_w;
                const uint8_t y_cell = grid_y0 + r * cell_h;

                const uint8_t x = x_cell + 2;
                const uint8_t y = y_cell + 1;

                const bool done  = s_pass_done[id];
                const bool flash = is_flashing_pass(id, now_ms);

                if (!done) {
                    u8g2.drawFrame(x, y, BOX_W, BOX_H);
                } else {
                    u8g2.drawBox(x, y, BOX_W, BOX_H);
                    if (flash) u8g2.drawFrame(x, y, BOX_W, BOX_H);
                }
            }
        }
    }

    inline void update_selfcheck(uint32_t now_ms,
                                uint8_t  nn,
                                uint32_t recent_window_ms,
                                char     ntp_indicator,
                                bool     sd_present)
    {
        if (nn < 1) nn = 1;
        if (nn > GRID_TAGS) nn = GRID_TAGS;

        char line1[32];
        char line2[32];

        format_line1(line1, sizeof(line1), ntp_indicator, sd_present);
        format_line2_selfcheck(line2, sizeof(line2), now_ms, nn, recent_window_ms);

        u8g2.clearBuffer();

        u8g2.setCursor(0, 8);
        u8g2.print(line1);

        u8g2.setCursor(0, 16);
        u8g2.print(line2);

        draw_grid_heard(now_ms, recent_window_ms);

        u8g2.sendBuffer();
    }

    inline void update_race(uint32_t now_ms,
                            uint8_t  nn,
                            char     ntp_indicator,
                            bool     sd_present,
                            uint32_t passes,
                            uint32_t samples_total)
    {
        if (nn < 1) nn = 1;
        if (nn > GRID_TAGS) nn = GRID_TAGS;

        char line1[32];
        char line2[32];

        format_line1(line1, sizeof(line1), ntp_indicator, sd_present);
        format_line2_race(line2, sizeof(line2), passes, samples_total, nn);

        u8g2.clearBuffer();

        u8g2.setCursor(0, 8);
        u8g2.print(line1);

        u8g2.setCursor(0, 16);
        u8g2.print(line2);

        draw_grid_pass(now_ms);

        u8g2.sendBuffer();
    }

} // namespace StatusDisplay
