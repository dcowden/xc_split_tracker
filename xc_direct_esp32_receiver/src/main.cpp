// main.cpp
#include <Arduino.h>
#include <ArduinoLog.h>
#include <HardwareSerial.h>
#include <Ticker.h>
#include <OneButton.h>

#include "timesync.h"
#include "config.h"
#include "types.h"
#include "rtc_time.h"
#include "sd_logger.h"
#include "pass_processor.h"
#include "uart_input.h"
#include "display.h"

// ---------------------------
// Forward declarations
// ---------------------------
void on_event(const Event &e);

// ---------------------------
// Runtime config
// ---------------------------
RuntimeConfig g_cfg = {
    .ema_alpha       = 0.01f,
    .threshold_dbm   = -100,
    .hysteresis_db   = 2,
    .idle_end_ms     = 1000,
};

TagContext g_tags[MAX_TAGS];
volatile uint32_t g_total_events  = 0;  // passes (race only)
volatile uint32_t g_total_samples = 0;  // raw samples (race only)

// ---------------------------
// Modes
// ---------------------------
enum class AppMode : uint8_t { INIT = 0, RACE };
static volatile AppMode g_mode = AppMode::INIT;

// ---------------------------
// Self-check config
// ---------------------------
static constexpr uint8_t  SELFCHK_TAGS_NN          = 3;
static constexpr uint32_t SELFCHK_RECENT_WINDOW_MS = 1000;

// ---------------------------
// UART input
// ---------------------------
static HardwareSerial ExtSerial(EXT_UART_NUM);
static UartInput g_uart(ExtSerial);

// ---------------------------
// Button (OneButton)
// ---------------------------
#ifndef SD_TOGGLE_BTN_PIN
#define SD_TOGGLE_BTN_PIN 2
#endif
static OneButton g_btn(SD_TOGGLE_BTN_PIN, /*activeLow=*/true, /*pullup=*/true);

// SD present flag for display
static bool g_sd_present = false;

// ---------------------------
// Safe SD control (requests handled in loop)
// ---------------------------
enum class SdUserReq : uint8_t { NONE, REQ_MOUNT, REQ_UNMOUNT };
static volatile SdUserReq g_sd_req_isr = SdUserReq::NONE;
static SdUserReq g_sd_req = SdUserReq::NONE;

// While true, we stop feeding pass processor + stop logging.
static bool g_logging_paused = true;

// ---------------------------
// Mode helpers
// ---------------------------
static inline bool can_enter_race() {
    return SdLogger::is_mounted() && RtcTime::rtc_time_valid_now();
}

static void enter_init_mode() {
    if (g_mode == AppMode::INIT) return;

    Log.noticeln(F("Entering INIT mode: stopping pass processing + logging"));
    g_mode = AppMode::INIT;

    // Hard stop immediately (don’t wait for loop gating)
    g_logging_paused = true;

    // Optional: reset UI to selfcheck baseline; ensures no stale race UI is shown
    StatusDisplay::reset_selfcheck();   // If you don’t have this function, remove this line.

    // Optional: reset pass detector state so you don't carry in-pass state back into a future race
    pass_init();
    pass_set_event_sink(on_event);
}

static void enter_race_mode() {
    if (!can_enter_race()) {
        Log.warningln(F("START blocked: need SD mounted and valid date/time"));
        return;
    }

    Log.noticeln(F("Entering RACE mode: clearing counters + pass state"));
    g_total_events  = 0;
    g_total_samples = 0;

    // Reset race pass completion grid
    StatusDisplay::reset_race();

    // Reset pass detector state to avoid stale in-pass data causing instant events
    pass_init();
    pass_set_event_sink(on_event);

    g_mode = AppMode::RACE;
}

// ---------------------------
// Display ticker (polled)
// ---------------------------
static void on_display_tick();
static Ticker g_display_ticker(on_display_tick, DISPLAY_INTERVAL_MS, 0, MILLIS);

static void on_display_tick() {
    const char ntp_c = TimeSync::indicator_char();

    if (g_mode == AppMode::INIT) {
        StatusDisplay::update_selfcheck(
            millis(),
            SELFCHK_TAGS_NN,
            SELFCHK_RECENT_WINDOW_MS,
            ntp_c,
            g_sd_present
        );
    } else {
        StatusDisplay::update_race(
            millis(),
            StatusDisplay::GRID_TAGS, // 60
            ntp_c,
            g_sd_present,
            (uint32_t)pass_get_completed_count(),
            (uint32_t)g_total_samples
        );
    }
}

// ---------------------------
// Event sink -> events.csv
// Use receiver timeline: unix_ms = base_unix + (e.rel_ms - base_rel)
// ---------------------------
void on_event(const Event &e) {
    if (g_mode != AppMode::RACE) return;
    if (g_logging_paused) return;

    int64_t unix_ms = g_uart.unix_from_rel(e.rel_ms);
    if (unix_ms == 0) unix_ms = RtcTime::unix_now_ms();
    if (unix_ms <= 0) return;

    SdLogger::log_event((uint64_t)unix_ms, e);
    g_total_events++;
}

// ---------------------------
// Sample callback (every UART line)
// NEW signature: includes unix_ms
// ---------------------------
static void on_sample(uint32_t rel_ms, int64_t unix_ms, uint16_t tag_id, uint8_t seq, int8_t rssi) {
    (void)seq;

    // Always record tag "heard" for the init UI
    StatusDisplay::note_tag_heard(tag_id, millis());

    // INIT: do NOT process passes, do NOT count, do NOT write to SD
    if (g_mode != AppMode::RACE) return;
    if (g_logging_paused) return;

    if (unix_ms <= 0) unix_ms = RtcTime::unix_now_ms();
    if (unix_ms <= 0) return;

    TagContext *ctx = pass_get_ctx(tag_id);
    const uint32_t pass_id = (ctx && ctx->in_pass) ? ctx->pass_id : 0;

    SdLogger::log_raw_sample((uint64_t)unix_ms, rel_ms, tag_id, pass_id, rssi);
    g_total_samples++;

    pass_process_sample(rel_ms, tag_id, rssi, millis());
}

// ---------------------------
// Button handlers
// ---------------------------
static void on_btn_click() {
    g_sd_req_isr = SdLogger::is_mounted() ? SdUserReq::REQ_UNMOUNT : SdUserReq::REQ_MOUNT;
}

static void on_btn_longpress_start() {
    // Long press toggles mode
    if (g_mode == AppMode::INIT) {
        enter_race_mode();
    } else {
        enter_init_mode();
    }
}

// ---------------------------
// Setup
// ---------------------------
void setup() {
    Serial.begin(115200);
    delay(200);

    setenv("TZ", "EST5EDT,M3.2.0/2,M11.1.0/2", 1);
    tzset();

    Log.begin(LOG_LEVEL_NOTICE, &Serial);
    Log.noticeln(F("ESP32 XC receiver starting..."));

    // RTC init + seed system time BEFORE SD mount (prevents 1970 file timestamps)
    Log.error(F("Rtc:: "));
    RtcTime::init();
    Log.errorln(F("[OK]"));
    RtcTime::seed_system_time_from_rtc();

    // Pass processor (initialized, but we won't feed it until RACE)
    pass_init();
    pass_set_event_sink(on_event);

    // UART reader
    g_uart.begin();
    g_uart.set_unix_now_fn(RtcTime::unix_now_ms);
    g_uart.set_callback(on_sample);

    // OLED display
    Log.error(F("Display:: "));
    StatusDisplay::init();
    Log.errorln(F("[OK]"));

    // Button
    g_btn.attachClick(on_btn_click);
    g_btn.attachLongPressStart(on_btn_longpress_start);
    g_btn.setPressMs(800);

    // Non-blocking NTP sync attempt immediately
    Log.noticeln(F("NTP: starting non-blocking sync attempt"));
    TimeSync::begin("bluedirt_IOT_2G", "allthethings!", 10000);

    // Request mount at boot (safe now because system time is seeded from RTC)
    Log.noticeln(F("SdLogger: requesting mount at boot"));
    SdLogger::mount();

    // Start in INIT mode (no logging)
    g_mode = AppMode::INIT;
    g_logging_paused = true;

    // Display ticker
    g_display_ticker.start();
}

// ---------------------------
// Loop
// ---------------------------
void loop() {
    g_btn.tick();

    // Latch request from ISR callback
    if (g_sd_req_isr != SdUserReq::NONE) {
        g_sd_req = g_sd_req_isr;
        g_sd_req_isr = SdUserReq::NONE;
    }

    // Handle SD requests
    if (g_sd_req != SdUserReq::NONE) {
        if (g_sd_req == SdUserReq::REQ_UNMOUNT) {
            Log.noticeln(F("SD: user requested UNMOUNT"));
            SdLogger::unmount();
            g_logging_paused = true;
        } else {
            Log.noticeln(F("SD: user requested MOUNT"));
            SdLogger::mount();
        }
        g_sd_req = SdUserReq::NONE;
    }

    // SD + NTP state machines
    SdLogger::service();
    g_sd_present = SdLogger::is_mounted();

    TimeSync::service();

    // If NTP finished successfully, apply to system time + RTC once
    int64_t ntp_ms = 0;
    if (TimeSync::consume_success_time_ms(ntp_ms)) {
        Log.noticeln(F("NTP: applying time to system + RTC"));
        RtcTime::apply_system_time_ms(ntp_ms);
        RtcTime::set_from_unix_ms(ntp_ms);
    }

    // Decide if logging/pass processing should run
    if (g_mode == AppMode::RACE) {
        g_logging_paused = !(g_sd_present && RtcTime::rtc_time_valid_now());
    } else {
        g_logging_paused = true;
    }

    // UART service (always)
    g_uart.service();

    // Pass idle timeout only when in race and not paused
    if (g_mode == AppMode::RACE && !g_logging_paused) {
        pass_check_idle_timeouts(millis());
    }

    // Display refresh
    g_display_ticker.update();
    delay(1);
}
