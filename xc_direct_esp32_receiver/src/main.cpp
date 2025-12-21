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
// Runtime config
// ---------------------------
RuntimeConfig g_cfg = {
    .ema_alpha       = 0.10f,
    .threshold_dbm   = -85,
    .hysteresis_db   = 2,
    .idle_end_ms     = 1000,
};

TagContext g_tags[MAX_TAGS];
volatile uint32_t g_total_events  = 0;
volatile uint32_t g_total_samples = 0;

// ---------------------------
// UART input
// ---------------------------
static HardwareSerial ExtSerial(EXT_UART_NUM);
static UartInput g_uart(ExtSerial);

// ---------------------------
// SD toggle button (OneButton)
// ---------------------------
#ifndef SD_TOGGLE_BTN_PIN
#define SD_TOGGLE_BTN_PIN 2   // set to your actual pin
#endif

static OneButton g_sd_button(SD_TOGGLE_BTN_PIN, /*activeLow=*/true, /*pullup=*/true);

// SD present flag for display
static bool g_sd_present = false;

// ---------------------------
// Safe SD control (requests handled in loop)
// ---------------------------
enum class SdUserReq : uint8_t { NONE, REQ_MOUNT, REQ_UNMOUNT };
static volatile SdUserReq g_sd_req_isr = SdUserReq::NONE; // set by button callback
static SdUserReq g_sd_req = SdUserReq::NONE;              // latched in loop

// While true, we stop feeding pass processor + stop logging.
static bool g_logging_paused = false;

// ---------------------------
// Display ticker (polled)
// ---------------------------
static void on_display_tick();
static Ticker g_display_ticker(on_display_tick, DISPLAY_INTERVAL_MS, 0, MILLIS);

static void on_display_tick() {
    StatusDisplay::update(
        g_total_events,
        g_total_samples,
        pass_get_tag_count(),
        g_sd_present
    );
}

// ---------------------------
// Event sink -> events.csv
// ---------------------------
static void on_event(const Event &e) {
    if (g_logging_paused) return;

    const uint64_t unix_ms = RtcTime::ms_to_unix_ms(millis());
    SdLogger::log_event(unix_ms, e);
    g_total_events++;
}

// ---------------------------
// Sample callback (every UART line)
// ---------------------------
static void on_sample(uint32_t rel_ms, uint16_t tag_id, uint8_t seq, int8_t rssi) {
    (void)seq;

    if (g_logging_paused) return;

    const uint32_t now_wall_ms = millis();
    const uint64_t unix_ms     = RtcTime::ms_to_unix_ms(now_wall_ms);

    TagContext *ctx = pass_get_ctx(tag_id);
    const uint32_t pass_id = (ctx && ctx->in_pass) ? ctx->pass_id : 0;

    SdLogger::log_raw_sample(unix_ms, rel_ms, tag_id, pass_id, rssi);
    g_total_samples++;

    pass_process_sample(rel_ms, tag_id, rssi, now_wall_ms);
}

// ---------------------------
// SD toggle handler (NO SD ops here)
// ---------------------------
static void on_sd_toggle_click() {
    // Toggle based on current UI-mounted state.
    // IMPORTANT: Don't call SdLogger functions from callback.
    g_sd_req_isr = SdLogger::is_mounted() ? SdUserReq::REQ_UNMOUNT : SdUserReq::REQ_MOUNT;
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
    Log.noticeln(F("ESP32 UART->SdFat RingBuf logger starting..."));

    // RTC
    Log.error(F("Rtc:: "));
    RtcTime::init();
    Log.errorln(F("[OK]"));

    // Pass processor
    pass_init();
    pass_set_event_sink(on_event);

    // UART reader
    g_uart.begin();
    g_uart.set_callback(on_sample);

    // Sync time
    Log.error(F("NTP: "));
    int64_t t = TimeSync::fetchTimeMs("bluedirt_IOT_2G", "allthethings!", 10000);
    if (t == -1) {
        Log.errorln(F(" [FAIL]"));
    } else {
        RtcTime::set_from_unix_ms(t);
        Log.errorln(F(" [OK]"));
    }

    // OLED display
    Log.error(F("Display:: "));
    StatusDisplay::init();
    Log.errorln(F("Display:: [OK]"));

    delay(1000);

    // OneButton
    g_sd_button.attachClick(on_sd_toggle_click);

    // Request mount at boot (non-blocking: SdLogger::service() performs it)
    Log.noticeln(F("SdLogger: requesting mount at boot"));
    SdLogger::mount();

    // Display ticker
    g_display_ticker.start();
}

// ---------------------------
// Loop
// ---------------------------
void loop() {
    // Button
    g_sd_button.tick();

    // Latch request from callback
    if (g_sd_req_isr != SdUserReq::NONE) {
        g_sd_req = g_sd_req_isr;
        g_sd_req_isr = SdUserReq::NONE;
    }

    // Handle SD requests (simple with stepper logger)
    if (g_sd_req != SdUserReq::NONE) {
        if (g_sd_req == SdUserReq::REQ_UNMOUNT) {
            Log.noticeln(F("SD: user requested UNMOUNT"));
            SdLogger::unmount();
        } else { // REQ_MOUNT
            Log.noticeln(F("SD: user requested MOUNT"));
            SdLogger::mount();
        }
        g_sd_req = SdUserReq::NONE;
    }

    // Always run SD state machine frequently (ONE step per call)
    //this takes 2 seconds when a card isn there. 
    //solution : modify sdfat timeout (default is 2s)
    SdLogger::service();

    // UI-mounted state for display
    g_sd_present = SdLogger::is_mounted();

    // Pause logging unless SD is mounted by your UI rule
    g_logging_paused = !g_sd_present;

    // UART service:
    // - Always service UART to avoid buffer overrun
    // - callback drops samples if paused
    g_uart.service();

    // Pass idle timeout: only meaningful when not paused (otherwise it could emit events)
    if (!g_logging_paused) {
        pass_check_idle_timeouts(millis());
    }

    // Display refresh
    g_display_ticker.update();

    delay(1);
}
