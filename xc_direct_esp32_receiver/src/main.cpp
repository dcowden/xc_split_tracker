#include <Arduino.h>
#include <ArduinoLog.h>
#include <HardwareSerial.h>
#include <Ticker.h>

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
    .ema_alpha       = 0.30f,
    .threshold_dbm   = -50,
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
// Display ticker (polled)
// ---------------------------
static void on_display_tick();
static Ticker g_display_ticker(on_display_tick, DISPLAY_INTERVAL_MS, 0, MILLIS);

static void on_display_tick() {
    StatusDisplay::update(g_total_events, g_total_samples, pass_get_tag_count());
}

// ---------------------------
// Event sink -> events.csv
// ---------------------------
static void on_event(const Event &e) {
    uint64_t unix_ms = RtcTime::ms_to_unix_ms(millis());
    SdLogger::log_event(unix_ms, e);
}

// ---------------------------
// Sample callback (every UART line)
// ---------------------------
static void on_sample(uint32_t rel_ms, uint16_t tag_id, int8_t rssi) {
    const uint32_t now_wall_ms = millis();
    const uint64_t unix_ms     = RtcTime::ms_to_unix_ms(now_wall_ms);

    // Pass id for raw logging (0 if not in pass yet)
    TagContext *ctx = pass_get_ctx(tag_id);
    uint32_t pass_id = (ctx && ctx->in_pass) ? ctx->pass_id : 0;

    // HOT PATH: write to RingBuf (non-blocking-ish)
    SdLogger::log_raw_sample(unix_ms, rel_ms, tag_id, pass_id, rssi);
    g_total_samples++;

    // Update EMA/pass logic (may emit events via sink)
    pass_process_sample(rel_ms, tag_id, rssi, now_wall_ms);
}

void setup() {
    Serial.begin(115200);
    delay(200);

    Log.begin(LOG_LEVEL_NOTICE, &Serial);
    Log.noticeln(F("ESP32 UART->SdFat RingBuf logger starting..."));

    // RTC (DS3231)
    RtcTime::init();

    // SD logger (SdFat + preAllocate + RingBuf)
    if (!SdLogger::init()) {
        Log.errorln(F("SdLogger::init FAILED - logging disabled"));
    } else {
        // ESP32 optimization: drain RingBuf in a background task (core 0)
        SdLogger::start_background_task();
    }

    // Pass processor
    pass_init();
    pass_set_event_sink(on_event);

    // UART line reader
    g_uart.begin();
    g_uart.set_callback(on_sample);

    // OLED display
    StatusDisplay::init();
    g_display_ticker.start();
}

void loop() {
    // Consume UART and call on_sample() per line
    g_uart.service();

    // End passes on idle timeout
    pass_check_idle_timeouts(millis());

    // Drive display updates (polled ticker)
    g_display_ticker.update();

    delay(1);
}
