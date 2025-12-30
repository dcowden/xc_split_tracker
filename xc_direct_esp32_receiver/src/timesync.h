// timesync.h  (non-blocking, connect-first; NO scanning)
// Header-only, call TimeSync::begin(...) once, then TimeSync::service() frequently.
// Use TimeSync::indicator_char() for UI: '?' running, 'Y' success, 'X' fail.
#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include <ArduinoLog.h>

namespace TimeSync {

    enum class State : uint8_t {
        IDLE = 0,
        CONNECTING,
        START_NTP,
        WAIT_TIME,
        CLEANUP,
        DONE_OK,
        DONE_FAIL
    };

    // Config
    static const char* s_ssid = nullptr;
    static const char* s_pw   = nullptr;
    static uint32_t    s_timeout_ms = 0;

    static const char* s_ntp1 = "pool.ntp.org";
    static const char* s_ntp2 = "time.nist.gov";
    static const char* s_ntp3 = "time.google.com";

    static State       s_state = State::IDLE;
    static uint32_t    s_start_ms = 0;
    static uint32_t    s_deadline_ms = 0;

    static bool        s_success_latched = false;
    static bool        s_success_consumed = true;
    static int64_t     s_result_ms = -1;

    // "time is real" threshold
    static constexpr time_t MIN_VALID_TIME = 1577836800; // 2020-01-01

    inline void wifi_cleanup() {
        WiFi.disconnect(true /*wifioff*/, true /*eraseAP*/);
        WiFi.mode(WIFI_OFF);
        Log.noticeln(F("TimeSync: disconnected wifi"));
        delay(10);
    }

    inline void reset_runtime() {
        s_result_ms = -1;
        s_success_latched = false;
        s_success_consumed = true;
    }

    inline bool timed_out() {
        return (int32_t)(millis() - s_deadline_ms) > 0;
    }

    // ------------------------------------------------------------
    // Public API
    // ------------------------------------------------------------
    inline void begin(const char* ssid,
                      const char* password,
                      uint32_t timeoutMs,
                      const char* ntp1 = "pool.ntp.org",
                      const char* ntp2 = "time.nist.gov",
                      const char* ntp3 = "time.google.com")
    {
        // If already running, ignore
        if (s_state != State::IDLE && s_state != State::DONE_OK && s_state != State::DONE_FAIL) return;

        s_ssid = ssid;
        s_pw   = password;
        s_timeout_ms = timeoutMs;

        s_ntp1 = ntp1;
        s_ntp2 = ntp2;
        s_ntp3 = ntp3;

        reset_runtime();

        if (!s_ssid || !*s_ssid || !s_pw) {
            Log.warningln(F("TimeSync: ssid/password missing"));
            s_state = State::DONE_FAIL;
            return;
        }

        s_start_ms = millis();
        s_deadline_ms = s_start_ms + s_timeout_ms;

        WiFi.persistent(false);
        WiFi.setAutoReconnect(false);
        WiFi.mode(WIFI_STA);
        WiFi.setSleep(false);

        WiFi.disconnect(true, true);
        delay(20);

        Log.noticeln(F("TimeSync: connecting (no scan)..."));
        WiFi.begin(s_ssid, s_pw);

        s_state = State::CONNECTING;
    }

    inline void service() {
        if (s_state == State::IDLE || s_state == State::DONE_OK || s_state == State::DONE_FAIL) return;

        if (timed_out()) {
            Log.warningln(F("TimeSync: timeout"));
            s_state = State::CLEANUP;
        }

        switch (s_state) {

            case State::CONNECTING: {
                const wl_status_t st = WiFi.status();
                if (st == WL_CONNECTED) {
                    Log.noticeln(F("TimeSync: connected, starting NTP"));
                    s_state = State::START_NTP;
                } else if (st == WL_CONNECT_FAILED || st == WL_NO_SSID_AVAIL) {
                    Log.warningln(F("TimeSync: connect failed / no ssid"));
                    s_state = State::CLEANUP;
                }
                return;
            }

            case State::START_NTP: {
                configTzTime("EST5EDT,M3.2.0/2,M11.1.0/2", s_ntp1, s_ntp2, s_ntp3);
                s_state = State::WAIT_TIME;
                return;
            }

            case State::WAIT_TIME: {
                // Wait for system time to become "real"
                time_t t = time(nullptr);
                if (t >= MIN_VALID_TIME) {
                    s_result_ms = (int64_t)t * 1000LL; // validated source, no gettimeofday dependency
                    s_success_latched = true;
                    s_success_consumed = false;
                    Log.noticeln(F("TimeSync: NTP OK"));
                    s_state = State::CLEANUP;
                }
                return;
            }

            case State::CLEANUP: {
                wifi_cleanup();
                s_state = s_success_latched ? State::DONE_OK : State::DONE_FAIL;
                return;
            }

            default:
                return;
        }
    }

    inline char indicator_char() {
        switch (s_state) {
            case State::DONE_OK:   return 'Y';
            case State::DONE_FAIL: return 'X';
            case State::IDLE:      return 'X';
            default:               return '?';
        }
    }

    // Returns true exactly once when a successful time is ready
    inline bool consume_success_time_ms(int64_t &out_ms) {
        if (!s_success_latched) return false;
        if (s_success_consumed) return false;
        out_ms = s_result_ms;
        s_success_consumed = true;
        return true;
    }

} // namespace TimeSync
