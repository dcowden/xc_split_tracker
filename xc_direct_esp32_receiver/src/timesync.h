// TimeSync.h
#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include <sys/time.h>
#include <ArduinoLog.h>
namespace TimeSync {

  // ============================================================
  // Public API
  // ============================================================
  // Returns Unix epoch time in milliseconds, or -1 on failure.
  // Assumes WiFi is OFF initially and turns it OFF before returning.
  inline int64_t fetchTimeMs(const char* ssid,
                             const char* password,
                             uint32_t timeoutMs,
                             const char* ntp1 = "pool.ntp.org",
                             const char* ntp2 = "time.nist.gov",
                             const char* ntp3 = "time.google.com") {

    Log.noticeln(F("Syncing Time..."));
    if (!ssid || !*ssid || !password){
        Log.warningln(F("Can't sync: ssid, password not populated"));                            
        return -1;
    } 

    const uint32_t startMs    = millis();
    const uint32_t deadlineMs = startMs + timeoutMs;

    // ------------------------------------------------------------
    // Helpers (inline so this remains header-only)
    // ------------------------------------------------------------
    auto wifiCleanup = []() {
      WiFi.disconnect(true /*wifioff*/, true /*eraseAP*/);
      WiFi.mode(WIFI_OFF);
      delay(20);
      Log.noticeln(F("Wifi Disconnected."));  
    };

    auto ssidIsVisible = [&](const char* target) -> bool {
      int n = WiFi.scanNetworks(false /*sync*/, true /*show hidden*/);
      if (n <= 0) return false;
      for (int i = 0; i < n; i++) {
        if (millis() > deadlineMs) break;
        if (WiFi.SSID(i).equals(target)) return true;
      }
      return false;
    };

    auto connectWifi = [&](const char* s, const char* p) -> bool {
      WiFi.persistent(false);     // don't touch flash
      WiFi.setAutoReconnect(false);
      WiFi.mode(WIFI_STA);
      WiFi.disconnect(true, true);
      delay(50);

      WiFi.begin(s, p);
      while (WiFi.status() != WL_CONNECTED) {
        if (millis() > deadlineMs) return false;
        delay(25);
      }
      return true;
    };

    auto waitForTimeSync = [&]() -> bool {
      // Anything after 2020-01-01 counts as "real"
      constexpr time_t MIN_VALID_TIME = 1577836800;
      while (true) {
        if (time(nullptr) >= MIN_VALID_TIME) return true;
        if (millis() > deadlineMs) return false;
        delay(25);
      }
    };

    auto unixMsNow = []() -> int64_t {
      struct timeval tv;
      gettimeofday(&tv, nullptr);
      return (int64_t)tv.tv_sec * 1000LL + (tv.tv_usec / 1000);
    };

    // ------------------------------------------------------------
    // (a) Bring WiFi up and scan
    // ------------------------------------------------------------
    Log.noticeln(F("Looking for Wifi.."));
    WiFi.mode(WIFI_STA);
    delay(10);

    if (!ssidIsVisible(ssid)) {
      wifiCleanup();
      Log.warningln(F("Cant find wifi network"));
      return -1;
    }

    // ------------------------------------------------------------
    // (b) Connect
    // ------------------------------------------------------------
    if (!connectWifi(ssid, password)) {
      Log.warningln(F("Cant connect to wifi."));  
      wifiCleanup();
      return -1;
    }
    Log.noticeln(F("Connected. Syncing NTP"));
    // ------------------------------------------------------------
    // (c) NTP sync (UTC)
    // ------------------------------------------------------------
    //configTime(0, 0, ntp1, ntp2, ntp3);
    configTzTime("EST5EDT,M3.2.0/2,M11.1.0/2", ntp1, ntp2, ntp3);


    if (!waitForTimeSync()) {
      Log.warningln(F("NTP Sync timed out."));  
      wifiCleanup();
      return -1;
    }

    int64_t resultMs = unixMsNow();
    Log.warningln(F("Received Network time OK"));
    // ------------------------------------------------------------
    // Always shut WiFi back down
    // ------------------------------------------------------------
    wifiCleanup();
    return resultMs;
  }

} // namespace TimeSync
