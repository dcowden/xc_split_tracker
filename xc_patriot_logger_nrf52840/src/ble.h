#pragma once

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <bluefruit.h>
#include <SdFat.h>

#include "rtc_time.h"

// =======================================
// BLE UUIDs
// =======================================

constexpr uint16_t UUID_LOG_SERVICE   = 0x1234;
constexpr uint16_t UUID_LOG_INFO_CHAR = 0xFFF1;
constexpr uint16_t UUID_LOG_DATA_CHAR = 0xFFF2;
constexpr uint16_t UUID_LOG_CTRL_CHAR = 0xFFF3;
constexpr uint16_t UUID_RTC_CHAR      = 0xFFF4;

// =======================================
// BLE service & characteristics
// =======================================

static BLEService        logService(UUID_LOG_SERVICE);
static BLECharacteristic logInfoChar(UUID_LOG_INFO_CHAR);
static BLECharacteristic logDataChar(UUID_LOG_DATA_CHAR);
static BLECharacteristic logCtrlChar(UUID_LOG_CTRL_CHAR);
static BLECharacteristic rtcTimeChar(UUID_RTC_CHAR);

// file + size provided by main.cpp
static File32*  g_log_file        = nullptr;
static uint32_t g_log_size_bytes  = 0;
static bool     g_ble_streaming   = false;
static uint32_t g_ble_sent_bytes  = 0;

// Forward decls for callbacks
static void ctrl_write_cb(uint16_t conn_hdl, BLECharacteristic* chr,
                          uint8_t* data, uint16_t len);
static void rtc_write_cb(uint16_t conn_hdl, BLECharacteristic* chr,
                         uint8_t* data, uint16_t len);
static void connect_callback(uint16_t conn_hdl);
static void disconnect_callback(uint16_t conn_hdl, uint8_t reason);

// =======================================
// BLE init + start upload mode
// =======================================

static void ble_internal_init() {
    static bool initialized = false;
    if (initialized) return;

    Bluefruit.begin();
    Bluefruit.setTxPower(4);
    Bluefruit.setName("XCLOGGER-01"); // adjust per box later

    Bluefruit.Periph.setConnectCallback(connect_callback);
    Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

    // Service & chars
    logService.begin();

    // Log size info (4-byte LE)
    logInfoChar.setProperties(CHR_PROPS_READ);
    logInfoChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    logInfoChar.setFixedLen(4);
    logInfoChar.begin();

    // Data: notify-only
    logDataChar.setProperties(CHR_PROPS_NOTIFY);
    logDataChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    logDataChar.setMaxLen(200);
    logDataChar.begin();

    // Control: 1-byte read/write
    logCtrlChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_READ);
    logCtrlChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    logCtrlChar.setFixedLen(1);
    logCtrlChar.setWriteCallback(ctrl_write_cb);
    logCtrlChar.begin();

    // RTC time: 4-byte unix seconds, read/write
    rtcTimeChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_READ);
    rtcTimeChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    rtcTimeChar.setFixedLen(4);
    rtcTimeChar.setWriteCallback(rtc_write_cb);
    rtcTimeChar.begin();

    initialized = true;
}

static void ble_advertise_start() {
    Bluefruit.Advertising.stop();
    Bluefruit.Advertising.clearData();

    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addService(logService);
    Bluefruit.Advertising.addName();

    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244); // 20ms - 152.5ms
    Bluefruit.Advertising.setFastTimeout(30);
    Bluefruit.Advertising.start();
}

// Called by main.cpp when log file is finalized and opened read-only
inline void ble_start_upload(File32* file, uint32_t logSize) {
    g_log_file       = file;
    g_log_size_bytes = logSize;
    g_ble_streaming  = false;
    g_ble_sent_bytes = 0;

    ble_internal_init();

    // Seed log size into info char
    uint8_t sizeBuf[4];
    sizeBuf[0] = (uint8_t)(g_log_size_bytes & 0xFF);
    sizeBuf[1] = (uint8_t)((g_log_size_bytes >> 8) & 0xFF);
    sizeBuf[2] = (uint8_t)((g_log_size_bytes >> 16) & 0xFF);
    sizeBuf[3] = (uint8_t)((g_log_size_bytes >> 24) & 0xFF);
    logInfoChar.write(sizeBuf, 4);

    // Seed RTC char with current unix seconds if we have it
    uint32_t now_unix = 0;
    if (RtcTime::is_ok()) {
        DateTime now_dt = RtcTime::now();
        now_unix = now_dt.unixtime();
    }
    uint8_t rtcBuf[4];
    rtcBuf[0] = (uint8_t)(now_unix & 0xFF);
    rtcBuf[1] = (uint8_t)((now_unix >> 8) & 0xFF);
    rtcBuf[2] = (uint8_t)((now_unix >> 16) & 0xFF);
    rtcBuf[3] = (uint8_t)((now_unix >> 24) & 0xFF);
    rtcTimeChar.write(rtcBuf, 4);

    // Control = 0 (not streaming yet)
    uint8_t zero = 0;
    logCtrlChar.write(&zero, 1);

    ble_advertise_start();
}

// =======================================
// Callbacks
// =======================================

static void ctrl_write_cb(uint16_t conn_hdl, BLECharacteristic* chr,
                          uint8_t* data, uint16_t len) {
    if (len < 1) return;
    if (!g_log_file) return;

    if (data[0] == 0x01) {
        // Start streaming from beginning
        g_log_file->seekSet(0);
        g_ble_sent_bytes = 0;
        g_ble_streaming  = true;
    }
}

static void rtc_write_cb(uint16_t conn_hdl, BLECharacteristic* chr,
                         uint8_t* data, uint16_t len) {
    if (len < 4) return;

    uint32_t unix_sec =
        (uint32_t)data[0] |
        ((uint32_t)data[1] << 8) |
        ((uint32_t)data[2] << 16) |
        ((uint32_t)data[3] << 24);

    Serial.print(F("BLE: set RTC to unix "));
    Serial.println(unix_sec);

    RtcTime::set_rtc_from_unix(unix_sec);

    // Echo back
    chr->write(data, 4);
}

static void connect_callback(uint16_t conn_hdl) {
    Serial.println(F("BLE: connected"));
}

static void disconnect_callback(uint16_t conn_hdl, uint8_t reason) {
    Serial.print(F("BLE: disconnected, reason "));
    Serial.println(reason);
    g_ble_streaming  = false;
    g_ble_sent_bytes = 0;
}

// =======================================
// BLE streaming task
// =======================================

inline void ble_task() {
    if (!g_ble_streaming) return;
    if (!Bluefruit.connected()) {
        g_ble_streaming = false;
        return;
    }
    if (!g_log_file) {
        g_ble_streaming = false;
        return;
    }

    int32_t remain = (int32_t)g_log_size_bytes - (int32_t)g_ble_sent_bytes;
    if (remain <= 0) {
        Serial.println(F("BLE: stream complete"));
        g_ble_streaming = false;
        return;
    }

    uint8_t buf[180]; // keep below typical MTU
    uint16_t toRead = remain > (int32_t)sizeof(buf) ? sizeof(buf) : (uint16_t)remain;
    int32_t n = g_log_file->read(buf, toRead);
    if (n <= 0) {
        Serial.println(F("BLE: read error or EOF"));
        g_ble_streaming = false;
        return;
    }

    if (!logDataChar.notify(buf, (uint16_t)n)) {
        // Notify buffer full, try again next loop
        return;
    }

    g_ble_sent_bytes += (uint32_t)n;
}
