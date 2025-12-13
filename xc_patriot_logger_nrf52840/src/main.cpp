#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <nrf.h>

#include <Wire.h>
#include <RTClib.h>
#include <ArduinoLog.h>

#ifndef DISABLE_FS_H_WARNING
#define DISABLE_FS_H_WARNING
#endif
#include <SdFat.h>
#include "RingBuf.h"

#include "rtc_time.h"   // the header shown above

// =======================================
// CONFIG
// =======================================

// XIAO nRF52840 SD chip select
constexpr uint8_t SD_CS_PIN = D3;

// Log file name
#define LOG_FILENAME "rawlog.csv"

// Pre-allocated log file size in bytes (adjust as needed)
//const uint32_t LOG_FILE_SIZE = 16UL * 1024UL * 1024UL; // 16 MB
const uint32_t LOG_FILE_SIZE = 0; // 16 MB
// Ring buffer size in bytes (must be multiple of 512)
static const size_t RING_BUF_SIZE = 16 * 512; // 8KB

// We will sync directory entry every N ms
static const uint32_t SYNC_INTERVAL_MS = 1000;

// =======================================
// Radio receive (low-level NRF RADIO)
// =======================================

#define PACKET_LEN 4

typedef struct __attribute__((packed)) {
    uint8_t  tag_id;
    uint8_t  seq;
    uint16_t reserved;
} xc_packet_t;

// Live buffer used by RADIO
static volatile xc_packet_t rx_packet;

// ISR → main mailbox (single-slot)
static volatile bool     packet_ready    = false;
static volatile uint8_t  packet_tag_id   = 0;
static volatile int8_t   packet_rssi_dbm = 0;


// Helper: print a uint64_t via Arduino Print without ambiguity
static void print_uint64(Print &p, uint64_t value) {
    // Max uint64_t is 18446744073709551615 -> 20 digits + null
    char buf[21];
    buf[20] = '\0';
    char *ptr = &buf[20];

    if (value == 0) {
        *--ptr = '0';
    } else {
        while (value > 0) {
            uint8_t digit = value % 10;
            value /= 10;
            *--ptr = '0' + digit;
        }
    }

    p.print(ptr);
}


// RADIO interrupt handler
extern "C" void RADIO_IRQHandler(void) {
    if (NRF_RADIO->EVENTS_READY) {
        NRF_RADIO->EVENTS_READY = 0;
        NRF_RADIO->TASKS_START  = 1;
    }

    if (NRF_RADIO->EVENTS_END) {
        NRF_RADIO->EVENTS_END = 0;

        if (NRF_RADIO->CRCSTATUS == 1) {
            int8_t rssi = (int8_t)NRF_RADIO->RSSISAMPLE;

            packet_tag_id   = rx_packet.tag_id;
            packet_rssi_dbm = rssi;
            packet_ready    = true;
        }

        // Keep listening
        NRF_RADIO->TASKS_START = 1;
    }
}

static void radio_init_rx() {
    // HFCLK on
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) { }

    // Address/sync – must match TX
    NRF_RADIO->PREFIX0     = 0x12UL;
    NRF_RADIO->BASE0       = 0x89ABCDEFUL;
    NRF_RADIO->TXADDRESS   = 0;
    NRF_RADIO->RXADDRESSES = (1 << 0);

    // Channel 80 => 2480 MHz
    NRF_RADIO->FREQUENCY = 80;

    // 1 Mbps GFSK
    NRF_RADIO->MODE = (RADIO_MODE_MODE_Nrf_1Mbit << RADIO_MODE_MODE_Pos);

    // Packet config – 4-byte payload, no length field
    NRF_RADIO->PCNF0 =
        (0 << RADIO_PCNF0_S0LEN_Pos) |
        (0 << RADIO_PCNF0_S1LEN_Pos) |
        (0 << RADIO_PCNF0_LFLEN_Pos);

    NRF_RADIO->PCNF1 =
        (RADIO_PCNF1_ENDIAN_Big << RADIO_PCNF1_ENDIAN_Pos) |
        (4 << RADIO_PCNF1_BALEN_Pos) |
        (PACKET_LEN << RADIO_PCNF1_STATLEN_Pos) |
        (PACKET_LEN << RADIO_PCNF1_MAXLEN_Pos);

    // 16-bit CRC
    NRF_RADIO->CRCCNF  = (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos);
    NRF_RADIO->CRCINIT = 0xFFFFUL;
    NRF_RADIO->CRCPOLY = 0x11021UL;

    NRF_RADIO->PACKETPTR = (uint32_t)&rx_packet;

    // Auto-start RSSI when address is detected
    NRF_RADIO->SHORTS = RADIO_SHORTS_ADDRESS_RSSISTART_Msk;

    // Enable interrupts
    NRF_RADIO->INTENSET = RADIO_INTENSET_READY_Msk | RADIO_INTENSET_END_Msk;
    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_SetPriority(RADIO_IRQn, 2);
    NVIC_EnableIRQ(RADIO_IRQn);

    // Start RX
    NRF_RADIO->TASKS_RXEN = 1;
}

// =======================================
// SdFat + RingBuf logger
// =======================================

// Use SdFat and File32 (FAT32) – works on XIAO nRF52840
SdFat  sd;
File32 log_file;

// RingBuf of fixed byte size (must be multiple of 512)
RingBuf<File32, RING_BUF_SIZE> rb;

static void fatal_error(const __FlashStringHelper *msg) {
    Serial.println(msg);
    // Hard stop
    while (true) {
        delay(1000);
    }
}

static void sd_init_and_open_file() {
    // Init SD over SPI (25 MHz is usually safe)
    if (!sd.begin(SD_CS_PIN, SD_SCK_MHZ(25))) {
        sd.initErrorPrint(&Serial);
        fatal_error(F("sd.begin failed"));
    }

    // Open or create file – truncate existing
    if (!log_file.open(LOG_FILENAME, O_RDWR | O_CREAT | O_TRUNC)) {
        fatal_error(F("open log file failed"));
    }

    // Pre-allocate to reduce latency
    if (LOG_FILE_SIZE > 0 && !log_file.preAllocate(LOG_FILE_SIZE)) {
        Serial.println(F("preAllocate failed, continuing without it"));
        // Not fatal, just means more occasional stalls.
    }

    // Initialize RingBuf for this file
    rb.begin(&log_file);

    // CSV header – note unix_ms for clarity
    rb.print(F("unix_ms,tag_id,rssi\n"));
}

// Write out full 512-byte blocks when possible
static void sd_service_ringbuf() {
    // If card is busy finishing a previous write, do nothing
    if (log_file.isBusy()) {
        return;
    }

    // Only write whole sectors
    while (rb.bytesUsed() >= 512 && !log_file.isBusy()) {
        if (512 != rb.writeOut(512)) {
            fatal_error(F("rb.writeOut failed"));
        }
    }
}

// =======================================
// MAIN
// =======================================

void setup() {
    Serial.begin(115200);
    unsigned long start = millis();
    while (!Serial && (millis() - start < 2000)) {
        delay(10);
    }

    Log.begin(LOG_LEVEL_NOTICE, &Serial);
    Log.noticeln(F("XIAO nRF52840 raw radio → SdFat RingBuf logger"));

    // RTC baseline
    RtcTime::init();

    // SD + file + ring buffer
    sd_init_and_open_file();

    // Radio RX
    radio_init_rx();

    Serial.println(F("Logging raw radio samples to " LOG_FILENAME));
    Serial.println(F("Format: unix_ms,tag_id,rssi"));

    // Make sure we start with an empty mailbox
    noInterrupts();
    packet_ready = false;
    interrupts();
}

void loop() {
    // 1) Drain the ISR mailbox
    while (true) {
        uint8_t tag;
        int8_t  rssi;
        bool    have_packet = false;

        noInterrupts();
        if (packet_ready) {
            tag          = packet_tag_id;
            rssi         = packet_rssi_dbm;
            packet_ready = false;
            have_packet  = true;
        }
        interrupts();

        if (!have_packet) {
            break;
        }

        // Timestamp as close to arrival as we can
        uint32_t now_ms   = millis();
        uint64_t unix_ms  = RtcTime::ms_to_unix_ms(now_ms);

        // Append CSV line to ring buffer: unix_ms,tag_id,rssi
        print_uint64(rb, unix_ms);
        rb.write(',');
        rb.print(tag);
        rb.write(',');
        rb.println((int)rssi);

        if (rb.getWriteError()) {
            fatal_error(F("RingBuf writeError (buffer too small)"));
        }

        // *** DO NOT print each sample to Serial here ***
        // If you *really* want some debug, sample it very sparsely, e.g.:
        /*
        static uint32_t debug_counter = 0;
        if ((debug_counter++ & 0x3FF) == 0) { // 1 out of 1024 samples
            Serial.print(F("RAW,"));
            Serial.print(unix_ms);
            Serial.print(',');
            Serial.print(tag);
            Serial.print(',');
            Serial.println((int)rssi);
        }
        */
    }

    // 2) Service SD – push full sectors out
    sd_service_ringbuf();

    // 3) Periodically sync file size / directory entry
    static uint32_t last_sync_ms = 0;
    uint32_t now = millis();
    if ((now - last_sync_ms) >= SYNC_INTERVAL_MS) {
        last_sync_ms = now;
        // Flush any partial block and update directory
        rb.sync();
        log_file.sync();
    }

    // Optional: hit any key in Serial to stop and close nicely
    if (Serial.available()) {
        Serial.println(F("Stopping log, syncing..."));
        rb.sync();
        log_file.sync();
        log_file.close();
        sd.end();
        fatal_error(F("Logging stopped by user"));
    }

    // Short yield – not strictly necessary, but polite
    delay(1);
}
