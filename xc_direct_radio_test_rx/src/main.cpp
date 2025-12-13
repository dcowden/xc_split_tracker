// main.cpp — nRF52840 RX: ring-buffered radio capture -> UART, 1Hz USB stats (counts + min/max/EMA)
// Adds: once/second USB line showing the last UART packet exactly as sent: "last sent: <data>"
//
// Notes:
//  - UART forwards every packet as CSV: t_ms,tag,seq,rssi_dbm\n
//  - USB prints a summary once per second (NOT full-rate per packet)

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <nrf.h>

#define PACKET_LEN 4

// =====================
// CONFIG
// =====================
static constexpr float    EMA_ALPHA  = 0.10f;
static constexpr uint16_t MAX_TAG_ID = 256;

static constexpr uint32_t UART_BAUD = 460800;

// Ring buffer (ISR -> loop). Must be power-of-two.
static constexpr uint16_t RB_SIZE = 512;
static constexpr uint16_t RB_MASK = RB_SIZE - 1;

typedef struct __attribute__((packed)) {
  uint8_t  tag_id;
  uint8_t  seq;
  uint16_t reserved;
} xc_packet_t;

// RADIO DMA buffer
static volatile xc_packet_t rx_packet;

// =====================
// Ring buffer
// =====================
struct RxEntry {
  uint32_t t_ms;
  uint8_t  tag_id;
  uint8_t  seq;
  int8_t   rssi_dbm;
};

static RxEntry           g_rb[RB_SIZE];
static volatile uint16_t g_rb_w     = 0;
static volatile uint16_t g_rb_r     = 0;
static volatile uint32_t g_rb_drops = 0;

// =====================
// Per-tag stats
// =====================
static volatile uint32_t g_tag_counts[MAX_TAG_ID];
static volatile int8_t   g_tag_min   [MAX_TAG_ID];
static volatile int8_t   g_tag_max   [MAX_TAG_ID];
static volatile float    g_tag_ema   [MAX_TAG_ID];
static volatile bool     g_tag_seen  [MAX_TAG_ID];
static volatile uint32_t g_total_ok  = 0;

// =====================
// Last UART line sent (for USB display)
// =====================
static char              g_last_uart_line[48] = {0};  // enough for "4294967295,255,255,-127\n"
static volatile uint16_t g_last_uart_len = 0;

// =====================
// RADIO IRQ
// =====================
extern "C" void RADIO_IRQHandler(void) {
  if (NRF_RADIO->EVENTS_READY) {
    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->TASKS_START  = 1;
  }

  if (NRF_RADIO->EVENTS_END) {
    NRF_RADIO->EVENTS_END = 0;

    if (NRF_RADIO->CRCSTATUS == 1) {
      const uint8_t tag = rx_packet.tag_id;
      const uint8_t seq = rx_packet.seq;

      // RSSISAMPLE is magnitude of (-RSSI). Convert to signed dBm by negating.
      const int8_t rssi_dbm = -(int8_t)NRF_RADIO->RSSISAMPLE;

      const uint32_t tms = millis();

      // Stats
      g_total_ok++;
      g_tag_counts[tag]++;

      if (!g_tag_seen[tag]) {
        g_tag_seen[tag] = true;
        g_tag_min[tag]  = rssi_dbm;
        g_tag_max[tag]  = rssi_dbm;
        g_tag_ema[tag]  = (float)rssi_dbm;
      } else {
        if (rssi_dbm < g_tag_min[tag]) g_tag_min[tag] = rssi_dbm;
        if (rssi_dbm > g_tag_max[tag]) g_tag_max[tag] = rssi_dbm;
        g_tag_ema[tag] = EMA_ALPHA * (float)rssi_dbm + (1.0f - EMA_ALPHA) * g_tag_ema[tag];
      }

      // Push into ring buffer
      const uint16_t w    = g_rb_w;
      const uint16_t next = (uint16_t)((w + 1) & RB_MASK);

      if (next == g_rb_r) {
        g_rb_drops++;
      } else {
        g_rb[w].t_ms     = tms;
        g_rb[w].tag_id   = tag;
        g_rb[w].seq      = seq;
        g_rb[w].rssi_dbm = rssi_dbm;
        g_rb_w = next;
      }
    }

    // Keep listening
    NRF_RADIO->TASKS_START = 1;
  }
}

static void radio_init_rx() {
  // HFCLK on
  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART    = 1;
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {}

  // Addressing (must match TX)
  NRF_RADIO->PREFIX0     = 0x12UL;
  NRF_RADIO->BASE0       = 0x89ABCDEFUL;
  NRF_RADIO->TXADDRESS   = 0;
  NRF_RADIO->RXADDRESSES = (1 << 0);

  // Channel 80 => 2480 MHz
  NRF_RADIO->FREQUENCY = 80;

  // 1 Mbps GFSK
  NRF_RADIO->MODE = (RADIO_MODE_MODE_Nrf_1Mbit << RADIO_MODE_MODE_Pos);

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

  // Start RSSI when ADDRESS detected
  NRF_RADIO->SHORTS = RADIO_SHORTS_ADDRESS_RSSISTART_Msk;

  // IRQs
  NRF_RADIO->INTENSET = RADIO_INTENSET_READY_Msk | RADIO_INTENSET_END_Msk;
  NVIC_ClearPendingIRQ(RADIO_IRQn);
  NVIC_SetPriority(RADIO_IRQn, 2);
  NVIC_EnableIRQ(RADIO_IRQn);

  // Start RX
  NRF_RADIO->TASKS_RXEN = 1;
}

// Pop one entry (atomic w.r.t ISR)
static inline bool rb_pop(RxEntry &out) {
  bool ok = false;
  noInterrupts();
  const uint16_t r = g_rb_r;
  const uint16_t w = g_rb_w;
  if (r != w) {
    out = g_rb[r];
    g_rb_r = (uint16_t)((r + 1) & RB_MASK);
    ok = true;
  }
  interrupts();
  return ok;
}

void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 1500)) delay(10);

  // Init stats arrays
  for (uint16_t i = 0; i < MAX_TAG_ID; i++) {
    g_tag_counts[i] = 0;
    g_tag_min[i]    = 127;
    g_tag_max[i]    = -128;
    g_tag_ema[i]    = 0.0f;
    g_tag_seen[i]   = false;
  }

  g_last_uart_line[0] = '\0';
  g_last_uart_len = 0;

  Serial.println("XC RX: USB summary once/sec. UART forwards all packets.");
  Serial.println("UART CSV: t_ms,tag,seq,rssi_dbm");
  Serial.print("EMA_ALPHA=");
  Serial.println(EMA_ALPHA, 3);

  Serial1.begin(UART_BAUD);

  radio_init_rx();
}

void loop() {
  // Drain ring buffer to UART (always)
  RxEntry e;
  char line[48];

  while (rb_pop(e)) {
    // CSV: t_ms,tag,seq,rssi_dbm\n
    const int n = snprintf(line, sizeof(line), "%lu,%u,%u,%d\n",
                           (unsigned long)e.t_ms,
                           (unsigned)e.tag_id,
                           (unsigned)e.seq,
                           (int)e.rssi_dbm);
    if (n > 0) {
      Serial1.write((const uint8_t*)line, (size_t)n);

      // Save "exactly as sent" for USB status (copy with interrupts off to avoid torn read)
      const uint16_t copy_len = (uint16_t)((n < (int)(sizeof(g_last_uart_line) - 1)) ? n : (int)(sizeof(g_last_uart_line) - 1));
      noInterrupts();
      memcpy((void*)g_last_uart_line, line, copy_len);
      g_last_uart_line[copy_len] = '\0';
      g_last_uart_len = copy_len;
      interrupts();
    }
  }

  // Once per second: print per-tag counts + min/max/ema and last UART line to USB
  static uint32_t last_print_ms = 0;
  const uint32_t now = millis();
  if ((now - last_print_ms) >= 1000) {
    last_print_ms = now;

    uint32_t drops, total;
    uint32_t counts[MAX_TAG_ID];
    int8_t   minv  [MAX_TAG_ID];
    int8_t   maxv  [MAX_TAG_ID];
    float    ema   [MAX_TAG_ID];
    bool     seen  [MAX_TAG_ID];

    char last_line_copy[48];
    uint16_t last_len_copy;

    noInterrupts();
    drops = g_rb_drops;
    total = g_total_ok;
    for (uint16_t i = 0; i < MAX_TAG_ID; i++) {
      counts[i] = g_tag_counts[i];
      minv[i]   = g_tag_min[i];
      maxv[i]   = g_tag_max[i];
      ema[i]    = g_tag_ema[i];
      seen[i]   = g_tag_seen[i];
    }
    last_len_copy = g_last_uart_len;
    memcpy(last_line_copy, g_last_uart_line, sizeof(last_line_copy));
    interrupts();

    // Ensure NUL even if something weird happens
    last_line_copy[sizeof(last_line_copy) - 1] = '\0';

    Serial.println();
    Serial.print("t=");
    Serial.print(now);
    Serial.print("ms  total_ok=");
    Serial.print(total);
    Serial.print("  rb_drops=");
    Serial.println(drops);

    Serial.print("last sent: ");
    if (last_len_copy == 0) {
      Serial.println("(none yet)");
    } else {
      // last_line_copy already includes '\n' because that's how we sent it
      Serial.print(last_line_copy);
      if (last_line_copy[last_len_copy - 1] != '\n') Serial.println();
    }

    Serial.println("Tag  Events  Min  Max  EMA");
    Serial.println("--------------------------");

    for (uint16_t i = 0; i < MAX_TAG_ID; i++) {
      if (seen[i] && counts[i]) {
        Serial.print(i);
        Serial.print("    ");
        Serial.print(counts[i]);
        Serial.print("    ");
        Serial.print((int)minv[i]);
        Serial.print("  ");
        Serial.print((int)maxv[i]);
        Serial.print("  ");
        Serial.println(ema[i], 1);
      }
    }
  }

  delay(1);  // let TinyUSB breathe
}
