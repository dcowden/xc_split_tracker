// TX TAG: Seeed XIAO nRF52840
// Raw RADIO TX, 1M GFSK
//
// PROGRAMMING_MODE = 1  -> debug-friendly (Serial + cooperative loop)
// PROGRAMMING_MODE = 0  -> production tight timing (no Serial)
//
// Battery:
//  - Sample VBAT on a Ticker at BATTERY_SAMPLE_INTERVAL_MS
//  - Map to uint8 health: 10 at 4.2V, 5 at "replace"
//  - Send health in every transmitted packet (battery[15:8])
//
// Motion sleep (SW-18020P / spring vibration switch):
//  - Wire switch between D2 and GND
//  - Add ~1 MΩ pull-up from D2 to 3V3
//  - Any vibration updates "last motion"
//  - After 15 minutes with no vibration, enter SYSTEMOFF
//  - Wake on D2 sensing LOW (switch closure to GND) -> reboot and resume TX

#include <Arduino.h>
#include <nrf.h>
#include <Ticker.h>
#include <nrf_gpio.h>

// ---------------------------
// Mode switch
// ---------------------------
#ifndef PROGRAMMING_MODE
#define PROGRAMMING_MODE 1
#endif

#if PROGRAMMING_MODE
  #include "Adafruit_TinyUSB.h"
#endif

// ---------------------------
// XIAO nRF52840 battery sense pins
// ---------------------------
static constexpr uint8_t VBAT_EN  = 14;  // P0.14 / D14 (active LOW)
static constexpr uint8_t VBAT_ADC = 32;  // P0.31 (ADC input)
static constexpr uint8_t LED_PIN = LED_BUILTIN;
// ---------------------------
// Motion sleep pins/config
// ---------------------------
static constexpr uint8_t  MOTION_PIN     = D2;                 // XIAO D2
//static constexpr uint32_t MOTION_IDLE_MS = 15UL * 60UL * 1000UL; // 15 minutes
static constexpr uint32_t MOTION_IDLE_MS = 15UL * 1000UL; // 15 seconds

static volatile bool     g_motion_flag    = false;
static volatile uint32_t g_last_motion_ms = 0;

// ---------------------------
// Config
// ---------------------------
#define TAG_ID                      4
#define TX_INTERVAL_MS              5
#define BATTERY_SAMPLE_INTERVAL_MS  2000

#if PROGRAMMING_MODE
  #define DEBUG_INTERVAL_MS         1000
  static constexpr uint32_t UART_BAUD = 115200;
#endif

static constexpr uint16_t VBAT_MAX_MV     = 4200;
static constexpr uint16_t VBAT_REPLACE_MV = 3600;   // adjust as desired
static constexpr float    VBAT_SCALE      = 3.0f;   // calibrate if needed

// ---------------------------
// Packet format (4 bytes total)
// ---------------------------
typedef struct __attribute__((packed)) {
    uint8_t  tag_id;
    uint8_t  seq;
    uint16_t battery;   // [15:8] = health (5..10), [7:0] reserved
} xc_packet_t;

static volatile xc_packet_t tx_packet;
static volatile bool radio_busy = false;

// Cached battery health (updated by ticker)
static volatile uint8_t g_batt_health = 0;

// ---------------------------
// Battery sense
// ---------------------------
static inline void battery_sense_init_safe()
{
  pinMode(VBAT_EN, OUTPUT);
  digitalWrite(VBAT_EN, LOW);   // IMPORTANT: never drive HIGH
  analogReadResolution(12);
}

static inline uint16_t read_battery_mv_safe()
{
  uint16_t raw = analogRead(VBAT_ADC);       // 0..4095
  float v_adc = (raw / 4095.0f) * 3.3f;      // volts at P0.31
  float v_bat = v_adc * VBAT_SCALE;          // undo divider
  return (uint16_t)(v_bat * 1000.0f + 0.5f); // mV
}

// Map mV -> health: 10 at 4.2V, 5 at replace, clamp outside.
static inline uint8_t battery_health_from_mv(uint16_t mv)
{
  if (mv >= VBAT_MAX_MV)     return 10;
  if (mv <= VBAT_REPLACE_MV) return 5;

  // Linear mapping between REPLACE_MV..MAX_MV -> 5..10
  uint32_t num = (uint32_t)(mv - VBAT_REPLACE_MV) * 5u;
  uint32_t den = (uint32_t)(VBAT_MAX_MV - VBAT_REPLACE_MV);
  uint8_t h = 5 + (uint8_t)(num / den);

  if (h < 5)  h = 5;
  if (h > 10) h = 10;
  return h;
}

// Battery sampler ticker callback
static void on_battery_sample_tick()
{
  uint16_t mv = read_battery_mv_safe();
  g_batt_health = battery_health_from_mv(mv);

#if PROGRAMMING_MODE
  Serial.print("VBATT(mV)=");
  Serial.print(mv);
  Serial.print(" health=");
  Serial.println(g_batt_health);
#endif
}

static Ticker g_battery_ticker(on_battery_sample_tick,
                               BATTERY_SAMPLE_INTERVAL_MS,
                               0,
                               MILLIS);

// ---------------------------
// Motion handling
// ---------------------------
static void on_motion_isr()
{
  g_motion_flag = true;
}

static void motion_init()
{
  // External ~1M pull-up to 3V3, switch to GND
  pinMode(MOTION_PIN, INPUT_PULLUP);
  pinMode(MOTION_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(MOTION_PIN), on_motion_isr, CHANGE);

  g_last_motion_ms = millis();
}

// ---------------------------
// Debug (optional)
// ---------------------------
#if PROGRAMMING_MODE
static void on_debug_tick()
{
  Serial.print("seq=");
  Serial.print(tx_packet.seq);
  Serial.print(" batt_health=");
  Serial.print((uint32_t)g_batt_health);
  Serial.print(" idle_ms=");
  Serial.println((uint32_t)(millis() - g_last_motion_ms));
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}
static Ticker g_debug_ticker(on_debug_tick, DEBUG_INTERVAL_MS, 0, MILLIS);

static void wait_for_serial()
{
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 3000) {
    delay(10);
  }
  Serial.println("boot");
}
#endif

// ---------------------------
// TIMER0 ISR (TX scheduler)
// ---------------------------
extern "C" void TIMER0_IRQHandler(void)
{
    if (NRF_TIMER0->EVENTS_COMPARE[0]) {
        NRF_TIMER0->EVENTS_COMPARE[0] = 0;

        if (!radio_busy) {
            radio_busy = true;

            tx_packet.seq++;

            // Inject battery health every packet
            tx_packet.battery = ((uint16_t)g_batt_health << 8);

            NRF_RADIO->PACKETPTR = (uint32_t)&tx_packet;
            NRF_RADIO->TASKS_TXEN = 1;
        }
    }
}

static void init_timer0_for_interval_ms(uint16_t interval_ms)
{
    NRF_TIMER0->TASKS_STOP = 1;
    NRF_TIMER0->MODE       = TIMER_MODE_MODE_Timer;
    NRF_TIMER0->BITMODE    = TIMER_BITMODE_BITMODE_16Bit;
    NRF_TIMER0->PRESCALER  = 4;                         // 1 µs ticks
    NRF_TIMER0->CC[0]      = interval_ms * 1000;        // µs
    NRF_TIMER0->SHORTS     = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
    NRF_TIMER0->INTENSET   = TIMER_INTENSET_COMPARE0_Msk;

    NVIC_ClearPendingIRQ(TIMER0_IRQn);
    NVIC_SetPriority(TIMER0_IRQn, 3);
    NVIC_EnableIRQ(TIMER0_IRQn);

    NRF_TIMER0->TASKS_START = 1;
}

// ---------------------------
// RADIO ISR
// ---------------------------
extern "C" void RADIO_IRQHandler(void)
{
    if (NRF_RADIO->EVENTS_READY) {
        NRF_RADIO->EVENTS_READY = 0;
        NRF_RADIO->TASKS_START  = 1;
    }

    if (NRF_RADIO->EVENTS_END) {
        NRF_RADIO->EVENTS_END = 0;
        NRF_RADIO->TASKS_DISABLE = 1;
        radio_busy = false;
    }
}

static void radio_init_tx()
{
    // HF clock on
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;
    while (!NRF_CLOCK->EVENTS_HFCLKSTARTED) {}

    // Address / sync word
    NRF_RADIO->PREFIX0     = 0x12;
    NRF_RADIO->BASE0       = 0x89ABCDEF;
    NRF_RADIO->TXADDRESS   = 0;
    NRF_RADIO->RXADDRESSES = (1 << 0);

    // Channel 80 → 2480 MHz
    NRF_RADIO->FREQUENCY = 80;

    // 1 Mbps GFSK
    NRF_RADIO->MODE = RADIO_MODE_MODE_Nrf_1Mbit;

    // Packet config (fixed 4 bytes)
    NRF_RADIO->PCNF0 = 0;
    NRF_RADIO->PCNF1 =
        (RADIO_PCNF1_ENDIAN_Big << RADIO_PCNF1_ENDIAN_Pos) |
        (4 << RADIO_PCNF1_BALEN_Pos) |
        (4 << RADIO_PCNF1_STATLEN_Pos) |
        (4 << RADIO_PCNF1_MAXLEN_Pos);

    // 16-bit CRC
    NRF_RADIO->CRCCNF  = RADIO_CRCCNF_LEN_Two;
    NRF_RADIO->CRCINIT = 0xFFFF;
    NRF_RADIO->CRCPOLY = 0x11021;

    // TX power
    NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_Pos8dBm;

    NRF_RADIO->PACKETPTR = (uint32_t)&tx_packet;

    // IRQs
    NRF_RADIO->INTENSET = RADIO_INTENSET_READY_Msk | RADIO_INTENSET_END_Msk;
    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_SetPriority(RADIO_IRQn, 2);
    NVIC_EnableIRQ(RADIO_IRQn);
}

// ---------------------------
// Deep sleep (SYSTEMOFF)
// ---------------------------
static void go_to_system_off()
{
#if PROGRAMMING_MODE
  Serial.println("Entering SYSTEMOFF (no motion)...");
  Serial.flush();
  digitalWrite(LED_PIN,0);
#endif

  // Stop periodic TX schedule
  NRF_TIMER0->TASKS_STOP = 1;
  NVIC_DisableIRQ(TIMER0_IRQn);

  // Stop radio
  NRF_RADIO->TASKS_DISABLE = 1;
  NVIC_DisableIRQ(RADIO_IRQn);

  // Stop tickers
  g_battery_ticker.stop();
#if PROGRAMMING_MODE
  g_debug_ticker.stop();
#endif

  // Configure MOTION_PIN as a wake source: sense LOW (switch closes to GND)
  // Arduino pin -> nRF pin number using core mapping table
  uint32_t nrf_pin = g_ADigitalPinMap[MOTION_PIN];

  nrf_gpio_cfg_sense_input(
      nrf_pin,
      NRF_GPIO_PIN_NOPULL,     // external pull-up handles it
      //NRF_GPIO_PIN_PULLUP,
      NRF_GPIO_PIN_SENSE_LOW   // wake when pin goes LOW
  );

  // Enter SYSTEMOFF (wake triggers reset-like boot)
  NRF_POWER->SYSTEMOFF = 1;

  __DSB();
  __WFE();
  while (1) { __WFE(); }
}

// ---------------------------
// Setup / Loop
// ---------------------------
void setup()
{
#if PROGRAMMING_MODE
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(UART_BAUD);
    wait_for_serial();
#endif

    battery_sense_init_safe();
    motion_init();

    tx_packet.tag_id  = TAG_ID;
    tx_packet.seq     = 0;
    tx_packet.battery = 0;

    // Ensure health isn't 0 at boot
    on_battery_sample_tick();

    // Start battery ticker in BOTH modes
    g_battery_ticker.start();

    radio_init_tx();
    init_timer0_for_interval_ms(TX_INTERVAL_MS);

#if PROGRAMMING_MODE
    g_debug_ticker.start();
#endif
}

void loop()
{
    // Tickers run in BOTH modes
    g_battery_ticker.update();

    // Handle motion events (keep millis() out of ISR)
    if (g_motion_flag) {
      g_motion_flag = false;
      g_last_motion_ms = millis();
    }

    // If no vibration for 15 minutes -> SYSTEMOFF
    if ((uint32_t)(millis() - g_last_motion_ms) > MOTION_IDLE_MS) {
      go_to_system_off();
    }

#if PROGRAMMING_MODE
    g_debug_ticker.update();
    delay(1);  // keep USB happy
#else
    // Tight timing: no delay, mostly sleep.
    __WFE();
    __SEV();
    __WFE();
#endif
}
