// TX TAG: Seeed XIAO nRF52840
// Raw RADIO TX, 1M GFSK

#include <Arduino.h>
#include <nrf.h>
#include <nrf_gpio.h>

#ifndef PROGRAMMING_MODE
//#define PROGRAMMING_MODE 1
#endif

#if PROGRAMMING_MODE
  #include "Adafruit_TinyUSB.h"
#endif

#include "nicenano_pins.h"
#include "battery.h"

// ---------------------------
// Pins / Config
// ---------------------------
static constexpr uint8_t  LED_PIN       = P0_15;
static constexpr uint8_t  MOTION_PIN    = P1_06;
static constexpr uint8_t  VDD_RAIL_PIN  = P1_13;

#define TAG_ID          1
#define TX_INTERVAL_MS  5


#if PROGRAMMING_MODE
  static constexpr uint32_t UART_BAUD = 115200;
  #define DEBUG_INTERVAL_MS 1000
  static constexpr uint32_t MOTION_IDLE_MS = 15UL * 1000UL; // 15 seconds (test)
  #else
  static constexpr uint32_t MOTION_IDLE_MS = 15UL * 60UL * 1000UL; // 15 minutes
#endif

// ---------------------------
// Packet format (4 bytes total)
// ---------------------------
typedef struct __attribute__((packed)) {
    uint8_t  tag_id;
    uint8_t  seq;
    uint16_t battery;   // [15:8] = health, [7:0] reserved
} xc_packet_t;

static volatile xc_packet_t tx_packet;
static volatile bool radio_busy = false;

// ---------------------------
// Motion handling
// ---------------------------
static volatile bool     g_motion_flag    = false;
static volatile uint32_t g_last_motion_ms = 0;

static void on_motion_isr()
{
  g_motion_flag = true;
}

static void motion_init()
{
  // External ~1M pull-up to 3V3, switch to GND
  pinMode(MOTION_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MOTION_PIN), on_motion_isr, CHANGE);

  g_last_motion_ms = millis();
}

// ---------------------------
// Debug (optional)
// ---------------------------
#if PROGRAMMING_MODE
#include <Ticker.h>

static void on_debug_tick()
{
  Serial.print("seq=");
  Serial.print(tx_packet.seq);
  Serial.print(" batt_health=");
  Serial.print((uint32_t)g_batt_health);
  Serial.print(" idle_ms=");
  Serial.println((uint32_t)(millis() - g_last_motion_ms));
  Serial.print(" wakepin=");
  Serial.println(digitalRead(MOTION_PIN));
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
      tx_packet.battery = battery_pack_health_u16();

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
#endif

  // Stop periodic TX schedule
  NRF_TIMER0->TASKS_STOP = 1;
  NVIC_DisableIRQ(TIMER0_IRQn);

  // Stop radio
  NRF_RADIO->TASKS_DISABLE = 1;
  NVIC_DisableIRQ(RADIO_IRQn);

  // Stop battery + LED
  battery_stop();

#if PROGRAMMING_MODE
  g_debug_ticker.stop();
#endif

  // Configure MOTION_PIN as a wake source: sense LOW (switch closes to GND)
  uint32_t nrf_pin = g_ADigitalPinMap[MOTION_PIN];

  nrf_gpio_cfg_sense_input(
      nrf_pin,
      NRF_GPIO_PIN_PULLUP,
      NRF_GPIO_PIN_SENSE_LOW
  );

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
  // Power rail enable (your original)
  pinMode(VDD_RAIL_PIN, OUTPUT);
  digitalWrite(VDD_RAIL_PIN, HIGH);

#if PROGRAMMING_MODE
  Serial.begin(UART_BAUD);
  wait_for_serial();
#endif

  // Battery + LED status
  battery_init(LED_PIN);
  battery_start();

  motion_init();

  tx_packet.tag_id  = TAG_ID;
  tx_packet.seq     = 0;
  tx_packet.battery = 0;

  radio_init_tx();
  init_timer0_for_interval_ms(TX_INTERVAL_MS);

#if PROGRAMMING_MODE
  g_debug_ticker.start();
#endif
}

void loop()
{
  // Battery sampler ticker + LED status sequencer
  battery_update();

  // Handle motion events (keep millis() out of ISR)
  if (g_motion_flag) {
     
    g_motion_flag = false;
    #if PROGRAMMING_MODE   
   Serial.println("Got Wake Event..");
   Serial.flush();
    #endif
    g_last_motion_ms = millis();
  }

  // If no vibration -> SYSTEMOFF
  if ((uint32_t)(millis() - g_last_motion_ms) > MOTION_IDLE_MS) {
    go_to_system_off();
  }

#if PROGRAMMING_MODE
  g_debug_ticker.update();
  delay(1);  // keep USB happy
#else
  __WFE();
  __SEV();
  __WFE();
#endif
}
