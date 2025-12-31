#pragma once

#include <Arduino.h>
#include <nrf.h>
#include <Ticker.h>
#include "nicenano_pins.h"
// -----------------------------------------------------------------------------
// Config defaults (override by defining before including this header)
// -----------------------------------------------------------------------------
#ifndef BATTERY_SAMPLE_INTERVAL_MS
#define BATTERY_SAMPLE_INTERVAL_MS 2000
#endif



// How often to run the LED sequencer tick.
// 10ms is plenty and cheap.
#ifndef LED_TICK_MS
#define LED_TICK_MS 10
#define TOTAL_CYCLES_BATTERY_OK 500
#define TOTAL_CYCLES_BATTERY_LOW 100
#endif


// -----------------------------------------------------------------------------
// Public state
// -----------------------------------------------------------------------------
static volatile uint8_t g_batt_health = 0;

// -----------------------------------------------------------------------------
// Internal state
// -----------------------------------------------------------------------------
static uint8_t  g_batt_led_pin = P0_15;

// LED sequencer state
static volatile uint8_t  g_led_pulses_left     = 0;
static volatile bool     g_led_is_on           = false;

// -----------------------------------------------------------------------------
// LED helpers (polarity-aware)
// -----------------------------------------------------------------------------
static inline void led_write_on()
{
  digitalWrite(g_batt_led_pin, HIGH);
}

static inline void led_write_off()
{
  digitalWrite(g_batt_led_pin, LOW);
}

// -----------------------------------------------------------------------------
// Battery read (nRF52 SAADC VDDHDIV5 method)
// -----------------------------------------------------------------------------
static inline float readNanoBatteryVoltage()
{
  volatile uint32_t raw_value = 0;

  NRF_SAADC->ENABLE     = 1;
  NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_12bit;

  NRF_SAADC->CH[0].CONFIG =
    (SAADC_CH_CONFIG_GAIN_Gain1_4      << SAADC_CH_CONFIG_GAIN_Pos) |
    (SAADC_CH_CONFIG_MODE_SE           << SAADC_CH_CONFIG_MODE_Pos) |
    (SAADC_CH_CONFIG_REFSEL_Internal   << SAADC_CH_CONFIG_REFSEL_Pos);

  NRF_SAADC->CH[0].PSELP = SAADC_CH_PSELP_PSELP_VDDHDIV5;
  NRF_SAADC->CH[0].PSELN = SAADC_CH_PSELN_PSELN_NC;

  NRF_SAADC->RESULT.PTR    = (uint32_t)&raw_value;
  NRF_SAADC->RESULT.MAXCNT = 1;

  NRF_SAADC->TASKS_START = 1;
  while (!NRF_SAADC->EVENTS_STARTED) {}
  NRF_SAADC->EVENTS_STARTED = 0;

  NRF_SAADC->TASKS_SAMPLE = 1;
  while (!NRF_SAADC->EVENTS_END) {}
  NRF_SAADC->EVENTS_END = 0;

  NRF_SAADC->TASKS_STOP = 1;
  while (!NRF_SAADC->EVENTS_STOPPED) {}
  NRF_SAADC->EVENTS_STOPPED = 0;

  NRF_SAADC->ENABLE = 0;

  double raw_double = (double)raw_value;
  double step1      = raw_double * 2.4;
  double step2      = step1 / 4095.0;
  double vddh       = 5.0 * step2;

  return (float)vddh;
}

static inline uint8_t battery_health_from_v(float v)
{
  if (v >= 4.1f) return 10;
  if (v >= 4.0f) return 9;
  if (v >= 3.9f) return 8;
  if (v >= 3.8f) return 7;
  if (v >= 3.7f) return 6;
  if (v >= 3.6f) return 5;
  if (v >= 3.5f) return 4;
  if (v >= 3.4f) return 3;
  if (v >= 3.3f) return 2;
  if (v >= 3.2f) return 1;
  return 0;
}

// -----------------------------------------------------------------------------
// LED sequencer step (runs from ticker; does NOT rely on loop timing)
// -----------------------------------------------------------------------------
static inline void led_sequencer_step()
{
  //flash once-- if battery is healthy, twice if its time to change 
  if ( g_led_is_on ){
    led_write_off();
    g_led_is_on = false;
    if ( g_batt_health > 3 ){
        g_led_pulses_left = TOTAL_CYCLES_BATTERY_OK;
    }
    else{
        g_led_pulses_left = TOTAL_CYCLES_BATTERY_LOW;
    }
    
  }
  else{
    if ( g_led_pulses_left > 0){
        g_led_pulses_left--;
    }
    else{
        g_led_is_on = true;
        led_write_on();
    }
  }

}

// Wrapper required by Ticker (no-arg function)
static void on_led_tick()
{
  led_sequencer_step();
}

// -----------------------------------------------------------------------------
// Battery sampler ticker
// -----------------------------------------------------------------------------
static inline void on_battery_sample_tick()
{
  float v = readNanoBatteryVoltage();
  g_batt_health = battery_health_from_v(v);

#if PROGRAMMING_MODE
  Serial.print("VBATT(V)=");
  Serial.print(v, 3);
  Serial.print(" health=");
  Serial.println(g_batt_health);
#endif
}

static Ticker g_battery_ticker(on_battery_sample_tick,
                               BATTERY_SAMPLE_INTERVAL_MS,
                               0,
                               MILLIS);

static Ticker g_led_ticker(on_led_tick,
                           LED_TICK_MS,
                           0,
                           MILLIS);

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------
static inline void battery_init(uint8_t led_pin)
{
  g_batt_led_pin = led_pin;
  pinMode(g_batt_led_pin, OUTPUT);
  led_write_off();

  g_led_pulses_left     = 0;
  g_led_is_on           = false;
}

static inline void battery_start()
{
  on_battery_sample_tick();
  g_battery_ticker.start();
  g_led_ticker.start();
}

static inline void battery_stop()
{
  g_battery_ticker.stop();
  g_led_ticker.stop();
  led_write_off();
}

static inline void battery_update()
{
  // Battery ticker still needs update() if you're using this Ticker library in MILLIS mode.
  // LED ticker too, but keeping both here is fine.
  g_battery_ticker.update();
  g_led_ticker.update();
}

static inline uint16_t battery_pack_health_u16()
{
  return ((uint16_t)g_batt_health << 8);
}
