#pragma once
#include <Arduino.h>
#include <CircularBuffer.hpp>   // From CircularBuffer library
#include "types.h"
#include "config.h"             // <-- RX_RING_CAPACITY lives here

// ======================================================================
// RADIO INPUT RING BUFFER (SPSC)
// ======================================================================
//
// - ISR produces: radio_on_packet_isr()
// - loop() consumes: radio_pop_measurement()
//
// CircularBuffer<T, N> is SPSC-safe as long as:
//   - ISR only ever calls push()
//   - loop() only calls shift()
//   - No removal from ISR
//
// ======================================================================

// Use RX_RING_CAPACITY from config.h
static CircularBuffer<Measurement, RX_RING_CAPACITY> g_rx_queue;

// Called ONLY from RADIO IRQ when a valid packet is received
inline void radio_on_packet_isr(uint8_t tag_id, int8_t rssi_dbm, uint32_t rel_ms)
{
    Measurement m;
    m.rel_ms = rel_ms;
    m.tag_id = tag_id;
    m.rssi   = rssi_dbm;

    if (g_rx_queue.isFull()) {
        // drop oldest
        (void)g_rx_queue.shift();
    }
    g_rx_queue.push(m);
}

// Called ONLY from loop() to pop one measurement
inline bool radio_pop_measurement(Measurement &out)
{
    bool got = false;

    noInterrupts();
    if (!g_rx_queue.isEmpty()) {
        out = g_rx_queue.shift();
        got = true;
    }
    interrupts();

    return got;
}
