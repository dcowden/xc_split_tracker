// TX TAG: Seeed XIAO nRF52840
// Raw RADIO TX, 1M GFSK, 4-byte packet every 5 ms.
// No BLE, no Serial/USB. Just RF.

/*
  Wiring: none needed for RF-only tag.
*/

#include <Arduino.h>
#include <nrf.h>

#define TAG_ID         2      // change per tag
#define PACKET_LEN     4
#define TX_INTERVAL_MS 5

typedef struct __attribute__((packed)) {
    uint8_t  tag_id;
    uint8_t  seq;
    uint16_t reserved;
} xc_packet_t;

static volatile xc_packet_t tx_packet;
static volatile bool radio_busy = false;

// TIMER0: fires every 5 ms
extern "C" void TIMER0_IRQHandler(void) {
    if (NRF_TIMER0->EVENTS_COMPARE[0]) {
        NRF_TIMER0->EVENTS_COMPARE[0] = 0;

        if (!radio_busy) {
            radio_busy = true;

            tx_packet.seq++;
            NRF_RADIO->PACKETPTR = (uint32_t)&tx_packet;

            NRF_RADIO->TASKS_TXEN = 1;
        }
    }
}

static void init_timer0_for_5ms() {
    NRF_TIMER0->TASKS_STOP = 1;
    NRF_TIMER0->MODE       = TIMER_MODE_MODE_Timer;
    NRF_TIMER0->BITMODE    = TIMER_BITMODE_BITMODE_16Bit;
    NRF_TIMER0->PRESCALER  = 4;                 // 16MHz / 2^4 = 1MHz (1 µs/tick)
    NRF_TIMER0->CC[0]      = TX_INTERVAL_MS * 1000; // 5 ms = 5000 µs
    NRF_TIMER0->SHORTS     = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
    NRF_TIMER0->INTENSET   = TIMER_INTENSET_COMPARE0_Msk;

    NVIC_ClearPendingIRQ(TIMER0_IRQn);
    NVIC_SetPriority(TIMER0_IRQn, 3);
    NVIC_EnableIRQ(TIMER0_IRQn);

    NRF_TIMER0->TASKS_START = 1;
}

// RADIO interrupt
extern "C" void RADIO_IRQHandler(void) {
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

static void radio_init_tx() {
    // HF clock on
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) { }

    // Address / sync word
    NRF_RADIO->PREFIX0     = 0x12UL;
    NRF_RADIO->BASE0       = 0x89ABCDEFUL;
    NRF_RADIO->TXADDRESS   = 0;
    NRF_RADIO->RXADDRESSES = (1 << 0);

    // Channel 80 → 2480 MHz
    NRF_RADIO->FREQUENCY = 80;

    // 1 Mbps GFSK
    NRF_RADIO->MODE = (RADIO_MODE_MODE_Nrf_1Mbit << RADIO_MODE_MODE_Pos);

    // Packet config
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

    // TX power
    NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_Pos8dBm << RADIO_TXPOWER_TXPOWER_Pos);

    NRF_RADIO->PACKETPTR = (uint32_t)&tx_packet;

    // IRQs
    NRF_RADIO->INTENSET = RADIO_INTENSET_READY_Msk | RADIO_INTENSET_END_Msk;
    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_SetPriority(RADIO_IRQn, 2);
    NVIC_EnableIRQ(RADIO_IRQn);
}

void setup() {
    tx_packet.tag_id   = TAG_ID;
    tx_packet.seq      = 0;
    tx_packet.reserved = 0;

    radio_init_tx();
    init_timer0_for_5ms();
}

void loop() {
    __WFE();
    __SEV();
    __WFE();
}
