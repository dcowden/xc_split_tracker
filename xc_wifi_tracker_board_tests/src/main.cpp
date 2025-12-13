#include "Arduino.h"
#include "HT_st7735.h"
#include "HT_TinyGPS++.h"

TinyGPSPlus GPS;
HT_st7735  st7735;

#define VGNSS_CTRL 3     // GNSS power control
#define PPS_PIN    36    // PPS from UC6580 to ESP32-S3

// ---- Time zone config ----
#define EASTERN_STD_OFFSET_H   (-5)  // winter (EST, UTC-5)
#define EASTERN_DST_OFFSET_H   (-4)  // summer (EDT, UTC-4)

// Use this as the active offset:
static const int UTC_OFFSET_HOURS = EASTERN_STD_OFFSET_H;   // change to DST in summer

// ========== PPS TIMING STATE ==========

volatile unsigned long g_ppsMicros = 0;   // time of last PPS edge (micros)
volatile bool          g_ppsSeen   = false;

void IRAM_ATTR pps_isr()
{
    g_ppsMicros = micros();   // capture exact time of PPS rising edge
    g_ppsSeen   = true;
}

// Helper: microseconds since last PPS
static unsigned long microsSincePPS()
{
    if (!g_ppsSeen) return 0;
    return micros() - g_ppsMicros;
}

// ========== Date helpers ==========

static bool isLeap(int year)
{
    return ((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0);
}

static int daysInMonth(int year, int month)
{
    static const int days[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    if (month == 2 && isLeap(year)) return 29;
    return days[month - 1];
}

static void incrementDate(int &year, int &month, int &day)
{
    int dim = daysInMonth(year, month);
    day++;
    if (day > dim) {
        day = 1;
        month++;
        if (month > 12) {
            month = 1;
            year++;
        }
    }
}

static void decrementDate(int &year, int &month, int &day)
{
    day--;
    if (day < 1) {
        month--;
        if (month < 1) {
            month = 12;
            year--;
        }
        day = daysInMonth(year, month);
    }
}

struct LocalDateTime {
    int  year;
    int  month;
    int  day;
    int  hour;
    int  minute;
    int  second;
    int  millis;   // from PPS, not from NMEA
    bool valid;
};

// Convert GPS UTC date/time to Eastern (seconds only; millis added separately)
static LocalDateTime utcToEastern()
{
    LocalDateTime ldt{};
    ldt.valid = false;

    if (!GPS.date.isValid() || !GPS.time.isValid()) {
        return ldt;
    }

    int year   = GPS.date.year();
    int month  = GPS.date.month();
    int day    = GPS.date.day();
    int hour   = GPS.time.hour();
    int minute = GPS.time.minute();
    int second = GPS.time.second();

    long totalSeconds = (long)hour * 3600L
                      + (long)minute * 60L
                      + (long)second
                      + (long)UTC_OFFSET_HOURS * 3600L;

    const long SECONDS_PER_DAY = 24L * 3600L;

    // Adjust date across day boundaries if needed
    while (totalSeconds < 0) {
        totalSeconds += SECONDS_PER_DAY;
        decrementDate(year, month, day);
    }
    while (totalSeconds >= SECONDS_PER_DAY) {
        totalSeconds -= SECONDS_PER_DAY;
        incrementDate(year, month, day);
    }

    int lh = (int)(totalSeconds / 3600L);
    int lm = (int)((totalSeconds % 3600L) / 60L);
    int ls = (int)(totalSeconds % 60L);

    ldt.year   = year;
    ldt.month  = month;
    ldt.day    = day;
    ldt.hour   = lh;
    ldt.minute = lm;
    ldt.second = ls;
    ldt.millis = 0;   // will be filled from PPS
    ldt.valid  = true;
    return ldt;
}

// ========== Main GPS + TFT logic ==========

void GPS_test(void)
{
    // Power GNSS
    pinMode(VGNSS_CTRL, OUTPUT);
    digitalWrite(VGNSS_CTRL, HIGH);

    // PPS input
    pinMode(PPS_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(PPS_PIN), pps_isr, RISING);

    // GNSS UART
    Serial1.begin(115200, SERIAL_8N1, 33, 34);

    // Debug UART
    Serial.begin(115200);
    delay(200);
    Serial.println("GPS_TEST with PPS-based ms");

    // TFT init
    st7735.st7735_fill_screen(ST7735_BLACK);
    delay(100);
    st7735.st7735_write_str(0, 0, (String)"GPS PPS", Font_7x10);

    unsigned long lastDraw = 0;

    while (1)
    {
        // Feed GPS
        while (Serial1.available() > 0) {
            char c = Serial1.read();
            //Serial.write(c);       // mirror raw NMEA for debugging
            GPS.encode(c);         // feed TinyGPS++
        }

        unsigned long now = millis();
        // Limit redraw rate to ~10 Hz so we don't thrash the TFT
        if (now - lastDraw < 3000) {
            continue;
        }
        lastDraw = now;

        st7735.st7735_fill_screen(ST7735_BLACK);

        if (GPS.time.isValid() && GPS.date.isValid() && GPS.location.isValid() && g_ppsSeen)
        {
            LocalDateTime ldt = utcToEastern();
            if (!ldt.valid) {
                st7735.st7735_write_str(0, 0, (String)"Time invalid", Font_7x10);
                continue;
            }

            // Use PPS to get milliseconds since start of current GPS second
            unsigned long msSincePps = microsSincePPS() / 1000UL;
            if (msSincePps >= 1000UL) {
                msSincePps = msSincePps % 1000UL;
            }
            ldt.millis = (int)msSincePps;

            // Date line
            char dateBuf[24];
            snprintf(dateBuf, sizeof(dateBuf),
                     "%04d-%02d-%02d",
                     ldt.year, ldt.month, ldt.day);
            st7735.st7735_write_str(0, 0, (String)"DAT: " + (String)dateBuf, Font_7x10);

            // Time line: HH:MM:SS.mmm ET
            char timeBuf[40];
            snprintf(timeBuf, sizeof(timeBuf),
                     "%02d:%02d:%02d.%03d ET",
                     ldt.hour, ldt.minute, ldt.second, ldt.millis);
            st7735.st7735_write_str(0, 15, (String)"TIM: " + (String)timeBuf, Font_7x10);

            // Latitude & longitude
            String latStr = "LAT: " + String(GPS.location.lat(), 8);
            st7735.st7735_write_str(0, 30, latStr, Font_7x10);

            String lonStr = "LNG: " + String(GPS.location.lng(), 8);
            st7735.st7735_write_str(0, 45, lonStr, Font_7x10);

            // Altitude & sats
            if (GPS.altitude.isValid()) {
                String alt = "ALT: " + String(GPS.altitude.meters()) + " m";
                st7735.st7735_write_str(0, 60, alt, Font_7x10);
            }

            if (GPS.satellites.isValid()) {
                String sats = "SAT: " + String(GPS.satellites.value());
                st7735.st7735_write_str(80, 60, sats, Font_7x10);
            }
        }
        else
        {
            // No fix or no PPS yet
            st7735.st7735_write_str(0, 0, (String)"Waiting GNSS/PPS", Font_7x10);

            if (GPS.satellites.isValid()) {
                String sats = "SAT: " + String(GPS.satellites.value());
                st7735.st7735_write_str(0, 15, sats, Font_7x10);
            }
            if (!g_ppsSeen) {
                st7735.st7735_write_str(0, 30, (String)"PPS: not seen", Font_7x10);
            }
        }
    }
}

void setup()
{
    delay(100);
    st7735.st7735_init();   // Heltec TFT init
    GPS_test();             // never returns
}

void loop()
{
    // not used; GPS_test has its own while(1)
    delay(100);
}
