/*
 * ============================================================================
 *  PH TYPHOON TRACKER
 *  Sensorless Philippines Weather & Typhoon Tracker for ESP32 + SSD1306 OLED
 * ----------------------------------------------------------------------------
 *  - Static Philippines archipelago map stored in flash (PROGMEM)
 *  - Live surface pressure / wind speed for a target point in the Philippine
 *    Area of Responsibility (PAR), fetched from the Open-Meteo REST API
 *    (free, no API key) every 10 minutes
 *  - Animated radar-swirl cyclone marker mapped from GPS coordinates
 *  - Emergency state when surface pressure drops below 1000 hPa:
 *      * flashing "!TYPHOON!" / "ALERT <1000" header banner
 *      * dual-tone siren (1.2 kHz <-> 2.4 kHz, 250 ms alternation)
 *  - Push-button mute with 50 ms software debounce + auto-reset on recovery
 *  - 60-minute pressure-trend readout (hPa/h); the row blinks during a
 *    rapid barometric fall (>= 2 hPa/h) BEFORE the 1000 hPa alert fires
 *  - Fully non-blocking: every timing decision uses millis().
 *    There are NO delay() calls anywhere in loop().
 *
 *  Target board : ESP32 DevKit / NodeMCU-32S (any ESP32 variant)
 *  Build system : PlatformIO (see platformio.ini) - Arduino framework
 *  Framework    : Arduino core for ESP32 (tested against core 3.x API,
 *                 with automatic fallback to the 2.x ledcSetup() API)
 *  Libraries    : Adafruit GFX, Adafruit SSD1306, ArduinoJson v7
 *                 (WiFi.h, HTTPClient.h, Wire.h ship with the core;
 *                 lib_deps in platformio.ini installs the rest)
 *
 *  WIRING
 *  ------
 *   SSD1306 OLED  SDA -> GPIO 21     Buzzer(+)   -> GPIO 18
 *   SSD1306 OLED  SCL -> GPIO 22     Buzzer(-)   -> GND
 *   SSD1306 OLED  VCC -> 3V3          Button     -> GPIO 4 (other leg -> GND)
 *   SSD1306 OLED  GND -> GND          (GPIO4 uses internal INPUT_PULLUP)
 * ============================================================================
 */

#include <Arduino.h>   // must come first in PlatformIO .cpp builds
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

/* ============================================================
 *  USER CONFIGURATION - edit these before flashing
 * ============================================================ */
// Wi-Fi credentials live in include/secrets.h (git-ignored, so your
// password never lands in the repository). Copy include/secrets.h.example
// to include/secrets.h and fill it in. Placeholders are used otherwise.
#if __has_include("secrets.h")
  #include "secrets.h"
#else
  #pragma message("secrets.h not found - building with PLACEHOLDER Wi-Fi credentials")
  #define WIFI_SSID       "YOUR_WIFI_SSID"
  #define WIFI_PASSWORD   "YOUR_WIFI_PASSWORD"
#endif

// Target point inside the PAR that is tracked and sampled.
// Default: Metro Manila. Move this to any storm/LPA coordinate you want.
#define TARGET_LAT      14.60f               // degrees North
#define TARGET_LON      120.98f              // degrees East

// Alert threshold: emergency state below this surface pressure (hPa)
#define ALERT_PRESSURE_HPA  1000.0f

/* ---------------- Pin map ---------------- */
static const uint8_t OLED_SDA   = 21;  // I2C data
static const uint8_t OLED_SCL   = 22;  // I2C clock
static const uint8_t BUZZER_PIN = 18;  // piezo signal pin
static const uint8_t BUTTON_PIN = 4;   // mute button to GND, internal pullup

/* ---------------- Display ---------------- */
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_ADDR       0x3C          // most 0.96" SSD1306 modules
#define OLED_RESET      -1            // no reset pin, share MCU reset
#define HEADER_H        11            // top banner height (px)

/* ---------------- Timing (ms) ---------------- */
const unsigned long FETCH_INTERVAL_MS = 600000UL;  // 10 min, successful fetch
const unsigned long FETCH_RETRY_MS    = 60000UL;   // 1 min retry after failure
const unsigned long WIFI_CHECK_MS     = 15000UL;   // WiFi watchdog period
const unsigned long WIFI_TIMEOUT_MS   = 30000UL;   // initial connect window
const unsigned long SIREN_TOGGLE_MS   = 250UL;     // siren tone alternation
const unsigned long HEADER_BLINK_MS   = 600UL;     // alert banner blink rate
const unsigned long ANIM_FRAME_MS     = 150UL;     // cyclone swirl animation
const unsigned long DEBOUNCE_MS       = 50UL;      // button debounce time

/* ---------------- Pressure trend ---------------- */
const float TREND_FAST_HPA_H = 2.0f;        // faster fall blinks the row
const unsigned long TREND_BLINK_MS = 400UL; // fast-fall blink period

/* ---------------- Storm proximity scan ---------------- */
const float SCAN_RADIUS_DEG  = 2.5f;        // ring radius, ~275 km on ground
const float DEFICIT_NEAR_HPA = 2.0f;        // smoothed deficit -> STORM NEAR
const uint8_t SCAN_POINTS    = 5;           // target + N, E, S, W

/* ---------------- Siren tones (Hz) ---------------- */
const unsigned int SIREN_FREQ_LOW  = 1200;
const unsigned int SIREN_FREQ_HIGH = 2400;

/* ---------------- PAR geographic bounds ---------------- */
// Longitude 116.0E..127.0E maps to X 0..127
// Latitude  21.5N..4.5N   maps to Y 63..0 (north = top of screen)
const float PAR_LON_MIN = 116.0f, PAR_LON_MAX = 127.0f;
const float PAR_LAT_MIN = 4.5f,   PAR_LAT_MAX = 21.5f;

/* ---------------- Telemetry sidebar geometry ---------------- */
const int16_t SB_X = 0, SB_Y = 40, SB_W = 76, SB_H = 24;

/* ---------------- Pressure-trend history ---------------- */
// One sample per successful fetch (10 min apart); 7 samples span 60 min.
const uint8_t TREND_SAMPLES = 7;
float   g_hist[TREND_SAMPLES];   // ring buffer of surface pressures
uint8_t g_histCount = 0;         // valid samples (caps at TREND_SAMPLES)
uint8_t g_histNext  = 0;         // next write slot (= oldest slot when full)

/* ============================================================
 *  LEDC buzzer wrappers (core 3.x pin-based API, 2.x fallback)
 * ============================================================ */
#if __has_include(<esp_arduino_version.h>)
  #include <esp_arduino_version.h>
#endif
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  #define LEDC_API_V3 1
#else
  #define LEDC_API_V3 0
#endif
#if !LEDC_API_V3
  static const uint8_t BUZZER_CHANNEL = 0;   // 2.x uses channels, not pins
#endif

static bool g_buzzerOn = false;

static void buzzerInit() {
#if LEDC_API_V3
  if (!ledcAttach(BUZZER_PIN, SIREN_FREQ_LOW, 10)) {  // pin, freq, 10-bit res
    Serial.println("[BUZZER] ledcAttach failed - no audio this boot");
  }
#else
  ledcSetup(BUZZER_CHANNEL, SIREN_FREQ_LOW, 10);
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
#endif
}

static void buzzerTone(unsigned int freqHz) {
#if LEDC_API_V3
  ledcWriteTone(BUZZER_PIN, freqHz);
#else
  ledcWriteTone(BUZZER_CHANNEL, freqHz);
#endif
  g_buzzerOn = true;
}

static void buzzerSilence() {
#if LEDC_API_V3
  ledcWriteTone(BUZZER_PIN, 0);
#else
  ledcWriteTone(BUZZER_CHANNEL, 0);
#endif
  g_buzzerOn = false;
}

/* ============================================================
 *  PHILIPPINES MAP - 128x64 monochrome bitmap in flash (PROGMEM)
 *  Generated by tools/gen_map.ps1 from simplified coastline
 *  polygons using the exact PAR mapping below.
 *  Format: row-major, 16 bytes/row, MSB = leftmost pixel
 *  (Adafruit_GFX drawBitmap convention).
 * ============================================================ */
const uint8_t PH_MAP[(SCREEN_WIDTH / 8) * SCREEN_HEIGHT] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x03, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xFE, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF,
  0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x1F, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0xFF, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xFF,
  0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x3F, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF,
  0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x07, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF,
  0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x01, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF,
  0xC3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x03, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xE0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F,
  0xFF, 0xC0, 0x70, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x1F, 0x03, 0xC0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xFC, 0x06, 0x00, 0x0F, 0x80,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xFE,
  0x00, 0x00, 0x03, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x0F, 0xFE, 0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xFE, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xF0,
  0x03, 0x00, 0x01, 0xF0, 0x7F, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x7F, 0xFC, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x7F, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x07, 0xFF, 0x00, 0x00, 0x7F, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x07, 0xFF, 0x80, 0x00, 0x7F, 0xFF, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xC0, 0x3C,
  0x7F, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00,
  0x03, 0xFF, 0xC0, 0xFC, 0x7F, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x38, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xF8, 0xFC, 0x7F, 0xC0, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xF8, 0xF8,
  0x1F, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x3F, 0xF8, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF8, 0x03, 0xF0, 0x03, 0x00, 0x00,
  0x00, 0x00, 0x00, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xF8, 0x00,
  0x00, 0x0F, 0xFC, 0x00, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x07, 0x80, 0xC0, 0x00, 0x3F, 0xFE, 0x00, 0x00, 0x00, 0x3C, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00,
  0x00, 0x00, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x03, 0xFF, 0xFF, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x3F, 0x80, 0x3F, 0xFF, 0xFF, 0x00, 0x00, 0x0C, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x7F, 0xF8, 0x1F, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0x80,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x01,
  0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x07, 0x80, 0x00, 0x00, 0x7F, 0xF8, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF0, 0x07, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x0F, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70,
  0x00, 0x00, 0x00, 0x00, 0x01, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,
};
static_assert(sizeof(PH_MAP) == (SCREEN_WIDTH / 8) * SCREEN_HEIGHT,
              "PH_MAP must be exactly 1024 bytes (128x64/8)");

/* ============================================================
 *  GLOBAL STATE
 * ============================================================ */
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Telemetry
float        g_pressureHpA  = NAN;    // last good surface pressure (hPa)
float        g_windKmh      = NAN;    // last good wind speed (km/h)
bool         g_haveData     = false;  // at least one successful fetch
bool         g_lastFetchOk  = true;
bool         g_neverFetched = true;
unsigned long g_lastFetchMs = 0;
float        g_trendRate    = 0.0f;   // hPa/h across the last full window
bool         g_trendValid   = false;  // true once 60 min of history exists
bool         g_trendFast    = false;  // barometer falling >= TREND_FAST_HPA_H

// Storm-proximity scan state
float   g_deficit     = NAN;             // smoothed ring-minus-center (hPa)
bool    g_stormNear   = false;           // smoothed deficit >= DEFICIT_NEAR_HPA
uint8_t g_stormBearing = 255;            // bearing to lowest sector, deg (255=none)
float   g_defHist[3]  = {NAN, NAN, NAN};// raw deficits, median-of-3 filter
uint8_t g_defCount    = 0;
uint8_t g_defNext     = 0;

// Alert / mute state machine
bool g_alertActive = false;
bool g_isMuted     = false;

// Scheduler timestamps
unsigned long g_wifiCheckMs  = 0;
unsigned long g_sirenLastMs  = 0;
unsigned long g_blinkLastMs  = 0;
unsigned long g_animLastMs   = 0;
bool          g_sirenHigh    = false;   // false = 1.2 kHz, true = 2.4 kHz
uint8_t       g_animFrame    = 0;       // swirl animation frame counter
uint8_t       g_headerPhase  = 0;       // alert banner blink phase
bool          g_wifiUp       = false;
bool          g_redraw       = true;    // render-on-demand flag

// Button debounce state
struct BtnState {
  bool          reading;     // last raw reading
  bool          stable;      // debounced stable level
  unsigned long lastChangeMs; // timestamp of last raw-level change
} g_btn = {HIGH, HIGH, 0};

/* ============================================================
 *  COORDINATE MAPPING
 *  lonToX: linear ramp 116.0E -> 0 ... 127.0E -> 127
 *  latToY: inverted ramp 21.5N -> 0 ... 4.5N -> 63 (north on top)
 * ============================================================ */
static int16_t lonToX(float lon) {
  float x = (lon - PAR_LON_MIN) * (SCREEN_WIDTH - 1) / (PAR_LON_MAX - PAR_LON_MIN);
  return (int16_t)constrain(lroundf(x), 0, SCREEN_WIDTH - 1);
}

static int16_t latToY(float lat) {
  float y = (PAR_LAT_MAX - lat) * (SCREEN_HEIGHT - 1) / (PAR_LAT_MAX - PAR_LAT_MIN);
  return (int16_t)constrain(lroundf(y), 0, SCREEN_HEIGHT - 1);
}

/* ============================================================
 *  PRESSURE TREND - 60-minute rolling window
 *  A steadily falling barometer is the classic advance signal that
 *  a deepening low (tropical disturbance) is approaching.
 * ============================================================ */
static void pushPressureSample(float p) {
  g_hist[g_histNext] = p;
  g_histNext = (uint8_t)((g_histNext + 1) % TREND_SAMPLES);
  if (g_histCount < TREND_SAMPLES) g_histCount++;
}

// True once a full hour of samples exists; rate is hPa per hour,
// positive = rising. Right after a push, g_histNext points at the
// oldest sample in the window.
static bool pressureTrend(float &rateHpAPerHour) {
  if (g_histCount < TREND_SAMPLES) return false;
  rateHpAPerHour = g_pressureHpA - g_hist[g_histNext];
  return true;
}

/* ============================================================
 *  NETWORK - Open-Meteo multi-point scan (no API key required)
 *  One request returns all SCAN_POINTS locations as a JSON array:
 *  [0] = target, [1..4] = ring N, E, S, W.
 * ============================================================ */
static bool fetchScan(float &outSurfP, float &outWind,
                      float &outCenterMsl, float outRingMsl[4]) {
  // Ring offsets: latitude step fixed; longitude step widened by
  // 1/cos(lat) so all four ring points sit at the same ground distance.
  const float dLat = SCAN_RADIUS_DEG;
  const float dLon = SCAN_RADIUS_DEG / cosf(TARGET_LAT * DEG_TO_RAD);
  const float lats[SCAN_POINTS] = { TARGET_LAT, TARGET_LAT + dLat, TARGET_LAT,
                                    TARGET_LAT - dLat, TARGET_LAT };
  const float lons[SCAN_POINTS] = { TARGET_LON, TARGET_LON, TARGET_LON + dLon,
                                    TARGET_LON, TARGET_LON - dLon };

  char url[256];
  snprintf(url, sizeof(url), "%s",
           "https://api.open-meteo.com/v1/forecast?latitude=");
  size_t len = strlen(url);
  for (uint8_t i = 0; i < SCAN_POINTS; i++)
    len += snprintf(url + len, sizeof(url) - len, "%.4f,", (double)lats[i]);
  url[len - 1] = '\0';                       // drop the trailing comma
  strncat(url, "&longitude=", sizeof(url) - strlen(url) - 1);
  len = strlen(url);
  for (uint8_t i = 0; i < SCAN_POINTS; i++)
    len += snprintf(url + len, sizeof(url) - len, "%.4f,", (double)lons[i]);
  url[len - 1] = '\0';
  strncat(url,
          "&current=pressure_msl,surface_pressure,wind_speed_10m"
          "&wind_speed_unit=kmh",
          sizeof(url) - strlen(url) - 1);

  WiFiClientSecure tls;
  tls.setInsecure();          // public read-only API; cert not pinned
  tls.setTimeout(15000);

  HTTPClient http;
  if (!http.begin(tls, url)) {
    Serial.println("[NET] http.begin() failed");
    return false;
  }
  http.setConnectTimeout(10000);
  http.setTimeout(15000);
  http.setReuse(false);
  http.addHeader("Accept-Encoding", "identity");   // refuse gzip/deflate

  bool ok = false;
  const int code = http.GET();
  if (code == HTTP_CODE_OK) {
    // Open-Meteo replies with "Transfer-Encoding: chunked". Only getString()
    // de-chunks (it routes through writeToStream()); getStream() returns the
    // RAW wire format, whose hex chunk sizes break the JSON parser.
    const String payload = http.getString();
    Serial.printf("[NET] HTTP %d, payload %u bytes\n", code, (unsigned)payload.length());
    http.end();   // body is fully buffered; close before parsing

    JsonDocument doc;   // ArduinoJson v7: capacity is allocated dynamically
    DeserializationError err = deserializeJson(doc, payload);
    if (err) {
      Serial.printf("[NET] JSON error: %s\n", err.c_str());
      Serial.println("  body[0..119]: " + payload.substring(0, 120));
      return false;
    }

    JsonArrayConst arr = doc.as<JsonArrayConst>();
    if (arr.isNull() || arr.size() < SCAN_POINTS) {
      Serial.printf("[NET] Expected %u locations in array\n", (unsigned)SCAN_POINTS);
      return false;
    }

    // Scan pressures MUST be pressure_msl: raw surface_pressure tracks
    // terrain elevation, so a ring point over mountains would read like a
    // deep low in perfectly fair weather (measured: 904 hPa vs 1013 msl).
    float msl[SCAN_POINTS];
    for (uint8_t i = 0; i < SCAN_POINTS; i++) {
      msl[i] = arr[i]["current"]["pressure_msl"] | NAN;
      if (isnan(msl[i]) || msl[i] <= 850.0f || msl[i] >= 1100.0f) {
        Serial.printf("[NET] Scan point %u out of physical range\n", (unsigned)i);
        return false;
      }
    }

    // Display/alert pressure stays on surface_pressure at the target,
    // matching the 1000 hPa threshold semantics for a coastal target.
    outSurfP = arr[0]["current"]["surface_pressure"] | NAN;
    outWind  = arr[0]["current"]["wind_speed_10m"] | NAN;
    if (isnan(outSurfP) || outSurfP <= 850.0f || outSurfP >= 1100.0f ||
        isnan(outWind)  || outWind < 0.0f     || outWind >= 500.0f) {
      Serial.println("[NET] Values out of physical range, discarded");
      return false;
    }

    outCenterMsl = msl[0];
    for (uint8_t i = 0; i < 4; i++) outRingMsl[i] = msl[i + 1];
    ok = true;
  } else {
    Serial.printf("[NET] HTTP GET failed, code: %d\n", code);
  }
  http.end();
  return ok;
}

/* ============================================================
 *  STORM PROXIMITY SCAN
 *  Compares sea-level-reduced pressure (pressure_msl) at the target
 *  against the mean of four ring points ~275 km out. A deepening low
 *  near the target pulls the center down relative to the ring, while
 *  diurnal and broad-scale swings lift both equally and cancel out.
 * ============================================================ */
static float median3(float a, float b, float c) {
  return fmaxf(fminf(a, b), fminf(fmaxf(a, b), c));
}

static void evaluateStormScan(float centerMsl, const float ringMsl[4]) {
  const float ringMean   = (ringMsl[0] + ringMsl[1] + ringMsl[2] + ringMsl[3]) * 0.25f;
  const float rawDeficit = ringMean - centerMsl;

  // Median-of-3 rejects single-sample glitches (model update jumps etc.)
  g_defHist[g_defNext] = rawDeficit;
  g_defNext = (uint8_t)((g_defNext + 1) % 3);
  if (g_defCount < 3) g_defCount++;
  if (g_defCount < 3) return;              // ~30 min of history needed

  g_deficit = median3(g_defHist[0], g_defHist[1], g_defHist[2]);

  // Bearing toward the lowest ring sector: rough steer TO the low.
  // Ring order is N, E, S, W -> bearings 0, 90, 180, 270.
  uint8_t minIdx = 0;
  for (uint8_t i = 1; i < 4; i++)
    if (ringMsl[i] < ringMsl[minIdx]) minIdx = i;
  const uint8_t bearing = (uint8_t)(minIdx * 90);

  const bool near = (g_deficit >= DEFICIT_NEAR_HPA);
  if (near != g_stormNear) {
    Serial.printf("[SCAN] STORM NEAR %s (deficit %+.1f hPa, bearing %03u deg)\n",
                  near ? "ON" : "off", (double)g_deficit, (unsigned)bearing);
    g_stormNear = near;
  }
  g_stormBearing = near ? bearing : 255;

  Serial.printf("[SCAN] center=%.1f ring=%.1f deficit=%+.1f smooth=%+.1f %s\n",
                (double)centerMsl, (double)ringMean,
                (double)rawDeficit, (double)g_deficit, near ? "NEAR" : "");
}

static void handleFetchSchedule(unsigned long now) {
  // Don't burn a TLS attempt before the first Wi-Fi association exists;
  // once associated, the stale g_lastFetchMs makes this fire immediately.
  if (!g_wifiUp && g_neverFetched) return;

  const unsigned long interval = g_lastFetchOk ? FETCH_INTERVAL_MS : FETCH_RETRY_MS;
  if (!g_neverFetched && (now - g_lastFetchMs) < interval) return;

  g_lastFetchMs  = now;
  g_neverFetched = false;

  float p, w, centerMsl, ringMsl[4];
  g_lastFetchOk = fetchScan(p, w, centerMsl, ringMsl);
  if (g_lastFetchOk) {
    g_pressureHpA = p;
    g_windKmh     = w;
    g_haveData    = true;

    // --- Pressure trend ---------------------------------------------
    pushPressureSample(p);
    float rate;
    if (pressureTrend(rate)) {
      g_trendRate  = rate;
      g_trendValid = true;
      g_trendFast  = (rate <= -TREND_FAST_HPA_H);
      Serial.printf("[NET] Trend %+.1f hPa/h%s\n", (double)rate,
                    g_trendFast ? "  << FAST FALL" : "");
    }

    // --- Storm proximity scan ---------------------------------------
    evaluateStormScan(centerMsl, ringMsl);

    // --- Alert evaluation -------------------------------------------
    const bool newAlert = (p < ALERT_PRESSURE_HPA);
    if (newAlert != g_alertActive) {
      g_alertActive = newAlert;
      Serial.printf("[ALERT] %s (P=%.1f hPa)\n",
                    newAlert ? "ACTIVE" : "cleared", p);
    }
    // Auto-reset: recovering above the threshold always clears the mute
    if (!g_alertActive && g_isMuted) {
      g_isMuted = false;
      Serial.println("[ALERT] Mute auto-reset (pressure recovered)");
    }
    g_redraw = true;
  }
}

static void handleWifiWatchdog(unsigned long now) {
  if ((now - g_wifiCheckMs) < WIFI_CHECK_MS) return;
  g_wifiCheckMs = now;

  const bool up = (WiFi.status() == WL_CONNECTED);
  if (up != g_wifiUp) {
    g_wifiUp = up;
    g_redraw = true;
    Serial.println(up ? "[WIFI] connected" : "[WIFI] link lost");
  }
  if (!up) WiFi.reconnect();   // non-blocking, autoReconnect also enabled
}

/* ============================================================
 *  BUTTON - software debounce (50 ms) + mute toggle
 * ============================================================ */
static void onButtonPress() {
  if (!g_alertActive) return;             // mute is only meaningful in an alert
  g_isMuted = !g_isMuted;
  if (g_isMuted) buzzerSilence();         // silence immediately, visuals stay
  Serial.printf("[BTN] mute toggled -> %s\n", g_isMuted ? "MUTED" : "ON");
  g_redraw = true;
}

static void handleButton(unsigned long now) {
  const bool reading = digitalRead(BUTTON_PIN);       // LOW = pressed (pullup)
  if (reading != g_btn.reading) {                     // raw level changed:
    g_btn.lastChangeMs = now;                         // restart the 50 ms timer
    g_btn.reading      = reading;
  }
  if ((now - g_btn.lastChangeMs) >= DEBOUNCE_MS &&    // level held long enough
      reading != g_btn.stable) {                      // and differs from stable
    g_btn.stable = reading;
    if (g_btn.stable == LOW) onButtonPress();         // falling edge = press
  }
}

/* ============================================================
 *  AUDIO - alternating dual-tone siren
 * ============================================================ */
static void handleSiren(unsigned long now) {
  if (g_alertActive && !g_isMuted) {
    if ((now - g_sirenLastMs) >= SIREN_TOGGLE_MS) {
      g_sirenLastMs = now;
      g_sirenHigh   = !g_sirenHigh;
      buzzerTone(g_sirenHigh ? SIREN_FREQ_HIGH : SIREN_FREQ_LOW);
    }
  } else if (g_buzzerOn) {
    buzzerSilence();   // covers muted, cleared alert, or both
  }
}

/* ============================================================
 *  RENDERING
 * ============================================================ */
static void centerText(const char *text, int16_t y, uint16_t color) {
  display.setTextSize(1);
  display.setTextColor(color);
  display.setCursor((SCREEN_WIDTH - (int16_t)strlen(text) * 6) / 2, y);
  display.print(text);
}

static void drawBanner(const char *text, bool highlighted) {
  if (highlighted) {                       // white bar, black text
    display.fillRect(0, 0, SCREEN_WIDTH, HEADER_H, SSD1306_WHITE);
    centerText(text, 1, SSD1306_BLACK);
  } else {                                 // outlined bar, white text
    display.drawRect(0, 0, SCREEN_WIDTH, HEADER_H, SSD1306_WHITE);
    centerText(text, 1, SSD1306_WHITE);
  }
}

static void drawHeader() {
  display.fillRect(0, 0, SCREEN_WIDTH, HEADER_H, SSD1306_BLACK);
  if (g_alertActive) {
    if (g_isMuted) {
      drawBanner("TYPH [MUTED]", true);                    // steady, highlighted
    } else if (g_headerPhase) {
      drawBanner("!TYPHOON!", true);                       // highlighted phase
    } else {
      drawBanner("ALERT <1000", false);                    // alternate phase
    }
  } else {
    // STORM NEAR: steady inverted banner (the full alert flashes instead)
    if (g_stormNear) drawBanner("STORM NEAR", true);
    else             centerText("PH TYPHOON TRACKER", 1, SSD1306_WHITE);
    // WiFi status pip, top-right corner
    if (g_wifiUp) display.fillCircle(123, 5, 2, SSD1306_WHITE);
    else          display.drawCircle(123, 5, 2, SSD1306_WHITE);
  }
}

static void drawTelemetrySidebar() {
  char buf[24];
  display.fillRect(SB_X, SB_Y, SB_W, SB_H, SSD1306_BLACK);
  display.drawRect(SB_X, SB_Y, SB_W, SB_H, SSD1306_WHITE);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  if (g_haveData) {
    snprintf(buf, sizeof(buf), "P:%.1fhPa", (double)g_pressureHpA);
  } else {
    snprintf(buf, sizeof(buf), "P:--hPa");
  }
  display.setCursor(SB_X + 3, SB_Y + 1);
  display.print(buf);

  if (g_haveData) {
    snprintf(buf, sizeof(buf), "W:%.0fkm/h", (double)g_windKmh);
  } else {
    snprintf(buf, sizeof(buf), "W:--km/h");
  }
  display.setCursor(SB_X + 3, SB_Y + 9);
  display.print(buf);

  // Row 3: 60-min pressure trend. Hidden until a full window exists;
  // blinks during a fast fall so it reads as a warning, not just data.
  if (g_trendValid && !(g_trendFast && ((millis() / TREND_BLINK_MS) & 1))) {
    snprintf(buf, sizeof(buf), "T:%+.1fhPa/h", (double)g_trendRate);
    display.setCursor(SB_X + 3, SB_Y + 17);
    display.print(buf);
  }
}

/* Small arrow from the target toward the inferred bearing of the low.
 * Screen direction: x = sin(bearing), y = -cos(bearing) (north = up). */
static void drawBearingArrow(int16_t cx, int16_t cy, uint8_t bearingDeg) {
  const float rad = bearingDeg * DEG_TO_RAD;
  const float dx = sinf(rad), dy = -cosf(rad);   // unit vector, screen coords
  const float px = -dy, py = dx;                 // perpendicular

  const int16_t tipX = cx + (int16_t)lroundf(dx * 13.0f);
  const int16_t tipY = cy + (int16_t)lroundf(dy * 13.0f);
  const int16_t b1X  = cx + (int16_t)lroundf(dx * 8.0f + px * 3.5f);
  const int16_t b1Y  = cy + (int16_t)lroundf(dy * 8.0f + py * 3.5f);
  const int16_t b2X  = cx + (int16_t)lroundf(dx * 8.0f - px * 3.5f);
  const int16_t b2Y  = cy + (int16_t)lroundf(dy * 8.0f - py * 3.5f);

  display.fillCircle(cx, cy, 7, SSD1306_BLACK);  // clear space under arrow
  display.fillTriangle(tipX, tipY, b1X, b1Y, b2X, b2Y, SSD1306_WHITE);
}

/* Animated counter-clockwise radar-swirl cyclone marker.
 * Northern-hemisphere cyclones spin counter-clockwise, so the swirl does too
 * (screen Y grows downward, hence the negative angle step). */
static void drawCyclone(int16_t cx, int16_t cy) {
  const float R_OUT = 5.5f, R_MID = 3.5f, R_IN = 1.5f;

  // Black clearing disc so the marker stays readable over land or sea
  display.fillCircle(cx, cy, (int16_t)R_OUT + 2, SSD1306_BLACK);

  const float base = -(float)g_animFrame * 30.0f;   // 30 deg CCW per frame
  for (int k = 0; k < 3; k++) {
    const float a0 = base + k * 120.0f;             // 3 arms, 120 deg apart
    const float a1 = a0 + 27.0f;                    // sweep along each arm
    const float a2 = a0 + 55.0f;
    const float r0 = a0 * DEG_TO_RAD, r1 = a1 * DEG_TO_RAD, r2 = a2 * DEG_TO_RAD;

    display.drawLine(cx + (int16_t)lroundf(cosf(r0) * R_IN),
                     cy + (int16_t)lroundf(sinf(r0) * R_IN),
                     cx + (int16_t)lroundf(cosf(r1) * R_MID),
                     cy + (int16_t)lroundf(sinf(r1) * R_MID),
                     SSD1306_WHITE);
    display.drawLine(cx + (int16_t)lroundf(cosf(r1) * R_MID),
                     cy + (int16_t)lroundf(sinf(r1) * R_MID),
                     cx + (int16_t)lroundf(cosf(r2) * R_OUT),
                     cy + (int16_t)lroundf(sinf(r2) * R_OUT),
                     SSD1306_WHITE);
  }

  // Pulsing storm "eye"
  if (g_animFrame & 1) display.fillCircle(cx, cy, 1, SSD1306_WHITE);
  else                 display.drawPixel(cx, cy, SSD1306_WHITE);

  // Crosshair ticks (kept inside the clearing disc)
  display.drawFastHLine(cx - 6, cy, 2, SSD1306_WHITE);
  display.drawFastHLine(cx + 5, cy, 2, SSD1306_WHITE);
  display.drawFastVLine(cx, cy - 6, 2, SSD1306_WHITE);
  display.drawFastVLine(cx, cy + 5, 2, SSD1306_WHITE);
}

static void drawNorthIndicator() {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(118, 13);
  display.print('N');
  display.fillTriangle(121, 25, 127, 25, 124, 20, SSD1306_WHITE); // up arrow
}

static void render() {
  display.clearDisplay();

  // 1. Static map background (full 128x64, PAR bounds = screen edges)
  display.drawBitmap(0, 0, PH_MAP, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);

  // 2. Animated cyclone marker at the mapped target coordinates
  const int16_t cx = lonToX(TARGET_LON), cy = latToY(TARGET_LAT);
  drawCyclone(cx, cy);

  // 2b. Bearing arrow toward the inferred low (storm-near state only)
  if (g_stormNear && g_stormBearing != 255)
    drawBearingArrow(cx, cy, g_stormBearing);

  // 3. North indicator (top-right, over open water east of Luzon)
  drawNorthIndicator();

  // 4. Telemetry sidebar (lower-left overlay)
  drawTelemetrySidebar();

  // 5. Status header (top overlay)
  drawHeader();

  display.display();
}

/* ============================================================
 *  SETUP
 * ============================================================ */
void setup() {
  Serial.begin(115200);
  delay(100);   // boot-time only: let the USB-CDC serial monitor attach

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  buzzerInit();
  buzzerSilence();

  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(400000);   // fast mode: full 1 KB framebuffer ~= 23 ms

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[OLED] SSD1306 init FAILED - check wiring/address");
    while (true) { /* halt: nothing to show on */ }
  }

  // Splash (tracked coordinates shown here; the sidebar row they used
  // to occupy now carries the live pressure trend)
  char coordBuf[18];
  snprintf(coordBuf, sizeof(coordBuf), "%.1fN %.1fE",
           (double)TARGET_LAT, (double)TARGET_LON);
  display.clearDisplay();
  centerText("PH TYPHOON", 8, SSD1306_WHITE);
  centerText("TRACKER v1.2", 22, SSD1306_WHITE);
  centerText(coordBuf, 36, SSD1306_WHITE);
  centerText(WIFI_SSID, 46, SSD1306_WHITE);
  display.display();

  // Initial WiFi association - bounded, non-blocking wait with progress dots
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);          // snappier HTTPS handshakes
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  const unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < WIFI_TIMEOUT_MS) {
    const uint8_t dots = ((millis() - t0) / 350) % 4;
    display.fillRect(0, 54, SCREEN_WIDTH, 10, SSD1306_BLACK);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(34, 54);
    display.print(F("WiFi"));
    for (uint8_t i = 0; i < dots; i++) display.print('.');
    display.display();
  }
  g_wifiUp = (WiFi.status() == WL_CONNECTED);
  Serial.printf("[WIFI] %s (IP %s)\n",
                g_wifiUp ? "connected" : "NOT connected",
                g_wifiUp ? WiFi.localIP().toString().c_str() : "-");

  g_redraw = true;   // first render happens in loop()
}

/* ============================================================
 *  MAIN LOOP - cooperative, strictly non-blocking.
 *  Every branch is guarded by a millis() deadline; there is no
 *  delay() anywhere. Blocking HTTPS work happens only inside its
 *  own scheduled slot (once per 10 min).
 * ============================================================ */
void loop() {
  const unsigned long now = millis();

  handleWifiWatchdog(now);   // reconnect + link-status pip

  handleFetchSchedule(now);  // Open-Meteo poll + alert evaluation

  handleButton(now);         // debounced mute toggle

  handleSiren(now);          // 1.2 kHz <-> 2.4 kHz alternation

  // Animation + banner blink timers.
  // Frame counter wraps at 12 because the swirl steps 30 deg/frame:
  // 12 x 30 deg = 360 deg, so the rotation is seamless across the wrap.
  if ((now - g_animLastMs) >= ANIM_FRAME_MS) {
    g_animLastMs = now;
    g_animFrame = (g_animFrame + 1) % 12;
    g_redraw = true;
  }
  if (g_alertActive && !g_isMuted && (now - g_blinkLastMs) >= HEADER_BLINK_MS) {
    g_blinkLastMs = now;
    g_headerPhase ^= 1;
    g_redraw = true;
  }

  if (g_redraw) {            // render on demand only
    render();
    g_redraw = false;
  }
}
