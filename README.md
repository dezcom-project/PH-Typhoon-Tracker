# PH Typhoon Tracker

Sensorless Philippines weather & typhoon tracker: an ESP32 + 0.96" SSD1306 OLED
that renders the Philippine archipelago, tracks a storm/LPA coordinate with an
animated radar-swirl marker, pulls live pressure/wind from Open-Meteo (no API
key), and raises a flashing-banner + dual-tone-siren emergency state when
surface pressure drops below 1000 hPa.

## Features

- **Static map** — 128×64 monochrome Philippines bitmap generated from
  simplified coastline polygons and stored in flash (`PROGMEM`)
- **Live telemetry** — surface pressure (hPa) and wind speed (km/h) for any
  target point in the PAR, refreshed every 10 minutes via Open-Meteo
- **Cyclone marker** — GPS coordinates mapped to pixels; counter-clockwise
  swirl animation (northern-hemisphere rotation)
- **Typhoon alert** — header alternates `!TYPHOON!` / `ALERT <1000`; buzzer
  plays an alternating 1.2 kHz / 2.4 kHz siren every 250 ms
- **Mute button** — debounced push-button silences the siren while the visual
  alert keeps flashing; mute auto-clears when pressure recovers ≥ 1000 hPa
- **Pressure trend** — 60-minute rolling barometer readout (`T:-1.4hPa/h` in
  the sidebar); a fall of ≥ 2 hPa/h blinks the row as an early warning before
  the 1000 hPa alert threshold is reached
- **Fully non-blocking** — no `delay()` in `loop()`; everything is scheduled
  with `millis()`

## Hardware

| Item | Detail |
| --- | --- |
| MCU | ESP32 DevKit / NodeMCU-32S (any ESP32, Wi-Fi required) |
| Display | 0.96" SSD1306 OLED, 128×64, I2C address `0x3C` |
| Buzzer | Passive piezo (driven by LEDC PWM square wave) |
| Button | Momentary push button, internal pull-up |
| Toolchain | PlatformIO (VS Code extension or `pio` CLI), `espressif32` platform, Arduino framework — **ESP32 core 3.x** (2.x also supported via automatic LEDC API fallback) |

### Wiring matrix

| Signal | ESP32 pin | Notes |
| --- | --- | --- |
| OLED SDA | **GPIO 21** | default I2C SDA |
| OLED SCL | **GPIO 22** | default I2C SCL |
| OLED VCC | 3V3 | never 5 V — most breakout boards are 3.3 V logic |
| OLED GND | GND | common ground with all modules |
| Buzzer (+) | **GPIO 18** | LEDC PWM tone output |
| Buzzer (−) | GND | |
| Button leg A | **GPIO 4** | `INPUT_PULLUP`, active-LOW |
| Button leg B | GND | no external resistor needed |

All logic is 3.3 V. The OLED draws ~20 mA, the buzzer ~10–30 mA — both fine
from the DevKit's 3V3 regulator.

## Build & flash (PlatformIO)

1. Install the **PlatformIO IDE** extension in VS Code, then open this project
   folder — `platformio.ini` is detected automatically and all libraries from
   `lib_deps` (Adafruit GFX, Adafruit SSD1306, ArduinoJson v7) install on the
   first build.
2. Create your credentials file (git-ignored, stays out of the repo):
   `Copy-Item include\secrets.h.example include\secrets.h` and fill in
   `WIFI_SSID` / `WIFI_PASSWORD` (2.4 GHz networks only).
3. Edit the remaining USER CONFIGURATION block at the top of `src/main.cpp`:
   - `TARGET_LAT` / `TARGET_LON` — the storm/LPA point to track
4. Build: **PlatformIO: Build** (or `pio run`).
5. Flash: connect the DevKit over USB and run **PlatformIO: Upload**
   (or `pio run -t upload`).
6. Serial monitor @115200 for diagnostics (`pio device monitor`).

## Coordinate mapping math

The screen is the Philippine Area of Responsibility. Both axes are linear
ramps; latitude is inverted because screen Y grows downward while latitude
grows northward:

```
X = round((lon − 116.0°E) × 127 / (127.0 − 116.0))     116°E → x=0 … 127°E → x=127
Y = round((21.5°N − lat) × 63 / (21.5 − 4.5))          21.5°N → y=0 …  4.5°N → y=63
```

Example — Manila (14.60°N, 120.98°E):

```
X = (120.98 − 116) × 127 / 11 = 57.5  → 58
Y = (21.5 − 14.60) × 63 / 17 = 25.6  → 26
```

Resolution works out to ≈0.087°/px in longitude and ≈0.270°/px in latitude,
so the map is horizontally stretched relative to true Mercator proportions —
a deliberate trade-off to fill the display, as specified. The bitmap itself
was rasterized from coastline polygons using these exact formulas
(`tools/gen_map.ps1`, regenerate with
`powershell -File tools/gen_map.ps1`). The header band overlays the top rows
(Batanes sits under it) and the telemetry sidebar overlays the lower-left
(Sulu Sea/Palawan area).

## Debounce logic

GPIO 4 uses the internal pull-up, so idle = HIGH, pressed = LOW. A classic
two-level state machine runs on `millis()`:

1. **Raw level** — every pass of `loop()` reads the pin. Any change from the
   previous raw reading restarts a 50 ms timer (`lastChangeMs = now`). This is
   what swallows contact bounce: bouncing only *extends* the timer, it never
   produces events.
2. **Stable level** — only when a raw level has survived unchanged for the
   full 50 ms does it become the new stable level. A press event fires exactly
   once, on the HIGH→LOW transition.

Because the check is `(now − lastChangeMs) >= DEBOUNCE_MS`, unsigned
subtraction, it is immune to the `millis()` rollover after ~49 days.

Pressing during an alert toggles `isMuted`; the buzzer stops immediately but
the banner keeps alternating. Outside an alert the press is ignored, and a
fresh reading ≥ 1000 hPa force-clears the mute (auto-reset).

## Alert system

| Condition | Header | Buzzer |
| --- | --- | --- |
| P ≥ 1000 hPa | `PH TYPHOON TRACKER` + Wi-Fi pip | silent |
| P < 1000 hPa | alternates `!TYPHOON!` (inverted) / `ALERT <1000` every 600 ms | 1.2 kHz ↔ 2.4 kHz every 250 ms |
| P < 1000 hPa, muted | steady `TYPH [MUTED]` | silent |

Threshold applies to Open-Meteo's `surface_pressure`. At coastal targets that
tracks sea-level pressure closely; if you place the target at altitude,
consider swapping the query field to `pressure_msl`.

### Early warning: pressure trend

The sidebar's third row shows the surface-pressure change over the last
60 minutes (7 samples, one per 10-minute fetch). It stays blank until a full
hour of history has accumulated after boot — that is normal, not a fault.
A sustained fall is the classic advance signal of an approaching low; once
the drop reaches **2 hPa/h or faster**, the row blinks (400 ms period) to
flag it before the 1000 hPa alert state itself triggers. The tracked
coordinates were moved to the boot splash to make room for this row.

## API notes

Request (no key):

```
https://api.open-meteo.com/v1/forecast?latitude=14.60&longitude=120.98&current=surface_pressure,wind_speed_10m&wind_speed_unit=kmh
```

Responses are sanity-checked (850–1100 hPa, 0–500 km/h) before being trusted;
failed fetches keep the last good values on screen and retry after 1 minute
instead of waiting the full 10. TLS uses `setInsecure()` since the endpoint is
public and read-only.

**Gotcha: chunked responses.** Open-Meteo replies with
`Transfer-Encoding: chunked` and no `Content-Length`. The body must be read
with `http.getString()`, which de-chunks via `HTTPClient::writeToStream()`;
parsing `http.getStream()` directly feeds the raw hex chunk sizes to the JSON
parser and fails with `InvalidInput`. The request also sends
`Accept-Encoding: identity` so the server never compresses the payload.

## Repository layout

```
platformio.ini                 PlatformIO env (esp32dev, lib_deps, flags)
src/main.cpp                   Firmware (single file, bitmap embedded)
include/secrets.h.example      Wi-Fi credentials template
include/secrets.h              Your real credentials (git-ignored, create it)
githooks/pre-commit            Blocks credentials from being committed
tools/gen_map.ps1              Map rasterizer (polygons -> C PROGMEM array)
tools/ph_map_bitmap.txt        Last generator output (for diffing)
```

## Credential guard

A pre-commit hook keeps credentials out of git history. Activate it once per
clone (already active in this working copy):

```
git config core.hooksPath githooks
```

It **blocks** commits that stage `include/secrets.h` or add any line pairing
`WIFI_SSID` / `WIFI_PASSWORD` with a non-placeholder value, and it **warns**
on other password/secret/token-looking assignments. The placeholder strings
(`YOUR_WIFI_*`) are always allowed, so normal development never trips it.
