# PH Typhoon Tracker

Sensorless typhoon & LPA tracker for the Philippine Area of Responsibility: an
ESP32 + 0.96" SSD1306 OLED that renders the PAR map, scans sea-level pressure
on a 5×5 grid across the entire basin with one Open-Meteo request (no API
key), and marks any detected low on the map. A weak system shows as an
`LPA IN PAR` banner with a frozen marker; once central pressure drops below
1000 hPa it escalates to a flashing `!TYPHOON!` banner and a dual-tone siren.
No home location is tracked — the device hunts storms, not the user.

## Features

- **PAR-wide storm scan** — one multi-point API call samples `pressure_msl`
  at 25 grid nodes (~280 × 390 km spacing) covering 117–127°E / 7–21°N;
  local-minimum detection locates the strongest low in the basin
- **Conditional cyclone marker** — appears only when a low is actually
  detected, at its estimated position; frozen swirl for an LPA, spinning
  swirl (northern-hemisphere rotation) once it reaches typhoon strength.
  Calm weather shows a clean map with no indicator
- **Tracked-system telemetry** — sidebar shows the detected low's central
  pressure, model wind, and its 60-minute deepening/filling trend
- **Typhoon alert** — central pressure < 1000 hPa: header alternates
  `!TYPHOON!` / `ALERT <1000` and the buzzer plays an alternating
  1.2 kHz / 2.4 kHz siren every 250 ms
- **Mute button** — debounced push-button silences the siren while the visual
  alert keeps flashing; mute auto-clears when the system weakens ≥ 1000 hPa
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
   `WIFI_SSID` / `WIFI_PASSWORD` (2.4 GHz networks only). That is the only
   configuration required — there is no location to set.
3. Build: **PlatformIO: Build** (or `pio run`).
4. Flash: connect the DevKit over USB and run **PlatformIO: Upload**
   (or `pio run -t upload`).
5. Serial monitor @115200 for diagnostics (`pio device monitor`).

Optional tuning constants live at the top of `src/main.cpp`:
`ALERT_PRESSURE_HPA` (alert threshold), `DEFICIT_LOW_HPA` (detection
sensitivity), `GRID_*` (scan coverage).

## Coordinate mapping math

The screen is the Philippine Area of Responsibility. Both axes are linear
ramps; latitude is inverted because screen Y grows downward while latitude
grows northward:

```
X = round((lon − 116.0°E) × 127 / (127.0 − 116.0))     116°E → x=0 … 127°E → x=127
Y = round((21.5°N − lat) × 63 / (21.5 − 4.5))          21.5°N → y=0 …  4.5°N → y=63
```

Resolution works out to ≈0.087°/px in longitude and ≈0.270°/px in latitude,
so the map is horizontally stretched relative to true Mercator proportions —
a deliberate trade-off to fill the display. The bitmap was rasterized from
coastline polygons using these exact formulas (`tools/gen_map.ps1`,
regenerate with `powershell -File tools/gen_map.ps1`). Detected storm
positions use the same two functions to land the cyclone marker on the map.

## PAR storm scan

Every 10 minutes the device fetches `pressure_msl` at 25 nodes in **one**
request (Open-Meteo accepts comma-separated coordinate lists; lists pair up
element-by-element, so the firmware emits all 25 lat/lon combinations).

Detection (`detectLow` in `src/main.cpp`):

1. Find the minimum-pressure node on the grid.
2. Average its surrounding nodes (up to 8). The minimum must sit at least
   **2.5 hPa below** that average — this rejects broad monsoon troughs,
   where the whole field sinks together and no single center stands out.
   Validated live: the NW Luzon trough produces a ~2 hPa smooth ramp
   (rejected); a tropical depression punches a localized 5+ hPa dent.
3. Refine position with a deficit-weighted centroid of the 3×3 block,
   pulling the estimate between nodes toward the true minimum.

Position accuracy is limited by grid spacing: expect ±150–300 km. This is
inference from the model pressure field, not an official PAGASA track —
treat it as "an organized low is inside the PAR near here", not a forecast.

`pressure_msl` is mandatory, not `surface_pressure`: raw surface pressure
tracks terrain elevation, so grid nodes over the Cordillera (~1800 m) read
~90 hPa "low" in perfectly fair weather (measured: 904 hPa raw vs 1013 hPa
sea-level-reduced at the same spot).

## Alert states

| Condition | Header | Marker | Buzzer |
| --- | --- | --- | --- |
| No low detected | `PH TYPHOON TRACKER` + Wi-Fi pip | none | silent |
| Low detected, central ≥ 1000 hPa | steady `LPA IN PAR` | frozen swirl at estimated position | silent |
| Central < 1000 hPa | alternates `!TYPHOON!` (inverted) / `ALERT <1000` every 600 ms | spinning swirl | 1.2 kHz ↔ 2.4 kHz every 250 ms |
| Central < 1000 hPa, muted | steady `TYPH [MUTED]` | spinning swirl | silent |

Severity hierarchy: LPA tracking is informational and silent; the alert state
flashes and sounds. If a fetch fails, the last known state stays on screen
and the scan retries after 1 minute instead of waiting the full 10.

### Telemetry sidebar

`P:` and `W:` refer to the **tracked system** (estimated central pressure
and model wind at the low), not to any home location. `T:` is the change in
central pressure over the last 60 minutes (7 samples): negative means the
system is deepening. A deepening rate of ≥ 2 hPa/h blinks the row. The row
stays blank until an hour of tracking has accumulated, and history resets
when a system dissipates or leaves the grid.

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
weakened system (≥ 1000 hPa) force-clears the mute (auto-reset).

## API notes

Single request, no key, 25 locations (abbreviated here — the firmware emits
all combinations):

```
https://api.open-meteo.com/v1/forecast?latitude=21.0,21.0,...,7.0&longitude=117.0,119.5,...,127.0&current=pressure_msl,wind_speed_10m&wind_speed_unit=kmh
```

Gotchas learned the hard way:

- **Chunked responses.** Open-Meteo replies with `Transfer-Encoding: chunked`
  and no `Content-Length`. The body must be read with `http.getString()`,
  which de-chunks via `HTTPClient::writeToStream()`; parsing
  `http.getStream()` directly feeds the raw hex chunk sizes to the JSON
  parser and fails with `InvalidInput`. The request also sends
  `Accept-Encoding: identity` so the server never compresses the payload.
- **Coordinate lists are pairwise**, not cartesian: N lats + M lons yields
  min(N, M) locations. Emit every row/column combination explicitly.
- **URL length.** The 25-point URL is ~520 characters — size the request
  buffer with headroom or `snprintf` truncates the query silently.

All returned values are sanity-checked (850–1100 hPa, 0–500 km/h) before
being trusted; any out-of-range node discards the whole scan.

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
