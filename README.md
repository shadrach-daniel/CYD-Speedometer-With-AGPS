# CYD-Speedometer-With-AGPS
A integration of assisted gps on neo m6 

# CYD GPS Speedometer V3-Turbo

A GPS speedometer / trip computer for the ESP32-2432S028 "Cheap Yellow Display" (CYD), with A-GPS warm-start injection, satellite caching, Kalman-filtered speed smoothing, and a beautiful analog gauge, running on a hardware-modified NEO-6M at 38400 baud.

- **Target hardware:** ESP32-2432S028 (CYD) + hardware-modded u-blox NEO-6M
- **GPS baud:** 38400
- **Language:** C++ / Arduino

## Table of Contents

1. [Features](#features)
2. [Hardware Requirements](#hardware-requirements)
3. [Hardware Modification: NEO-6M 38400 Baud Boost](#hardware-modification-neo-6m-38400-baud-boost)
4. [Wiring](#wiring)
5. [Software Dependencies](#software-dependencies)
6. [Installation](#installation)
7. [Configuration](#configuration)
8. [How It Works](#how-it-works)
9. [File-System Layout (SPIFFS)](#file-system-layout-spiffs)
10. [A-GPS Strategy](#a-gps-strategy)
11. [Speed Smoothing Pipeline](#speed-smoothing-pipeline)
12. [Troubleshooting](#troubleshooting)
13. [Known Quirks](#known-quirks)
14. [License](#license)
15. [Credits](#credits)

## Features

- **Hardware-boosted NEO-6M at 38400 baud:** pin 15 (CFG COM1) pulled to GND through a 10k ohm resistor forces the module to boot at 38400 instead of the stock 9600. A-GPS injection timing, NMEA stream parsing, and satellite cache batching are all tuned for the higher throughput. Full-potential NEO-6M without buying an 8M / 9 series.
- **Fast A-GPS injection:** injects time, location, and satellite data into the GPS module before fix, drastically reducing TTFF (Time-To-First-Fix).
- **Multi-stage A-GPS fallback:** Super-Hot -> Hot -> Warm -> Offline -> Fully Autonomous.
- **Satellite cache with priority sorting:** persists the last good satellite constellation to SPIFFS and injects it on next boot. Uses an insertion sort / merge sort hybrid with SNR + elevation + recency weighting.
- **Kalman + spring-damper speed smoothing:** smooth, natural needle movement without sacrificing responsiveness.
- **Analog gauge with arc caching:** only redraws the segments the needle touches, dramatically reducing SPI bus traffic.
- **WiFi + NTP via Kalman-filtered offset:** an NTP offset filter that improves accuracy across repeated syncs. Uses time.google.com.
- **IP-based approximate location fallback:** via ip-api.com, used to seed the GPS module before first fix.
- **CPU frequency scaling:** 240 MHz when acquiring fix, drops to 80 MHz once fixed to reduce heat and power.
- **Persistent last fix:** reloads last known latitude/longitude/altitude on cold boot.
- **Compact status bar:** WiFi SSID, A-GPS state, NTP sync, LOC source, and CPU temperature/voltage, all at a glance.
- **AGPS stopwatch:** shows how long the last fix took (time-to-fix telemetry).
- **Boot animation:** progressive bar so you know the unit is alive.
- **Bmorceli launcher compatible:** stores WiFi credentials in NVS on launcher 2.6.10 (older/newer versions use a different credential hash scheme, see [Troubleshooting](#troubleshooting)).
- **Speaker module removed:** the CYD's onboard speaker was desoldered to free up GPIOs for the thermistor / voltage dividers and the GPS UART. The FIX pin on the GPS is still configurable but was left unpopulated in this build.

## Hardware Requirements

| Component | Model | Notes |
|---|---|---|
| MCU + Display | ESP32-2432S028 ("Cheap Yellow Display") | 240x320 ST7789 |
| GPS module | u-blox NEO-6M (or pin-compatible clone) | UART + CFG COM1 pin 15 accessible. Modified to 38400 baud, see Hardware Modification |
| Thermistors (x2) | NTC 10k ohm (B=3950) | One near the ESP32, one near the GPS module |
| Divider resistors | 10k ohm series for NTCs; 480k ohm + 480k ohm for battery / GPS dividers | |
| 10k ohm pull-down | 10k ohm resistor | NEO-6M CFG COM1 (pin 15) to GND baud-select mod |
| Status LED | Standard 5 mm / 3 mm LED | Optional |
| Power | 5 V USB | As required by the CYD |

**Notes**

- **CYD speaker removed.** The onboard speaker / amp module on the CYD was desoldered to reclaim GPIOs. This build uses almost every remaining pin for thermistors, voltage dividers, and the GPS UART. If you re-add the speaker, audit the pin map before flashing.
- Display pins are hard-soldered on the CYD board and fixed inside the LovyanGFX class. No wiring or configuration is needed for the display side.

### Pin Map

Only user-configurable pins are listed. Display pins (SPI bus, DC, CS, backlight) are hard-soldered to the CYD board and fixed inside the LovyanGFX class, so they do not need to be wired or configured.

| Signal | GPIO |
|---|---|
| GPS TX (to ESP RX) | 27 |
| GPS RX (from ESP TX) | 22 |
| Status LED | 26 |
| CPU thermistor | 34 |
| GPS thermistor | 32 |
| Battery divider | 35 |
| GPS voltage divider | 36 |

The GPS module's FIX / timepulse pin is configurable but was left unpopulated in this build, since the CYD is already at its GPIO budget. If you want a hardware fix indicator, wire the FIX pin to any free GPIO and read it in `updateLED()`, replacing the current `gps.location.isValid()` check.

## Hardware Modification: NEO-6M 38400 Baud Boost

The stock u-blox NEO-6M comes up on 9600 baud. That is fine for a stock GPS but leaves headroom on the table: every NMEA sentence, every `$PUBX` injection, and every satellite ephemeris exchange is serialised through 9600 bps, which limits how much of the A-GPS pipeline you can actually exercise.

### Datasheet Reference

- **Document:** u-blox NEO-6 Data Sheet (GPS.G6-HW-09005)
- **URL:** https://content.u-blox.com/sites/default/files/products/documents/NEO-6_DataSheet_%28GPS.G6-HW-09005%29.pdf
- **Relevant pin:** pin 15 (CFG COM1)
- **Behavior:**
  - CFG COM1 floating or high: module boots on the default UART configuration (9600 8N1).
  - CFG COM1 tied to GND through ~10k ohm: module boots on the alternate UART configuration, which on the NEO-6M factory image is 38400 8N1.

### The Mod

```
       NEO-6M (pin 15, CFG COM1)
                  |
              [ 10k ohm ]
                  |
                 GND
```

That single resistor is the whole mod. No firmware flashing of the GPS, no u-center surgery, no `$PUBX,41` gymnastics needed at the protocol level. The module simply powers up already speaking 38400.

### Why It Matters

- **Faster NMEA stream.** A full GSV + GSA + GGA + RMC + VTG cycle at 5 Hz is comfortably within 38400 bps but starts to saturate 9600. At 38400 the firmware sees every sentence, not every other one.
- **A-GPS injection throughput.** Injecting multiple `$GNZDA` / `$GPZDA` / `$GPGGA` / `$PUBX,04` sentences back-to-back is bandwidth-bound. At 38400 the whole warm-start burst finishes in a fraction of the time it would take at 9600, which is why the injection delays in `injectTimeToGPS()`, `injectLocationToGPS()`, and `injectSatelliteData()` are tuned to the tight spacing they use. They assume the higher throughput.
- **Satellite cache replay.** `injectSatelliteData()` walks up to 10 satellites with 5 ms spacing between sentences. At 38400 those fit inside a single GPS scheduler tick; at 9600 they would queue and spill into the next one.
- **No price premium.** You get most of the practical benefit of a NEO-8M / 9 series for the cost of a NEO-6M and one resistor. The module still has the same cold-start characteristics, but warm starts (which is what A-GPS is all about) become noticeably snappier.

### Software Side

`setupGPSBaudRate()` still runs the full protocol: it opens the UART at 9600, sends `$PUBX,41` and a UBX CFG-PRT message, then re-opens at 38400. On a modded module the 9600 phase is just polite noise the GPS ignores, and the 38400 reopen is what actually establishes the link. On an unmodded module the 9600 phase is what negotiates the switch. Either way the firmware ends up at 38400. The mod just makes it deterministic and skips the negotiation cost.

### Without the Mod

Remove or comment out the 9600 phase and leave `GPS_BAUD` at 9600, then re-tune the injection delays in `injectTimeToGPS()`, `injectLocationToGPS()`, and `injectSatelliteData()` upward. Otherwise the burst injections will overrun the module's serial buffer at 9600.

### Datasheet Verification

- **Pin function:** CFG COM1 is a UART configuration strap. Referenced in section 5.2 "Pinout" and section 5.3 "Configuration pins" of GPS.G6-HW-09005.
- **Pull-down value:** ~10k ohm to GND selects the alternate configuration. The internal pull is weak; an external 10k ohm is what sets it reliably on every boot.
- **Resulting baud:** the NEO-6M factory UBX default for the alternate config is 38400 8N1, which matches the firmware's `GPSUART.begin(38400, SERIAL_8N1, GPS_RX, GPS_TX)` reopen.

## Wiring

### GPS Connection

```
       ESP32-2432S028                          NEO-6M
       +--------------+                        +----------+
       |  GPIO 27  ---+------------------------+ TX       |
       |  GPIO 22  ---+------------------------+ RX       |
       |  GND      ---+------------------------+ GND      |
       |  3V3 / 5V ---+------------------------+ VCC      |
       +--------------+

                                               |          |
                                               |  pin 15 -+--[ 10k ohm ]-- GND
                                               |(CFG COM1)|   <- baud-select Directly on the main chip
                                               +----------+
```

### NTC Thermistor Divider

Repeat for GPIO 34 and GPIO 32.

```
       3V3 --[ 10k ohm ]--+-- GPIO 34 / 32
                          |
                      [ NTC 10k ohm ]
                          |
                         GND
```

### Voltage Dividers

For battery (GPIO 35) and GPS rail (GPIO 36).

```
       V+ --[ 480k ohm ]--+-- GPIO 35 / 36
                          |
                      [ 480k ohm ]
                          |
                         GND
```

### Wiring Notes

- The CYD's onboard speaker module is removed in this build. Its GPIOs (and shared I2S/amp circuitry) were freed up for the sensor and UART pins above. If you are starting from a fresh CYD, verify which pins the speaker occupies on your board revision before soldering.
- The NEO-6M FIX pin is intentionally left unconnected. It is configurable (any free GPIO can read it), but the CYD has no spare pins. `updateLED()` currently uses `gps.location.isValid()` as the fix indicator; swap it to the FIX pin if you free up a GPIO.
- The CYD display pins (SPI bus, DC, CS, backlight) are hard-soldered on the board and fixed in the LovyanGFX class, so there is nothing to wire.

## Software Dependencies

| Library | Version tested | Author | Purpose |
|---|---|---|---|
| LovyanGFX | >= 1.1.9 | lovyan03 | ST7789 display driver with DMA |
| TinyGPSPlus | >= 1.0.3 | Mikal Hart | NMEA parsing |
| ArduinoJson | >= 6.x | Benoit Blanchon | Parsing ip-api.com JSON |
| ESP32 Arduino Core | >= 2.0.9 (or 3.x) | Espressif | WiFi, HTTPClient, SPIFFS, time |

No additional third-party libraries are required. Everything else (WiFi, HTTPClient, SPIFFS, FS, time, math) ships with the ESP32 Arduino core.

## Installation

### Board Setup

1. Install the ESP32 Arduino core (Board Manager -> esp32 by Espressif).
2. Select Board: ESP32 Dev Module
3. Partition Scheme: Default 4MB with spiffs (or Huge APP if you disable SPIFFS-heavy caching)
4. Upload Speed: 921600 (or 460800 if flaky)
5. CPU Frequency: 240 MHz
6. Flash Frequency: 80 MHz

### WiFi Credentials

This sketch does not call `WiFi.begin("ssid", "password")`. It relies on credentials already stored in NVS by the launcher (e.g. Bmorceli Launcher 2.6.10) or by a prior `WiFi.begin()` in a different sketch.

**If never provisioned:** add a one-time provisioning call inside `setup()`: `WiFi.begin("YourSSID", "YourPassword");`. Build and flash once, then remove the line. The ESP32 will persist the credentials.

> **Warning: launcher version pin.** Bmorceli Launcher 2.6.10 stores WiFi credentials in the NVS as UTF-8, which this firmware expects. Newer launcher builds store credentials as a hash, and the sketch will silently fail to connect. Pin the launcher to 2.6.10 or use the one-time `WiFi.begin()` above.

### Hardware Mod First

Solder a 10k ohm resistor from the NEO-6M's pin 15 (CFG COM1) to GND before powering the module. See [Hardware Modification](#hardware-modification-neo-6m-38400-baud-boost). Without it, the firmware still works, but the A-GPS burst timing assumes 38400 and will be sloppy at 9600.

### Upload

1. Open `CYD_GPS_Trip_V3-Turbo.ino` in the Arduino IDE.
2. Connect the CYD via USB.
3. Click Upload.
4. Open Serial Monitor at 115200 baud to see boot logs.

## Configuration

All tunables are `#define`s near the top of the sketch. The most useful ones are grouped below.

### GPS / A-GPS

| Constant | Default | Meaning |
|---|---|---|
| `GPS_BAUD` | 57600 | Legacy define. Actual UART is reconfigured by `setupGPSBaudRate()` to 38400 (modded module), matching the CFG COM1 pull-down |
| `AGPS_START_DELAY_MS` | 3000 | Delay before A-GPS injection begins |
| `SUPER_HOT_TIMEOUT_MS` | 300000 | 5 min, time spent in SUPER_HOT before falling back to HOT |
| `HOT_TIMEOUT_MS` | 300000 | 5 min, time per HOT / WARM stage |
| `STOPWATCH_DISPLAY_DURATION` | 180000 | 3 min, how long the fix-time readout stays visible |

The injection delays inside `injectTimeToGPS()` (3-5 ms), `injectLocationToGPS()` (2-5 ms), and `injectSatelliteData()` (5 ms) are tuned for 38400 baud. If you revert to stock 9600, scale them up, otherwise burst injections will overrun the module's serial buffer.

### NTP

| Constant | Default | Meaning |
|---|---|---|
| `FAST_NTP_TIMEOUT_MS` | 500 | Per-attempt timeout |
| `FAST_NTP_INTERVAL` | 500 | Interval between attempts |
| `NTP_BURST_INTERVAL` | 100 | Burst spacing for fast sync |

NTP server: time.google.com (change `ntpServer` if desired).

### Speed Smoothing

| Constant | Default | Meaning |
|---|---|---|
| `SPEED_SMOOTHING_STIFFNESS` | 0.11 | Spring stiffness. Lower = smoother, higher = snappier |
| `SPEED_SMOOTHING_DAMPING` | 0.60 | Damping. Higher = less oscillation |
| `SPEED_MAX_ACCELERATION` | 12.0 | Max km/h change per second |
| `SPEED_KALMAN_Q` | 0.05 | Kalman process noise |
| `SPEED_KALMAN_R` | 0.3 | Kalman measurement noise |
| `SPEED_MAX_VALUE` | 140.0 | Full-scale gauge value |

### CPU Frequency

| Constant | Default | Meaning |
|---|---|---|
| `CPU_FREQ_HIGH` | 240 | MHz during fix acquisition |
| `CPU_FREQ_LOW` | 80 | MHz after fix is acquired |

### Display Refresh

| Constant | Default | Meaning |
|---|---|---|
| `REFRESH_MS_ALT` | 1000 | Lat/Lon refresh rate |
| `REFRESH_MS_SPEED` | 10 | Speed / needle refresh rate |

## How It Works

### Boot Sequence

```
setup()
 |-- Mount SPIFFS ----------------> format if fails
 |-- Load last fix ---------------> inject into GPS
 |-- Load satellite cache --------> inject into GPS
 |-- tft.init() + boot animation
 |-- setupGPSBaudRate() ----------> negotiate / reopen at 38400
 |-- initializeArcCache()
 |-- WiFi.begin()  (NVS creds)
 +-- xTaskCreatePinnedToCore(agpsTaskAdaptive, core 0)
```

`loop()` on core 1 handles: GPS parsing, gauge drawing, status display, sensors, CPU frequency scaling.

### Core 0: A-GPS Task

`agpsTaskAdaptive()` runs the A-GPS state machine independently of the display loop, so the UI never freezes waiting for the fix.

### Core 1: Main Loop

- `readGPS()`: feeds TinyGPSPlus, collects NMEA, captures satellite data
- `drawStats()`: gauge + speed + NMEA line
- `updateStatusDisplay()`: WiFi / AGPS / NTP / LOC rows
- `updateLED()`: HIGH when fix acquired
- `updateSensors()`: NTC + voltage with EMA filter (alpha=0.25)
- `manageCpuFrequency()`: drops to 80 MHz once fix acquired

## File-System Layout (SPIFFS)

SPIFFS paths used by the firmware.

| Path | Content |
|---|---|
| `/lastfix.txt` | CSV: lat,lon,alt,course,timestamp. Last known good fix |
| `/sat_data.txt` | CSV: one SAT,prn,elev,azim,snr,used,priority per line |
| `/sat_data.tmp` | Atomic-write staging file |
| `/sat_data.bak` | Rotated backup of previous satellite cache |

The satellite cache is written atomically (write to .tmp, rename to .bak, rename .tmp to .txt) so a power loss mid-write cannot corrupt the cache.

## A-GPS Strategy

The task progresses through four fallback stages with timeouts, so it always tries the fastest option first, then degrades gracefully. All stages assume a 38400 baud link (modded module). The injection burst timing is sized for that throughput.

```
              +--------------+
              |   AGPS_IDLE  |
              +------+-------+
                     | (WiFi up + injected time/pos/sats)
                     v
              +---------------+   5 min  +--------------+
              | AGPS_SUPER_HOT|--------->|   AGPS_HOT   |
              +------+--------+          +------+-------+
                     | fixed                    | 5 min
                     v                          v
              +--------------+           +--------------+
              |  AGPS_FIXED  |           |  AGPS_WARM   |
              +--------------+           +------+-------+
                                                | 5 min
                                                v
                                         +-----------------+
                                         |AGPS_OFFLINE_MODE|
                                         +-----------------+
```

| Stage | Description |
|---|---|
| SUPER_HOT | WiFi is up, time + approximate location + satellite constellation all injected. Fastest possible TTFF. |
| HOT | No WiFi but cached time / location / satellites exist, injected from SPIFFS. |
| WARM | Only `$PUBX,41,1,0007` warm-start command; no external data available. |
| OFFLINE | Every 5 minutes, re-injects everything from cache and restarts the stopwatch. Keeps trying. |

If WiFi comes back while in offline mode, the state machine auto-upgrades back to SUPER_HOT and injects again.

## Speed Smoothing Pipeline

Raw GPS speed is noisy (especially at low speed and low SNR). We chain three filters:

```
raw km/h
   |
   |--> Kalman (Q=0.05, R=0.3) -----------------> kalmanSpeed
   |
   |--> Spring-damper (stiff=0.11, damp=0.60) --> smoothSpeed
   |
   +--> Clamp to max accel = 12 km/h/s ---------> displaySpeed
                                                     |
                                                     v
                                             gauge + needle
```

1. Kalman removes measurement noise at a mathematically optimal rate.
2. Spring-damper adds "weight" to the needle, giving physical, natural motion.
3. Clamp prevents the needle from teleporting when GPS jumps (e.g. when a fix is first acquired).
4. Top speed is tracked with its own EMA (alpha = 0.2) so it does not jump on a single noisy sample.

The needle angle itself is also smoothed (`smoothNeedleAngle += angleDiff * 0.3`).

## Troubleshooting

### "WiFi: N/A" forever

- Verify the Bmorceli Launcher version is 2.6.10. Newer versions store credentials as a hash and the sketch cannot decode them.
- Or add a one-time `WiFi.begin("SSID", "PASS")` to `setup()` and re-flash.
- Check Serial Monitor for the SSID at boot.

### GPS still runs at 9600, not 38400

- Check the CFG COM1 pull-down. Pin 15 on the NEO-6M must be tied to GND through ~10k ohm. Floating or high leaves the module on the stock 9600.
- Verify with a serial sniffer: at 38400 you should see clean `$GPGGA` / `$GPRMC` sentences; at 9600 they will be garbled. If garbled at 38400, the pull-down is not making contact.
- Some clone NEO-6M boards route CFG COM1 to a header pin but not to the actual module. Trace continuity from the module pad, not the header.

### "AGPS: Offline" but no fix after 15 minutes

- Check GPS wiring (TX to GPIO 27, RX to GPIO 22, GND, VCC).
- Confirm GPS module has a clear view of sky (or a window).
- Confirm the 10k ohm CFG COM1 pull-down is in place (see above).
- Confirm `$PUBX,41` is supported (u-blox modules). MediaTek modules use `$PMTK` instead.

### "CV: ... GV: ..." values are wrong

- Adjust `B_COEFFICIENT` to your NTC's datasheet value (common: 3950, 3435, 3977).
- If readings are off by a constant, tune the +1.35 offset in `readTempC()`.
- The voltage calc applies a (4.33 / 3.64) scale, which is a hardware calibration factor for that specific unit's ADC. If your divider reads wrong, change the 4.33 / 3.64 factor in `readVoltage()` where 4.33 = measured mV at the divider and 3.64 = reference.

### Speed reads 0 even with fix

- `haveGoodFix = (satellites >= 3 && location.valid)`. A 3D fix requires at least 4 satellites for altitude, but speed can work with 3. Check `gps.satellites.value()` in the top notch.
- TinyGPSPlus takes a few seconds to fill `gps.speed` after `gps.location` becomes valid. Wait 10-30 s.

### Display colours inverted / rotated

- The CYD panel is inverted (`cfg.invert = true`); if your unit shows inverted colours, flip to false.
- To rotate the UI 180 degrees, change `tft.setRotation(0)` to `tft.setRotation(2)`.

### Needle jitter or lag

- Jittery: decrease `SPEED_KALMAN_R` (more trust in raw measurement) and/or increase `SPEED_SMOOTHING_DAMPING`.
- Laggy: decrease `SPEED_SMOOTHING_STIFFNESS` (currently 0.11) and/or decrease `SPEED_MAX_ACCELERATION`.
- Teleporting on fix: leave `SPEED_MAX_ACCELERATION` at 12 or lower.

### SPIFFS mount failed

- The sketch will auto-format SPIFFS. If your partition scheme is set to something without SPIFFS (e.g. Huge APP), change it back to Default 4MB with spiffs.

## Known Quirks

- A-GPS burst timing is calibrated for 38400 baud. If you run an unmodded NEO-6M at 9600, the injection delays in `injectTimeToGPS()`, `injectLocationToGPS()`, and `injectSatelliteData()` will overflow the module's serial buffer. Scale them up or accept partial injections.
- A-GPS best-case requires WiFi + a valid network route for ip-api.com. If the router blocks HTTP to that host, LOC falls back to cache.
- Time is only injected to GPS after an NTP sync. If NTP never succeeds, the firmware falls back to compile time for the offline mode.
- The NMEA row is intentionally simplified: it prints `GPS:ACTIVE!` rather than raw NMEA. Raw NMEA is disabled to keep the refresh rate high. If you want raw NMEA output, modify `drawNMEALog()` to `tft.print(lastDisplayedNMEA)`.
- WiFi retry backoff grows up to 5 minutes. If you change networks, reset the device.
- CPU frequency drop to 80 MHz happens after fix. If you need lower latency for some reason, comment out `manageCpuFrequency()`.
- NTC calibration (see Troubleshooting) is per-unit. See "CV / GV values are wrong".
- The gauge draws labels via a custom 11-point table `{0,10,20,40,50,70,80,90,110,120,140}`, not evenly spaced. This is intentional so labels fit inside the arc. Change `labels[]` if you want a different scale.
- Speaker removed / FIX pin unpopulated. The CYD's onboard speaker module was desoldered to reclaim GPIOs. The NEO-6M FIX pin is configurable but left unconnected in this build; `updateLED()` uses the software fix flag instead.
- Display pins are hard-soldered on the CYD. The LovyanGFX class is fixed to the board's onboard wiring and cannot be re-mapped without board rework.

## License

Choose one and add it. Recommended for a personal GitHub project:

- **MIT:** permissive, works for commercial reuse.
- **GPL-3.0:** requires derivative projects to be open-source.
- **CERN-OHL-v2:** if you also publish a PCB / wiring schematic.

Add a `LICENSE` file to the repo root with the chosen license text.

## Credits

### Libraries and Services

| Name | Author | Role |
|---|---|---|
| LovyanGFX | lovyan03 | Display driver and DMA-like wrappers |
| TinyGPSPlus | Mikal Hart | NMEA parsing |
| ArduinoJson | Benoit Blanchon | JSON parsing for ip-api.com |
| ip-api.com | | Free IP geolocation service (rate-limited to 45 req/min, well above what this firmware uses) |
| ESP32 Arduino Core | Espressif | WiFi, SPIFFS, and time |
| [u-blox NEO-6 Data Sheet (GPS.G6-HW-09005)](https://content.u-blox.com/sites/default/files/products/documents/NEO-6_DataSheet_%28GPS.G6-HW-09005%29.pdf) | | CFG COM1 (pin 15) baud-select strap documentation |

### Special Thanks: Bmorceli Launcher (2.6.10)

The Bmorceli Launcher project has been a foundational resource for this firmware, providing the WiFi credential provisioning system in NVS this sketch relies on, and serving as a wellspring of knowledge on NVS handling, launcher architecture, credential storage, and practical ESP32 patterns. Without it this project would not exist in its current form.

**Version pin warning:** pin the launcher to 2.6.10. Newer versions store credentials as a hash which this firmware cannot decode, and the sketch will silently fail to connect. Older versions are untested.

Heartfelt thanks to the Bmorceli Launcher project and its contributors. Your work made this build possible.

### Adaptation Note

If you have adapted any code from other open-source repositories, add the original author + license here.

---

Built for the ESP32-2432S028 "Cheap Yellow Display" + hardware-modded NEO-6M @ 38400 baud. Flash responsibly. Pull requests welcome.
