# Drone-Based Air Quality Monitoring System — ESP32 Firmware

Firmware for the drone-mounted air quality monitoring **payload**, using **LoRa / LoRaWAN** telemetry. An
`ESP32-WROOM-32` node acquires, logs, and relays environmental data (gas, particulate matter, CO₂), geotagged via GPS,
with event-triggered imaging.

> The **ground station** (receiver firmware + laptop dashboard) is a separate deployable and lives in its own repo:
> **[aqm-ground-station](https://github.com/ukemeikot/aqm-ground-station)**.

Built with [PlatformIO](https://platformio.org/) on the Arduino-ESP32 framework.

> Based on the project design by Essien I. J., Ukpong K. L., James E. G., Herkins H. E., and Ntuk G. I. —
> Department of Electrical / Electronic Engineering, Akwa Ibom State University.

## Overview

| Node | Repo | Role |
|---|---|---|
| **Drone node** | this repo | Samples all sensors, geotags with GPS, logs to SD, transmits over LoRa, triggers the camera on threshold events. |
| **Ground node** | [aqm-ground-station](https://github.com/ukemeikot/aqm-ground-station) | Receives LoRa packets, validates them, and streams decoded NDJSON to a Streamlit dashboard over USB serial. |

### Sensors & peripherals

- **Gas (electrochemical):** Winsen ME4-H2, ME4-O3, ME4-3SO2, SPEC 3SP-CO-1000 — via OPA4192 TIAs and an ADS1115 16-bit ADC (I²C)
- **CO₂:** MH-Z19C (NDIR)
- **Particulate matter:** Plantower PMS5003 (PM1.0 / PM2.5 / PM10)
- **Position:** u-blox NEO-M8N GPS
- **Radio:** RAK3172 LoRaWAN module (P2P mode first)
- **Storage:** microSD (FAT32)
- **Imaging:** event-triggered camera module

## Repository layout

```
esp32-project/
├─ platformio.ini   # PlatformIO build configuration
├─ src/             # firmware sources (compiled into the image)
├─ lib/             # private/shared drivers (compiled)
├─ include/         # headers
├─ test/            # unit tests
└─ docs/            # design & implementation docs (NOT compiled/flashed)
   └─ IMPLEMENTATION_PLAN.md
```

## Getting started

Requires [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation/).

```bash
# Build drone-node firmware
pio run

# Flash + serial monitor
pio run -t upload -t monitor

# Run unit tests
pio test -e native
```

## Documentation

- **[Implementation Plan](docs/IMPLEMENTATION_PLAN.md)** — firmware architecture, performance strategy, hardware
  interface map, phased milestones, and the LoRa payload format.
- **[Wiring / Circuit Diagram](docs/WIRING.md)** — as-coded pin map and Mermaid wiring diagram (matches `config.h`).

## Status

Project scaffolding. Firmware modules are implemented incrementally on the bench (sensor-by-sensor) before integration,
following the phased plan in the docs.

## License

No license specified yet.
