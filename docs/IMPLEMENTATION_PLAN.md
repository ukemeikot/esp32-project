# Firmware Implementation Plan
## Drone-Based Air Quality Monitoring System (LoRaWAN / LoRa P2P)

**Target hardware:** 2 × ESP32-WROOM-32 (drone node + ground node)
**Framework:** Arduino-ESP32 on PlatformIO (`board = esp32dev`)
**Source documents:** *Updated Design and Implementation* (academic design, Ch. 1–3) and *Drone_AQM_Implementation_Guide* (practical build/test plan).

> **Scope note.** This repository is the **firmware** for the ESP32 processing units. The drone airframe, Pixhawk flight
> control, motors/ESCs and their calibration (PDF Phases 4–5) are electrically and logically independent of this code —
> they share only the battery. This plan therefore concentrates on the monitoring payload firmware and the ground-station
> firmware, and references the hardware phases only where the firmware must interlock with them.

---

## 0. Where this document lives (and why it never reaches the board)

PlatformIO's build system compiles **only** `src/` and `lib/` (plus resolved library dependencies under `.pio/`). Anything
under `docs/`, `README`, `test/` fixtures, etc. is ignored by the compiler and the linker and is **never** part of the
firmware image flashed to the ESP32.

```
esp32-project/
├─ platformio.ini      # build config (compiled settings)
├─ src/                # <-- ONLY this is compiled into the .bin
├─ lib/                # <-- and private libraries here
├─ include/            # headers
├─ test/              # unit tests (host/target, not in the shipped image)
└─ docs/               # <-- THIS PLAN. Documentation only. Never flashed.
   └─ IMPLEMENTATION_PLAN.md
```

Rule we will hold to: **no prose, no design notes, no large lookup tables-as-comments bloat inside `src/`** beyond what the
code needs. Design lives in `docs/`; code stays lean so the image stays small and the build stays fast.

---

## 1. Hardware interface map (from the wiring diagram)

Pin assignments taken from the PDF circuit diagram. **UART budget is the single biggest constraint** and is addressed in §1.1.

| Peripheral | Bus | ESP32 pins (per diagram) | Logic | Notes |
|---|---|---|---|---|
| ADS1115 ADC (4× TIA outputs) | I²C @ 0x48 | SDA=GPIO21, SCL=GPIO22 | 3.3 V | Digitises the 4 electrochemical channels |
| RAK3172 LoRa | UART | RX=GPIO16, TX=GPIO17 (UART2) | 3.3 V | AT-command / P2P |
| MH-Z19C CO₂ | UART | GPIO25 / GPIO26 | 5 V → LLC | 9600 baud; **PWM mode is an option** (see §1.1) |
| PMS5003 PM | UART | GPIO32 / GPIO33 | 5 V → LLC | 9600 baud, passive/active |
| NEO-M8N GPS | UART | RX=GPIO34 (input-only) | 3.3 V | 9600 NMEA; TX-to-GPS needs a spare output pin for config |
| SD card | SPI | SCK=18, MISO=19, MOSI=23, CS=5 | 3.3 V | FAT32, primary log |
| Camera trigger | GPIO | GPIO4 | 3.3 V | Event-triggered capture |

### 1.1 The UART shortage — a design decision that must be made first

The ESP32-WROOM-32 has **three** hardware UART controllers: UART0 (used for USB/serial debug and flashing), UART1, UART2.
The payload needs **four** serial devices: RAK3172, GPS, MH-Z19C, PMS5003. Four devices, two free controllers.

Because the code must be **performant**, bit-banged `SoftwareSerial` is the wrong default — it burns CPU cycles and disables
interrupts during frame reception, which will jitter the sensor loop. Recommended resolution, in order of preference:

1. **Free one UART by running MH-Z19C in PWM mode.** The MH-Z19C outputs CO₂ as a PWM duty cycle on its PWM pin. Read it
   with a single `IRAM_ATTR` edge-capture ISR + the MCPWM/PCNT capture peripheral — near-zero CPU cost. This drops us to
   three UART devices → RAK3172 (UART2), GPS (UART1), PMS5003 (…).
2. **PMS5003 on the third channel.** Options, cheapest-first: (a) share UART0 by reading PMS5003 on the RX-only path and
   keeping debug output-only over the same pins is fragile — avoid. (b) Add an **SC16IS752 I²C↔dual-UART bridge** ($1–2) on
   the existing I²C bus: fully hardware-buffered, zero bit-banging, keeps the CPU free. **This is the recommended clean
   solution** given the performance requirement.
3. If the bridge is rejected, accept **one** `EspSoftwareSerial` instance for PMS5003 only (9600 baud, low duty), pinned to
   the core that is *not* running the sensor state machine, and read it via its RX interrupt into a ring buffer.

> **Decision required from the team.** Default assumption for the rest of this plan: **MH-Z19C in PWM mode + SC16IS752 bridge
> for PMS5003.** If hardware can't change, we fall back to option 3. This choice affects the wiring, so confirm before Phase 1.5.

---

## 2. Firmware architecture

Two firmware images share ~80% of their code through `lib/`:

- **`drone-node`** — acquires all sensors, geotags, logs to SD, transmits over LoRa, handles event triggers + camera.
- **`ground-node`** — receives LoRa packets, validates CRC, and streams decoded JSON over USB serial to the laptop dashboard.

We build both from one repository using PlatformIO **build environments** (`[env:drone]`, `[env:ground]`) selected by a
compile-time flag, so shared drivers are compiled once and never diverge.

### 2.1 Concurrency model (FreeRTOS, dual-core)

The ESP32 is dual-core (PRO_CPU=core 0, APP_CPU=core 1). We exploit both and **pin tasks to cores** to keep the acquisition
path deterministic and isolated from the (bursty, blocking) radio/SD I/O.

| Task | Core | Priority | Responsibility |
|---|---|---|---|
| `acq_task` | 1 | high | Deterministic sensor sampling tick (2–5 s), ADS1115 reads, GPS/PM/CO₂ decode. Never blocks on radio or SD. |
| `log_task` | 0 | med | Drains a queue to the SD card in batched writes. |
| `radio_task` | 0 | med | LoRa TX/RX, AT-command state machine, retry/backoff. |
| `event_task` | 0 | high | Threshold watchdog → camera trigger → immediate alert packet. |
| ISRs | — | — | UART/PCNT capture, timer tick — all `IRAM_ATTR`. |

Inter-task hand-off is via **statically allocated FreeRTOS queues** (`xQueueCreateStatic`) carrying fixed-size POD records —
no dynamic allocation, no `String`, no `std::vector` in the steady state. A malloc-free hot path is what makes the loop
jitter-bounded and the heap non-fragmenting over a multi-hour flight.

### 2.2 Data flow

```
electrochemical cells --> OPA4192 TIA --> RC LPF --> ADS1115 --(I2C)-->┐
MH-Z19C (PWM capture) ---------------------------------------------->  ├─> acq_task builds a Record{}
PMS5003 (UART/bridge) ---------------------------------------------->  │      (fixed-point, packed)
NEO-M8N GPS (UART/NMEA) -------------------------------------------->  ┘        │
                                                                                ├─> log_task  --> SD (CSV/binary)
                                                                                └─> radio_task --> LoRa P2P --> ground node --> USB JSON --> dashboard
```

---

## 3. Performance-first design principles

This workload is fundamentally **I/O-bound**, not compute-bound: at a 2–5 s sample period the CPU is idle most of the time.
So the honest engineering position is: **most performance comes from architecture, not from assembly.** We optimise in this
order, and only drop to assembly where profiling proves it pays.

### 3.1 Structural performance (applies everywhere — do these first)
- **Integer / fixed-point math only** in the acquisition and packet path. Sensor linearisation, unit scaling, and filtering
  use `int32`/`Q16.16` fixed-point, not `float`/`double`. (The ESP32 LX6 has a single-precision FPU but no double FPU; doubles
  are software-emulated and slow. Avoid `double` entirely; avoid `float` in hot loops.)
- **No heap in the steady state.** Static queues/buffers, `char[]` not `String`, no per-loop `new`/`malloc`. Pre-allocate at boot.
- **DMA / hardware-buffered UART.** Use the ESP32 UART driver's built-in RX ring buffer and event queue rather than polling
  `Serial.read()`. For the SC16IS752 bridge, batch-read its FIFO over I²C.
- **`IRAM_ATTR` on every ISR** so they don't fault/stall on flash-cache misses; keep ISRs to "timestamp + push to ring buffer".
- **Pin tasks to cores** (§2.1) and size stacks statically; measure high-water marks with `uxTaskGetStackHighWaterMark`.
- **Batched SD writes.** Accumulate N records and flush on a 512-byte boundary; never `flush()` per sample. Card latency is
  the dominant blocking cost — keep it off core 1.
- **Compiler optimisation:** build with `-O2` (default release) and hot files with `-O3`; place hot functions in IRAM via
  `IRAM_ATTR` where cache-miss jitter matters. Consider `-ffast-math` **only** for isolated, verified DSP code (not globally).

### 3.2 Where assembly / intrinsics are actually justified (profiling-gated)

We will **measure first** (cycle counts via the `XTHAL_GET_CCOUNT()` register, GPIO-toggle + logic analyser on the loop tick).
Hand-optimisation is authorised only for a hotspot that shows up in the profile. Realistic candidates, most-likely-first:

1. **Packet CRC-16/CRC-32.** Computed on every TX and RX frame. If it lands on the hot path, replace the table-driven C loop
   with the ESP32 ROM CRC routines (`esp_rom_crc16_le`) or a hand-tuned Xtensa loop. Cheap win, easy to verify.
2. **Fixed-point sensor linearisation / oversampling filter.** The ADS1115 averaging + Q16.16 scale-and-offset across 4
   channels every tick. Xtensa has MAC/`mull` instructions and zero-overhead loops (`LOOP`) that a hand-written inner loop can
   exploit if the compiler doesn't. Verify with a C reference in `test/`.
3. **NMEA parse hot path.** If GPS decode dominates (unlikely at 9600 baud), the field-splitting scan is the only part worth
   intrinsic-izing; usually `-O3` on plain C is enough.
4. **ULP coprocessor (not Xtensa ASM, but the same spirit).** For any low-power ground-idle sampling, the ULP can sample a
   channel while the main cores sleep — written in ULP assembly or the `ulp_fsm`/`ulp-riscv` toolchain.

> Assembly is a scalpel, not a hammer. Every ASM/intrinsic block must (a) be triggered by a real profile, (b) sit behind a
> plain-C reference implementation kept in `test/` so we can prove equivalence, and (c) be `#ifdef`-guardable so a build can
> fall back to portable C. We will **not** hand-write assembly for I/O-bound code where the CPU is already waiting on a bus.

### 3.3 What "performant" means for this system (the real targets)
- Acquisition loop jitter **< 1 ms** on the sample tick (measured GPIO toggle).
- Event-trigger → alert-packet-sent latency **< 5 s** (PDF pass criterion), targeting < 1 s in firmware.
- Zero heap growth over a 6-hour bench log (PDF SD-robustness test).
- No missed sensor samples during a full-throttle motor EMI run (PDF EMI test).

---

## 4. Module-by-module implementation (mirrors PDF Phase 1 bring-up order)

Each driver is a self-contained module under `lib/` with its own unit test in `test/`. Bring them up **one at a time**, on the
bench, before integration — exactly the PDF's incremental order.

| # | Module (`lib/…`) | Bus | Key work | Perf notes |
|---|---|---|---|---|
| 4.1 | `power` (verify only) | — | Confirm 5.0 V / 3.3 V rails under load *before* connecting expensive parts (PDF 1.1). No code, a checklist. | — |
| 4.2 | `board` | — | Blink + serial banner; clock at 240 MHz; brownout detector config. | Sets `-O2`, verifies IRAM budget. |
| 4.3 | `ads1115` | I²C | 4-channel single-shot reads, PGA set so full-scale TIA output ≈ 80% of range; oversample + average. | Fixed-point averaging; batched I²C. |
| 4.4 | `gps_neo_m8n` | UART | NMEA RMC/GGA parse → lat/lon/alt/fix. | Zero-copy field scan; no `String`. |
| 4.5 | `co2_mhz19c` | PWM | Duty-cycle capture ISR → ppm. | PCNT/MCPWM capture, `IRAM_ATTR`. |
| 4.6 | `pm_pms5003` | UART/bridge | 32-byte frame parse, checksum, PM1/2.5/10. | Ring-buffer framing, no polling. |
| 4.7 | `sdlog` | SPI | Open FAT32, append fixed-width records, batched flush, rotation. | Batched 512 B writes on core 0. |
| 4.8 | `camera` | GPIO/SPI | Trigger + capture-to-SD on event; store filename in event record. | Off hot path; runs in `event_task`. |
| 4.9 | `lora_rak3172` | UART | AT config (region **868 MHz / Nigeria per NCC**), **P2P mode first** (PDF 2.2), TX/RX, RSSI/SNR. | State machine, non-blocking, retry/backoff. |
| 4.10 | `record` + `codec` | — | The `Record{}` POD, binary pack/unpack, CRC. | The main ASM candidate (§3.2). |

### 4.1 Analog front-end note (from DOCX §3.6)
The four electrochemical cells (H₂, O₃, SO₂, CO) each feed one OPA4192 TIA channel: `Vout = Isensor · Rf`, with `Rf` chosen so
full-scale current maps to ~80% of the ADS1115 input range, followed by a first-order RC low-pass (`fc = 1/2πRC`, 1–5 Hz is
ample for gas signals). Firmware does **not** control this — it only reads the 4 conditioned voltages via ADS1115 — but the
`ads1115` module's scaling constants (`Rf`, PGA, offset) must match the as-built resistor values, so those live in one config
header per unit and are set during the ADC/analog-front-end bench test (PDF test: measured vs. calculated within 2%).

---

## 5. LoRa payload definition (PDF 2.3 — compact binary, < 50 bytes)

A packed, fixed-layout struct. Little-endian, no padding (`__attribute__((packed))`), integer-scaled fields. This is the
contract shared by both nodes (lives in `lib/codec/`), and is the primary CRC/pack hotspot.

| Field | Type | Scale / unit | Bytes |
|---|---|---|---|
| `magic` + `version` | u8 + u8 | protocol id | 2 |
| `seq` | u16 | packet counter | 2 |
| `epoch` | u32 | seconds | 4 |
| `lat`, `lon` | i32 | ×1e7 deg | 8 |
| `alt` | i16 | metres | 2 |
| `gas[4]` (H₂,O₃,SO₂,CO) | u16 ×4 | ppb (scaled) | 8 |
| `co2` | u16 | ppm | 2 |
| `pm1/pm2_5/pm10` | u16 ×3 | µg/m³ | 6 |
| `vbat` | u16 | mV | 2 |
| `flags` | u8 | fix, event, warmup, faults | 1 |
| `crc16` | u16 | over all prior bytes | 2 |
| **Total** | | | **~41 B** |

Two packet types share this layout: **periodic** (every N-th sample) and **event/alert** (sent immediately on threshold
exceedance, `flags.event=1`). Ground node validates `magic`/`version`/`crc16`, then emits one JSON line per packet over USB.

---

## 6. Ground station firmware + laptop dashboard

- **`ground-node` firmware:** LoRa RX → CRC check → decode → print one compact JSON object per line to USB serial. No SD, no
  sensors. Minimal, so we can reuse the same LoRa driver and codec.
- **Laptop dashboard:** a small Python (Streamlit) or Node-RED reader of the USB serial JSON stream (PDF 2.4). Lives in a
  separate `groundstation/` folder or its own repo — **not** ESP32 firmware, so not in this PlatformIO build. Plots gas/PM/CO₂
  vs. time and maps GPS track; raises visible alerts on `flags.event`.

---

## 7. Phased milestones (firmware view, mapped to the source docs)

| Milestone | Maps to | Firmware deliverable | Exit criterion |
|---|---|---|---|
| **M1 Bench bring-up** | PDF 1.2–1.3 | `board`, `ads1115`, `gps`, `co2`, `pm`, `sdlog`, `camera` drivers, each with a unit test | Each sensor read correctly in isolation; ADC within 2% of calculated (PDF test) |
| **M2 Analog FE** | PDF 1.4–1.5, DOCX 3.6 | `ads1115` scaling calibrated to as-built `Rf`; 4 channels stable baseline | Stable zero in clean air; response to incense/candle/exhale tests |
| **M3 Comms link** | PDF 2.1–2.4 | `lora_rak3172` P2P + `codec` + `ground-node` + JSON dashboard | Counter packet RX; range walk >95% delivery at planned distance |
| **M4 Integrated firmware** | PDF 3.1–3.3 | `acq/log/radio/event` tasks merged; watchdog + last-known-good; event trigger + camera | 6-h bench log, zero corruption/heap growth; alert < 5 s |
| **M5 EMI hardening** | PDF 4.5 test | Confirm no missed samples / baseline shift with motors running (props off) | Gas baseline shift < 2× noise floor motors-on vs off |
| **M6 Flight data** | PDF 5.1–5.3 | Operating routine: warm-up handling, pre-flight self-test, post-flight offload | Pass-to-pass agreement within sensor noise |

**Spending/sequence note (PDF cost control):** M1–M4 are completed **fully on the bench** before any airframe money is spent.
This firmware repo delivers M1–M4 independently of the drone build.

---

## 8. Reliability requirements (PDF 3.3)

- **Task watchdog (TWDT)** feeding per task; a hung sensor read cannot stall the logger.
- **Last-known-good:** a faulted sensor writes its last valid value + a fault flag; logging and TX continue.
- **Brownout + reset cause logging:** record reset reason to SD on boot for post-mortem.
- **Warm-up gate:** electrochemical cells need minutes to stabilise; firmware flags `warmup` until elapsed and the dashboard
  greys those readings.

---

## 9. PlatformIO build & flash workflow

Recommended `platformio.ini` (to be applied when firmware work starts — not yet committed):

```ini
[env]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
build_flags = -O2 -Wall -Wextra
build_unflags = -Os

[env:drone]
build_flags = ${env.build_flags} -DNODE_DRONE

[env:ground]
build_flags = ${env.build_flags} -DNODE_GROUND
```

- Build a node: `pio run -e drone` / `pio run -e ground`
- Flash + monitor: `pio run -e drone -t upload -t monitor`
- Unit tests: `pio test -e native` (host-side codec/CRC/fixed-point tests) and on-target where hardware is needed.

Documentation (this file, `docs/`) is untouched by every one of these commands — it cannot enter the firmware image.

---

## 10. Open decisions to confirm before coding

1. **UART topology (§1.1):** confirm MH-Z19C-PWM + SC16IS752 bridge, or fall back to one SoftwareSerial. *Blocks wiring & M1.*
2. **ESP32 variant:** plan assumes classic ESP32-WROOM-32. If the camera is an **ESP32-CAM**, its pin map and PSRAM change §4.8.
3. **Region/frequency:** 868 MHz assumed for Nigeria (NCC) — confirm the exact P2P frequency/SF/BW plan.
4. **SD record format:** binary (compact, fast) vs. CSV (human-readable). Recommend binary on-card + a converter tool.
5. **Deliverable format:** this plan is Markdown in `docs/`. Say the word if you also want it exported to `.docx`/PDF.
