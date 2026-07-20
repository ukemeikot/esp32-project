/**
 * @file main.cpp
 * @brief Drone air-quality monitoring node — firmware entry point and task wiring.
 *
 * Concurrency model (plan §2.1): sensor acquisition is isolated on core 1 and kept
 * deterministic; the bursty, blocking I/O (SD, LoRa) lives on core 0. Tasks hand
 * fixed-size packed packets to each other through statically-sized queues, so the
 * steady state performs no heap allocation.
 *
 *   sensor_io  (core 1) : continuously drains GPS + PMS5003 into latest-value snapshot
 *   acq        (core 1) : periodic tick — reads ADC/CO2/battery, builds+dispatches a packet
 *   log        (core 0) : drains the log queue to the SD card in batches
 *   radio      (core 0) : drains the radio queue to the RAK3172 (LoRa P2P)
 *   event      (core 0) : on threshold exceedance, pulses the camera trigger
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <esp_task_wdt.h>

#include "config.h"
#include "aqm_protocol.h"
#include "Ads1115.h"
#include "GpsTap.h"
#include "Mhz19cPwm.h"
#include "Pms5003.h"
#include "Sc16is752Port.h"
#include "SdLog.h"
#include "CameraTrigger.h"
#include "Rak3172.h"

// ---- Peripheral objects -------------------------------------------------------
static Ads1115       g_adc(Wire, cfg::kAds1115Addr);
static Sc16is752Port g_pmsPort(Wire, cfg::kSc16Addr, /*channel=*/0, cfg::kSc16XtalHz);
static Pms5003       g_pms(g_pmsPort);
static GpsTap        g_gps(Serial1, cfg::kGpsRx);
static Mhz19cPwm     g_co2(cfg::kCo2PwmPin, cfg::kCo2RangePpm);
static SdLog         g_sd(cfg::kSdCs, cfg::kSdFlushEveryN, aqm::kPacketSize);
static CameraTrigger g_cam(cfg::kCamTrigPin, cfg::kCamPulseMs);
static Rak3172       g_lora(Serial2);

// ---- Cross-task state ---------------------------------------------------------
/** Latest values produced by the sensor-IO task, consumed by the acq task. */
struct Snapshot {
  bool     gpsFix = false, gpsLocked = false, pmValid = false;
  int32_t  lat = 0, lon = 0;
  int16_t  alt = 0;
  uint8_t  sats = 0;
  uint32_t epoch = 0;
  uint16_t pm1 = 0, pm25 = 0, pm10 = 0;
};
static Snapshot          g_snap;
static SemaphoreHandle_t g_snapMutex;   ///< guards g_snap
static SemaphoreHandle_t g_i2cMutex;    ///< serialises the shared I2C bus (ADC + PMS bridge)

/** One packed 41-byte wire packet, the unit passed through the queues. */
struct PacketBytes { uint8_t b[aqm::kPacketSize]; };
static QueueHandle_t g_logQueue;    ///< PacketBytes -> log task
static QueueHandle_t g_radioQueue;  ///< PacketBytes -> radio task
static TaskHandle_t  g_eventTask;   ///< notified to fire the camera

static uint32_t g_warmupEndMs = 0;

// ---- Helpers ------------------------------------------------------------------

/** Read the battery rail through the ESP32 ADC and the resistor divider. */
static uint16_t readBatteryMv() {
  const int raw = analogRead(cfg::kVbatAdcPin);        // 0..4095
  const float v = (raw / 4095.0f) * cfg::kAdcVref * cfg::kVbatDivider;
  return static_cast<uint16_t>(v * 1000.0f + 0.5f);
}

/** Clamp a floating gas concentration (ppm) into its scaled u16 wire value. */
static uint16_t scaleGas(float ppm, uint8_t channel) {
  if (ppm < 0.0f) ppm = 0.0f;
  float scaled = ppm * aqm::kGasTxMul[channel];
  if (scaled > 65535.0f) scaled = 65535.0f;
  return static_cast<uint16_t>(scaled + 0.5f);
}

// ---- Tasks --------------------------------------------------------------------

/** Core 1: keep the GPS and PMS parsers fed so their UART/FIFO never overflow. */
static void sensorIoTask(void*) {
  esp_task_wdt_add(nullptr);
  for (;;) {
    g_gps.update();                               // Serial1 (UART) — no I2C lock

    if (xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      g_pms.update();                             // PMS via the I2C-UART bridge
      xSemaphoreGive(g_i2cMutex);
    }

    if (xSemaphoreTake(g_snapMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      g_snap.gpsFix    = g_gps.hasFix();
      g_snap.gpsLocked = g_gps.locked();
      g_snap.lat = g_gps.lat1e7();
      g_snap.lon = g_gps.lon1e7();
      g_snap.alt = g_gps.altM();
      g_snap.sats = g_gps.sats();
      g_snap.epoch = g_gps.epoch();
      g_snap.pmValid = g_pms.valid();
      g_snap.pm1 = g_pms.pm1();
      g_snap.pm25 = g_pms.pm2_5();
      g_snap.pm10 = g_pms.pm10();
      xSemaphoreGive(g_snapMutex);
    }

    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

/** Core 1: the deterministic sample tick — build and dispatch one packet. */
static void acqTask(void*) {
  esp_task_wdt_add(nullptr);
  TickType_t next = xTaskGetTickCount();
  uint16_t seq = 0;
  uint16_t periodic = 0;

  for (;;) {
    aqm::Packet p;
    memset(&p, 0, sizeof(p));
    p.seq = seq++;
    uint8_t flags = 0;
    bool event = false;
    const bool warmup = millis() < g_warmupEndMs;
    if (warmup) flags |= aqm::FLAG_WARMUP;

    // --- Gas channels (ADS1115 over the shared I2C bus) ---
    if (xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      for (uint8_t ch = 0; ch < 4; ++ch) {
        bool ok = false;
        const int16_t raw = g_adc.readRaw(ch, cfg::kGasPga[ch], ok);
        if (!ok) { flags |= aqm::FLAG_FAULT_ADC; continue; }
        const float volts = Ads1115::toVolts(raw, cfg::kGasPga[ch]);
        const float ppm = volts / (cfg::kGasRf[ch] * cfg::kGasSensAperPpm[ch]);
        p.gas[ch] = scaleGas(ppm, ch);
        if (!warmup && ppm > cfg::kThreshGasPpm[ch]) event = true;
      }
      xSemaphoreGive(g_i2cMutex);
    } else {
      flags |= aqm::FLAG_FAULT_ADC;
    }

    // --- CO2 (PWM capture ISR) ---
    {
      bool ok = false;
      const uint16_t co2 = g_co2.readPpm(ok);
      if (!ok) flags |= aqm::FLAG_FAULT_CO2;
      else {
        p.co2 = co2;
        if (!warmup && co2 > cfg::kThreshCo2Ppm) event = true;
      }
    }

    // --- Snapshot of GPS + PM ---
    Snapshot s;
    if (xSemaphoreTake(g_snapMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      s = g_snap;
      xSemaphoreGive(g_snapMutex);
    }
    if (s.gpsFix) flags |= aqm::FLAG_GPS_FIX;
    if (!s.gpsLocked) flags |= aqm::FLAG_FAULT_GPS;
    p.lat = s.lat; p.lon = s.lon; p.alt = s.alt;
    p.epoch = s.epoch ? s.epoch : (millis() / 1000);
    if (!s.pmValid) flags |= aqm::FLAG_FAULT_PM;
    p.pm1 = s.pm1; p.pm2_5 = s.pm25; p.pm10 = s.pm10;
    if (!warmup && s.pm25 > cfg::kThreshPm25) event = true;

    // --- Battery + SD health flag ---
    p.vbat = readBatteryMv();
    if (!g_sd.healthy()) flags |= aqm::FLAG_FAULT_SD;
    if (event) flags |= aqm::FLAG_EVENT;
    p.flags = flags;

    // --- Serialise and dispatch ---
    PacketBytes pkt;
    aqm::finalize(p, pkt.b);

    xQueueSend(g_logQueue, &pkt, 0);                    // always log
    if (event || (periodic % cfg::kTxEveryN) == 0)      // transmit periodic + all events
      xQueueSend(g_radioQueue, &pkt, 0);
    if (event)                                          // fire the camera co-processor
      xTaskNotifyGive(g_eventTask);
    ++periodic;

    esp_task_wdt_reset();
    vTaskDelayUntil(&next, pdMS_TO_TICKS(cfg::kSamplePeriodMs));
  }
}

/** Core 0: batch packets to the SD card. */
static void logTask(void*) {
  esp_task_wdt_add(nullptr);
  PacketBytes pkt;
  for (;;) {
    if (xQueueReceive(g_logQueue, &pkt, pdMS_TO_TICKS(1000)) == pdTRUE)
      g_sd.append(pkt.b, aqm::kPacketSize);
    else
      g_sd.flush();                 // idle -> make sure nothing lingers unwritten
    esp_task_wdt_reset();
  }
}

/** Core 0: transmit packets over LoRa P2P. */
static void radioTask(void*) {
  esp_task_wdt_add(nullptr);
  PacketBytes pkt;
  for (;;) {
    if (xQueueReceive(g_radioQueue, &pkt, pdMS_TO_TICKS(1000)) == pdTRUE)
      g_lora.send(pkt.b, aqm::kPacketSize);
    esp_task_wdt_reset();
  }
}

/** Core 0: fire the camera on an event notification. */
static void eventTask(void*) {
  for (;;) {
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) > 0)
      g_cam.trigger();
  }
}

// ---- Setup / loop -------------------------------------------------------------

/** Configure the Task Watchdog for a generous timeout across framework versions. */
static void watchdogInit() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  const esp_task_wdt_config_t cfg = {.timeout_ms = 30000, .idle_core_mask = 0, .trigger_panic = true};
  esp_task_wdt_reconfigure(&cfg);
#else
  esp_task_wdt_init(30, true);
#endif
}

void setup() {
  Serial.begin(cfg::kDebugBaud);
  delay(200);
  Serial.printf("\n[AQM] drone node boot; reset reason=%d\n", esp_reset_reason());

  g_warmupEndMs = millis() + cfg::kWarmupMs;
  watchdogInit();

  // Buses.
  Wire.begin(cfg::kI2cSda, cfg::kI2cScl, cfg::kI2cHz);
  SPI.begin(cfg::kSdSck, cfg::kSdMiso, cfg::kSdMosi, cfg::kSdCs);
  analogReadResolution(12);

  // Peripherals (report but do not abort on missing hardware — last-known-good).
  Serial.printf("[AQM] ADS1115  : %s\n", g_adc.begin() ? "ok" : "FAULT");
  Serial.printf("[AQM] PMS port : %s\n", g_pmsPort.begin(9600) ? "ok" : "FAULT");
  g_gps.begin();
  g_co2.begin();
  Serial.printf("[AQM] SD card  : %s\n", g_sd.begin(SPI) ? "ok" : "FAULT");
  g_cam.begin();
  Serial.printf("[AQM] RAK3172  : %s\n", g_lora.begin(cfg::kLoraBaud, cfg::kLoraRx, cfg::kLoraTx) ? "ok" : "FAULT");
  Serial.printf("[AQM] LoRa P2P : %s\n",
                g_lora.configureP2P(cfg::kLoraFreqHz, cfg::kLoraSf, cfg::kLoraBwCode,
                                    cfg::kLoraCrCode, cfg::kLoraPreamble, cfg::kLoraTxDbm)
                    ? "ok" : "FAULT");

  // Sync primitives + queues.
  g_snapMutex  = xSemaphoreCreateMutex();
  g_i2cMutex   = xSemaphoreCreateMutex();
  g_logQueue   = xQueueCreate(32, sizeof(PacketBytes));
  g_radioQueue = xQueueCreate(8, sizeof(PacketBytes));

  // Tasks: acquisition on core 1 (APP_CPU), I/O on core 0 (PRO_CPU).
  xTaskCreatePinnedToCore(eventTask,    "event",   3072, nullptr, 3, &g_eventTask, 0);
  xTaskCreatePinnedToCore(sensorIoTask, "sensorio",4096, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(acqTask,      "acq",     6144, nullptr, 4, nullptr, 1);
  xTaskCreatePinnedToCore(logTask,      "log",     4096, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(radioTask,    "radio",   4096, nullptr, 2, nullptr, 0);

  Serial.println("[AQM] tasks started");
}

void loop() {
  // All work runs in the pinned FreeRTOS tasks; nothing to do here.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
