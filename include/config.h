// config.h — Board pin map, bus addresses, and mission parameters for the
// drone monitoring node. This is the ONLY place hardware-specific values live.
//
// Values marked [CALIBRATE] are placeholders that MUST be set to the as-built
// values before flight (plan §4.1 / §10): feedback resistors, sensor
// sensitivities, the battery divider ratio, and the alert thresholds.

#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

namespace cfg {

// ---- Serial / debug ----------------------------------------------------------
static constexpr uint32_t kDebugBaud = 115200;

// ---- I2C (ADS1115 + SC16IS752 bridge) ----------------------------------------
static constexpr int      kI2cSda = 21;
static constexpr int      kI2cScl = 22;
static constexpr uint32_t kI2cHz  = 400000;         // fast-mode
static constexpr uint8_t  kAds1115Addr = 0x48;      // ADDR -> GND
static constexpr uint8_t  kSc16Addr    = 0x4D;      // A0/A1 strap; verify on board
static constexpr uint32_t kSc16XtalHz  = 1843200;   // bridge crystal; verify on board

// ---- GPS: NEO-M8N on UART1 ----------------------------------------------------
static constexpr int      kGpsRx   = 34;            // ESP32 RX  (GPS TX); input-only pin
static constexpr int      kGpsTx   = 33;            // ESP32 TX  (GPS RX); optional config
static constexpr uint32_t kGpsBaud = 9600;

// ---- LoRa: RAK3172 on UART2 ---------------------------------------------------
static constexpr int      kLoraRx   = 16;           // ESP32 RX (RAK TX)
static constexpr int      kLoraTx   = 17;           // ESP32 TX (RAK RX)
static constexpr uint32_t kLoraBaud = 115200;       // RAK3172 default

// LoRa P2P PHY (Decision D3): 868 MHz Nigeria/NCC, SF9/BW125/CR4-5.
static constexpr uint32_t kLoraFreqHz   = 868000000;
static constexpr uint8_t  kLoraSf       = 9;
static constexpr uint8_t  kLoraBwCode   = 0;        // RAK: 0=125kHz,1=250kHz,2=500kHz
static constexpr uint8_t  kLoraCrCode   = 1;        // RAK: 1=4/5
static constexpr uint8_t  kLoraPreamble = 8;
static constexpr uint8_t  kLoraTxDbm    = 14;

// ---- CO2: MH-Z19C in PWM mode (Decision D1) ----------------------------------
static constexpr int      kCo2PwmPin = 25;          // PWM output of MH-Z19C
static constexpr uint16_t kCo2RangePpm = 5000;      // sensor full-scale (PWM formula)

// ---- SD card (SPI) ------------------------------------------------------------
static constexpr int kSdSck  = 18;
static constexpr int kSdMiso = 19;
static constexpr int kSdMosi = 23;
static constexpr int kSdCs   = 5;

// ---- Camera trigger (ESP32-CAM co-processor, Decision D2) --------------------
static constexpr int      kCamTrigPin  = 4;
static constexpr uint32_t kCamPulseMs  = 50;        // trigger pulse width

// ---- Battery monitor (ESP32 ADC1) --------------------------------------------
static constexpr int   kVbatAdcPin = 35;            // ADC1_CH7, input-only
static constexpr float kVbatDivider = 5.545f;       // [CALIBRATE] (R1+R2)/R2, e.g. 100k/22k
static constexpr float kAdcVref     = 3.30f;        // [CALIBRATE] measure actual 3V3 rail

// ---- Analog front-end: 4 electrochemical gas channels on the ADS1115 ---------
// Channel order MUST match the wiring: 0=H2, 1=O3, 2=SO2, 3=CO.
// gas_ppm = Vout / (Rf * sensitivity_A_per_ppm).  [CALIBRATE] all six values.
static constexpr float kGasRf[4]          = {100000.f, 100000.f, 100000.f, 100000.f};      // ohms
static constexpr float kGasSensAperPpm[4] = {6.0e-8f,  6.0e-8f,  6.0e-8f,  4.0e-9f};       // A/ppm
// ADS1115 PGA per channel (full-scale volts): pick so full-scale current ~= 80% of range.
// 0=6.144V 1=4.096V 2=2.048V 3=1.024V 4=0.512V 5=0.256V
static constexpr uint8_t kGasPga[4] = {1, 1, 1, 1};

// ---- Alert thresholds (WHO-guideline-derived placeholders) [CALIBRATE] -------
static constexpr float    kThreshGasPpm[4] = {10.0f, 0.10f, 0.075f, 9.0f};  // H2,O3,SO2,CO
static constexpr uint16_t kThreshCo2Ppm    = 1500;
static constexpr uint16_t kThreshPm25      = 25;    // ug/m^3 (WHO 24h)

// ---- Mission cadence (Decision D8) -------------------------------------------
static constexpr uint32_t kSamplePeriodMs = 3000;   // acquisition tick
static constexpr uint16_t kTxEveryN       = 1;      // transmit every N-th periodic sample
static constexpr uint32_t kWarmupMs       = 180000; // electrochemical warm-up gate (3 min)

// ---- SD logging ---------------------------------------------------------------
static constexpr uint16_t kSdFlushEveryN = 8;       // batch records before flush (D4)

}  // namespace cfg

#endif  // CONFIG_H
