#include "Ads1115.h"

namespace {
constexpr uint8_t  kRegConv   = 0x00;
constexpr uint8_t  kRegConfig = 0x01;

// Config register bit fields (ADS1115 datasheet, Table 8).
constexpr uint16_t kOsSingle  = 0x8000;  // start single conversion
constexpr uint16_t kModeSingle = 0x0100; // single-shot mode
constexpr uint16_t kDr128Sps  = 0x0080;  // 128 SPS
constexpr uint16_t kCompDisable = 0x0003;

// Full-scale volts per PGA code, for count->volt conversion.
constexpr float kFsVolts[8] = {6.144f, 4.096f, 2.048f, 1.024f,
                               0.512f, 0.256f, 0.256f, 0.256f};
}  // namespace

bool Ads1115::begin() {
  uint16_t v;
  return readReg(kRegConfig, v);
}

bool Ads1115::writeReg(uint8_t reg, uint16_t value) {
  wire_.beginTransmission(addr_);
  wire_.write(reg);
  wire_.write(static_cast<uint8_t>(value >> 8));
  wire_.write(static_cast<uint8_t>(value & 0xFF));
  return wire_.endTransmission() == 0;
}

bool Ads1115::readReg(uint8_t reg, uint16_t& value) {
  wire_.beginTransmission(addr_);
  wire_.write(reg);
  if (wire_.endTransmission(false) != 0) return false;   // repeated start
  if (wire_.requestFrom(addr_, static_cast<uint8_t>(2)) != 2) return false;
  const uint8_t hi = wire_.read();
  const uint8_t lo = wire_.read();
  value = (static_cast<uint16_t>(hi) << 8) | lo;
  return true;
}

int16_t Ads1115::readRaw(uint8_t channel, uint8_t pgaCode, bool& ok) {
  const uint16_t mux = 0x4000 | (static_cast<uint16_t>(channel & 0x03) << 12);
  const uint16_t pga = static_cast<uint16_t>(pgaCode & 0x07) << 9;
  const uint16_t config = kOsSingle | mux | pga | kModeSingle | kDr128Sps | kCompDisable;

  if (!writeReg(kRegConfig, config)) { ok = false; return 0; }

  // Wait for OS bit to go high (conversion complete). 128 SPS => ~7.8 ms; poll
  // with a hard timeout so a bus fault can never wedge the acquisition task.
  const uint32_t t0 = millis();
  uint16_t status = 0;
  do {
    if (!readReg(kRegConfig, status)) { ok = false; return 0; }
    if (status & kOsSingle) break;
  } while (millis() - t0 < 12);

  uint16_t raw;
  if (!readReg(kRegConv, raw)) { ok = false; return 0; }
  ok = true;
  return static_cast<int16_t>(raw);
}

float Ads1115::toVolts(int16_t raw, uint8_t pgaCode) {
  return (static_cast<float>(raw) * kFsVolts[pgaCode & 0x07]) / 32768.0f;
}
