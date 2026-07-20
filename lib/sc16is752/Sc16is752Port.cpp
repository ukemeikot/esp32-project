#include "Sc16is752Port.h"

namespace {
// SC16IS752 register indices (general register set, LCR[7] = 0).
constexpr uint8_t kRHR = 0x00;  // Rx holding (read) / Tx holding (write)
constexpr uint8_t kIER = 0x01;  // interrupt enable
constexpr uint8_t kFCR = 0x02;  // FIFO control (write)
constexpr uint8_t kLCR = 0x03;  // line control
constexpr uint8_t kMCR = 0x04;  // modem control
constexpr uint8_t kLSR = 0x05;  // line status
constexpr uint8_t kTXLVL = 0x08;  // Tx FIFO free space
constexpr uint8_t kRXLVL = 0x09;  // Rx FIFO filled level
// Divisor-latch registers, accessible only when LCR[7] = 1.
constexpr uint8_t kDLL = 0x00;
constexpr uint8_t kDLH = 0x01;
}  // namespace

bool Sc16is752Port::begin(uint32_t baud) {
  // Probe the device by reading a benign register.
  wire_.beginTransmission(addr_);
  wire_.write(subaddr(kLCR));
  if (wire_.endTransmission() != 0) return false;

  // Program the baud-rate divisor: divisor = xtal / (prescaler * 16 * baud),
  // prescaler = 1 (MCR[7] = 0).
  const uint16_t divisor = static_cast<uint16_t>(xtal_ / (16UL * baud));
  writeReg(kLCR, 0x80);                         // enable divisor latch
  writeReg(kDLL, static_cast<uint8_t>(divisor & 0xFF));
  writeReg(kDLH, static_cast<uint8_t>(divisor >> 8));
  writeReg(kLCR, 0x03);                          // 8 data bits, 1 stop, no parity
  writeReg(kFCR, 0x07);                          // enable + reset Rx/Tx FIFOs
  writeReg(kMCR, 0x00);                          // prescaler = 1
  writeReg(kIER, 0x00);                          // polled mode, no interrupts
  return true;
}

void Sc16is752Port::writeReg(uint8_t reg, uint8_t val) {
  wire_.beginTransmission(addr_);
  wire_.write(subaddr(reg));
  wire_.write(val);
  wire_.endTransmission();
}

uint8_t Sc16is752Port::readReg(uint8_t reg) {
  wire_.beginTransmission(addr_);
  wire_.write(subaddr(reg));
  if (wire_.endTransmission(false) != 0) return 0;   // repeated start
  if (wire_.requestFrom(addr_, static_cast<uint8_t>(1)) != 1) return 0;
  return wire_.read();
}

int Sc16is752Port::available() {
  return readReg(kRXLVL) + (peeked_ >= 0 ? 1 : 0);
}

int Sc16is752Port::read() {
  if (peeked_ >= 0) { const int v = peeked_; peeked_ = -1; return v; }
  if (readReg(kRXLVL) == 0) return -1;
  return readReg(kRHR);
}

int Sc16is752Port::peek() {
  if (peeked_ < 0) peeked_ = read();
  return peeked_;
}

size_t Sc16is752Port::write(uint8_t b) {
  // Block briefly until the Tx FIFO has room, then push the byte.
  for (uint8_t tries = 0; tries < 100 && readReg(kTXLVL) == 0; ++tries) {
    /* spin */;
  }
  writeReg(kRHR, b);
  return 1;
}
