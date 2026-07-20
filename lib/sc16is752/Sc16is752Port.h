// Sc16is752Port.h — One UART channel of an SC16IS752 I2C<->dual-UART bridge,
// exposed as an Arduino Stream. Used to give the PMS5003 a hardware-buffered
// serial port without spending one of the ESP32's three hardware UARTs
// (Decision D1). NOTE: register logic is per the SC16IS752 datasheet but has not
// yet been validated against physical hardware — verify on first bring-up.

#ifndef SC16IS752_PORT_H
#define SC16IS752_PORT_H

#include <Arduino.h>
#include <Wire.h>

/** @brief One UART channel of an SC16IS752 I2C-UART bridge, as an Arduino Stream. */
class Sc16is752Port : public Stream {
 public:
  /**
   * @param wire    I2C bus the bridge is on.
   * @param addr    7-bit I2C address (set by the A0/A1 straps).
   * @param channel Bridge UART channel: 0 = UART A, 1 = UART B.
   * @param xtalHz  Bridge crystal frequency, used to compute the baud divisor.
   */
  Sc16is752Port(TwoWire& wire, uint8_t addr, uint8_t channel, uint32_t xtalHz)
      : wire_(wire), addr_(addr), channel_(channel), xtal_(xtalHz) {}

  /**
   * @brief Configure the channel for 8N1 at the given baud rate.
   * @param baud Desired baud rate.
   * @return false on I2C failure, true otherwise.
   */
  bool begin(uint32_t baud);

  int available() override;      ///< Bytes available (bridge Rx FIFO + peek cache).
  int read() override;           ///< Read one byte, or -1 if none available.
  int peek() override;           ///< Peek the next byte without consuming it, or -1.
  size_t write(uint8_t b) override;  ///< Transmit one byte (blocks briefly for FIFO space).
  using Print::write;

 private:
  void    writeReg(uint8_t reg, uint8_t val);
  uint8_t readReg(uint8_t reg);
  uint8_t subaddr(uint8_t reg) const {
    return static_cast<uint8_t>((reg << 3) | (channel_ << 1));
  }

  TwoWire& wire_;
  uint8_t  addr_;
  uint8_t  channel_;
  uint32_t xtal_;
  int      peeked_ = -1;
};

#endif  // SC16IS752_PORT_H
