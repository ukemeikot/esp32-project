/**
 * @file Ads1115.h
 * @brief Minimal, dependency-light driver for the TI ADS1115 16-bit ADC.
 *
 * Performs single-shot conversions on the four single-ended channels carrying
 * the OPA4192 transimpedance-amplifier outputs (one per electrochemical gas
 * cell). Uses only the Arduino @c Wire library — no third-party dependency.
 */

#ifndef ADS1115_H
#define ADS1115_H

#include <Arduino.h>
#include <Wire.h>

/** @brief Driver for a single ADS1115 on an I2C bus. */
class Ads1115 {
 public:
  /**
   * @param wire I2C bus the device is attached to.
   * @param addr 7-bit I2C address (0x48..0x4B depending on the ADDR strap).
   */
  Ads1115(TwoWire& wire, uint8_t addr) : wire_(wire), addr_(addr) {}

  /**
   * @brief Probe for the device.
   * @return true if the configuration register is reachable (device present).
   */
  bool begin();

  /**
   * @brief Perform one blocking single-shot conversion.
   * @param channel Single-ended input channel, 0..3.
   * @param pgaCode Programmable-gain code, 0..7 (see @ref toVolts full-scale table).
   * @param[out] ok Set to false on I2C failure, true otherwise.
   * @return Signed raw ADC count (undefined if @p ok is false).
   */
  int16_t readRaw(uint8_t channel, uint8_t pgaCode, bool& ok);

  /**
   * @brief Convert a raw count and PGA code to volts.
   * @param raw     Signed raw ADC count from @ref readRaw.
   * @param pgaCode The PGA code used for that conversion.
   * @return Input voltage in volts.
   */
  static float toVolts(int16_t raw, uint8_t pgaCode);

 private:
  /** @brief Write a 16-bit big-endian value to a device register. */
  bool writeReg(uint8_t reg, uint16_t value);
  /** @brief Read a 16-bit big-endian value from a device register. */
  bool readReg(uint8_t reg, uint16_t& value);

  TwoWire& wire_;  ///< I2C bus
  uint8_t  addr_;  ///< 7-bit device address
};

#endif  // ADS1115_H
