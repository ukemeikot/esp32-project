// Pms5003.h — Plantower PMS5003 particulate-matter frame parser.
// Transport-agnostic: reads from any Stream (a hardware UART, or the SC16IS752
// I2C-UART bridge port used on this board per Decision D1). 32-byte frames,
// 0x42 0x4D header, big-endian fields, 16-bit checksum. No dynamic allocation.

#ifndef PMS5003_H
#define PMS5003_H

#include <Arduino.h>

/** @brief Plantower PMS5003 frame parser over any Arduino Stream. */
class Pms5003 {
 public:
  /** @param io Serial stream the sensor transmits on (UART or bridge port). */
  explicit Pms5003(Stream& io) : io_(io) {}

  /**
   * @brief Drain buffered bytes, advancing the frame state machine.
   * @return true if a valid frame completed during this call (values updated).
   */
  bool update();

  bool     valid()  const { return valid_; }   ///< true once a valid frame was parsed
  uint16_t pm1()    const { return pm1_; }      ///< PM1.0, ug/m^3 (atmospheric)
  uint16_t pm2_5()  const { return pm25_; }     ///< PM2.5, ug/m^3 (atmospheric)
  uint16_t pm10()   const { return pm10_; }     ///< PM10,  ug/m^3 (atmospheric)

 private:
  Stream& io_;
  uint8_t buf_[32];
  uint8_t idx_ = 0;

  bool     valid_ = false;
  uint16_t pm1_  = 0;
  uint16_t pm25_ = 0;
  uint16_t pm10_ = 0;
};

#endif  // PMS5003_H
