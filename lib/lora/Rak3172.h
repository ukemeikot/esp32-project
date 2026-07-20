/**
 * @file Rak3172.h
 * @brief Driver for the RAKwireless RAK3172 LoRa module in P2P mode (Decision D3).
 *
 * Communicates over a hardware UART using the RAK AT command set. Configures the
 * P2P PHY (frequency, spreading factor, bandwidth, coding rate, preamble, power)
 * and transmits raw binary payloads (hex-encoded for the AT interface). Intended
 * to run on its own FreeRTOS task, so the blocking AT round-trips never stall the
 * acquisition loop.
 */

#ifndef RAK3172_H
#define RAK3172_H

#include <Arduino.h>

class Rak3172 {
 public:
  /** @param uart Hardware serial port wired to the RAK3172. */
  explicit Rak3172(HardwareSerial& uart) : uart_(uart) {}

  /**
   * @brief Open the UART and confirm the module responds to "AT".
   * @param baud  UART baud rate (RAK3172 default 115200).
   * @param rxPin ESP32 RX GPIO (RAK TX).
   * @param txPin ESP32 TX GPIO (RAK RX).
   * @return true if the module acknowledged.
   */
  bool begin(uint32_t baud, int rxPin, int txPin);

  /**
   * @brief Switch to P2P mode and program the radio PHY.
   * @param freqHz   Centre frequency in Hz (e.g. 868000000).
   * @param sf       Spreading factor (7..12).
   * @param bwCode   Bandwidth code (0=125kHz, 1=250kHz, 2=500kHz).
   * @param crCode   Coding-rate code (1=4/5 ...).
   * @param preamble Preamble length in symbols.
   * @param txDbm    Transmit power in dBm.
   * @return true if every configuration command was accepted.
   */
  bool configureP2P(uint32_t freqHz, uint8_t sf, uint8_t bwCode, uint8_t crCode,
                    uint8_t preamble, uint8_t txDbm);

  /**
   * @brief Transmit a binary payload as a single P2P frame.
   * @param data Pointer to the payload bytes.
   * @param len  Payload length (<= 128 bytes for a sane airtime).
   * @return true if the module accepted the send command.
   */
  bool send(const uint8_t* data, uint16_t len);

 private:
  /**
   * @brief Send one AT command and wait for an "OK"/"ERROR" terminated reply.
   * @param cmd       Command text without trailing CRLF.
   * @param timeoutMs Maximum time to wait for a terminating response.
   * @return true if the module replied "OK".
   */
  bool sendAt(const char* cmd, uint32_t timeoutMs = 500);

  HardwareSerial& uart_;
};

#endif  // RAK3172_H
