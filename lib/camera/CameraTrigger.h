/**
 * @file CameraTrigger.h
 * @brief Fires the ESP32-CAM capture co-processor on an event (Decision D2).
 *
 * The camera is a separate ESP32-CAM so its capture/DMA never steals cycles from
 * the acquisition loop. This node only pulses a GPIO to request a capture and
 * derives a deterministic filename from (epoch, seq); the ESP32-CAM firmware is
 * expected to name the stored image by the same convention, so no serial
 * hand-back link (and no extra UART) is required.
 */

#ifndef CAMERA_TRIGGER_H
#define CAMERA_TRIGGER_H

#include <Arduino.h>

class CameraTrigger {
 public:
  /**
   * @param pin      GPIO wired to the ESP32-CAM trigger input.
   * @param pulseMs  Active-high trigger pulse width in milliseconds.
   */
  CameraTrigger(int pin, uint32_t pulseMs) : pin_(pin), pulseMs_(pulseMs) {}

  /** @brief Configure the trigger GPIO as an output (idle low). */
  void begin();

  /**
   * @brief Pulse the trigger line to request one capture.
   * @note Blocks for @c pulseMs; call from the event task, never the acq loop.
   */
  void trigger();

  /**
   * @brief Build the deterministic image filename for a given record.
   * @param epoch  Record timestamp (unix seconds).
   * @param seq    Record sequence number.
   * @param out    Destination buffer.
   * @param outLen Size of @p out in bytes.
   */
  static void makeFilename(uint32_t epoch, uint16_t seq, char* out, size_t outLen);

 private:
  int      pin_;
  uint32_t pulseMs_;
};

#endif  // CAMERA_TRIGGER_H
