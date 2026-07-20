/**
 * @file GpsTap.h
 * @brief Flight-controller-agnostic GPS reader that passively taps the drone's
 *        existing GPS module (Decision D10).
 *
 * Instead of a second, dedicated GPS, the monitoring node listens on the same
 * serial line the flight controller's GPS transmits on: the GPS TX pin is wired
 * to BOTH the flight controller RX and this ESP32's RX, while the ESP32's TX is
 * left unconnected so it can never contend with the flight controller. Because
 * the ESP32 only listens to the GPS (and never talks to the flight controller),
 * this works with any low-cost FC — Betaflight, INAV, ArduPilot, etc.
 *
 * Different flight controllers leave the GPS in different modes, so this reader
 * decodes BOTH protocols and auto-detects the baud rate:
 *   - NMEA 0183 : GGA (fix quality, altitude) and RMC (position, date/time)
 *   - u-blox UBX: NAV-PVT (position, altitude, fix, sats, UTC in one frame)
 *
 * No dynamic allocation; fixed buffers only.
 */

#ifndef GPS_TAP_H
#define GPS_TAP_H

#include <Arduino.h>

class GpsTap {
 public:
  /**
   * @param uart   Hardware serial port wired (RX only) to the GPS TX tap.
   * @param rxPin  ESP32 RX GPIO connected to the GPS TX line.
   */
  GpsTap(HardwareSerial& uart, int rxPin) : uart_(uart), rxPin_(rxPin) {}

  /** @brief Open the port at the first candidate baud and start auto-detection. */
  void begin();

  /**
   * @brief Drain buffered bytes, decode NMEA/UBX, and run baud auto-detection.
   *
   * Call frequently (from the sensor-IO task) so the UART RX buffer never
   * overflows. If no valid data is seen for a few seconds the reader rotates to
   * the next candidate baud until it locks onto the GPS's actual rate.
   */
  void update();

  bool     hasFix()   const { return fix_; }        ///< true when a valid fix is present
  int32_t  lat1e7()   const { return lat_; }        ///< latitude, degrees * 1e7
  int32_t  lon1e7()   const { return lon_; }        ///< longitude, degrees * 1e7
  int16_t  altM()     const { return alt_; }        ///< altitude above MSL, metres
  uint8_t  sats()     const { return sats_; }       ///< satellites used
  uint32_t epoch()    const { return epoch_; }      ///< UTC unix seconds (0 until known)
  uint32_t baud()     const { return kBauds[baudIdx_]; }  ///< currently selected baud
  bool     locked()   const { return locked_; }     ///< true once valid data was decoded

 private:
  void feed(uint8_t b);
  void handleNmeaLine();
  void handleUbxFrame();
  void markValid();

  static constexpr uint32_t kBauds[] = {9600, 38400, 57600, 115200, 230400};
  static constexpr uint8_t  kNumBauds = 5;

  HardwareSerial& uart_;
  int             rxPin_;

  // Parser state machine.
  enum class Mode : uint8_t { Idle, Nmea, UbxWait62, UbxHdr, UbxPayload, UbxCkA, UbxCkB };
  Mode    mode_ = Mode::Idle;
  char    nmea_[100];
  uint8_t nmeaLen_ = 0;
  uint8_t ubxHdr_[4];        // class, id, lenL, lenH
  uint8_t ubxIdx_ = 0;
  uint8_t ubxPayload_[100];
  uint16_t ubxLen_ = 0;
  uint16_t ubxPos_ = 0;
  uint8_t ubxCkA_ = 0;

  // Auto-baud bookkeeping.
  uint8_t  baudIdx_ = 0;
  bool     locked_  = false;
  uint32_t lastValidMs_ = 0;
  uint32_t lastSwitchMs_ = 0;

  // Latest fix (written here, read by the acquisition task).
  volatile bool     fix_   = false;
  volatile int32_t  lat_   = 0;
  volatile int32_t  lon_   = 0;
  volatile int16_t  alt_   = 0;
  volatile uint8_t  sats_  = 0;
  volatile uint32_t epoch_ = 0;
};

#endif  // GPS_TAP_H
