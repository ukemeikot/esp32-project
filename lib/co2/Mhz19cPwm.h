// Mhz19cPwm.h — MH-Z19C CO2 sensor read via its PWM output (Decision D1).
// An IRAM edge-capture ISR times the high pulse; readPpm() applies the datasheet
// formula. This frees a hardware UART and costs ~nothing on the CPU (no polling,
// no bit-banging), which is why PWM mode was chosen over UART for this sensor.
//
//   ppm = range * (Th_ms - 2) / (Th_ms + Tl_ms - 4)   (cycle ~1004 ms)

#ifndef MHZ19C_PWM_H
#define MHZ19C_PWM_H

#include <Arduino.h>

/** @brief MH-Z19C CO2 sensor read through its PWM output pin. */
class Mhz19cPwm {
 public:
  /**
   * @param pin      GPIO wired to the sensor's PWM output (interrupt-capable).
   * @param rangePpm Sensor full-scale range in ppm (feeds the PWM formula).
   */
  Mhz19cPwm(int pin, uint16_t rangePpm) : pin_(pin), range_(rangePpm) {}

  /** @brief Configure the input pin and attach the edge-capture interrupt. */
  void begin();

  /**
   * @brief Latest CO2 concentration from the most recently completed PWM cycle.
   * @param[out] ok Set false if no plausible cycle was seen recently (sensor
   *                absent or still warming up), true otherwise.
   * @return CO2 in ppm (0 if @p ok is false).
   */
  uint16_t readPpm(bool& ok);

 private:
  static void IRAM_ATTR isrThunk();
  void IRAM_ATTR onEdge();

  int      pin_;
  uint16_t range_;

  // Written in ISR, read in task context; guarded by a portMUX critical section.
  volatile uint32_t riseUs_   = 0;
  volatile uint32_t highUs_   = 0;   // last measured high duration
  volatile uint32_t cycleUs_  = 0;   // last full cycle duration
  volatile uint32_t lastEdgeMs_ = 0;

  static Mhz19cPwm* self_;
};

#endif  // MHZ19C_PWM_H
