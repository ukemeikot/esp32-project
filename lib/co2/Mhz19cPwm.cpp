#include "Mhz19cPwm.h"

Mhz19cPwm* Mhz19cPwm::self_ = nullptr;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

void Mhz19cPwm::begin() {
  self_ = this;
  pinMode(pin_, INPUT);
  attachInterrupt(digitalPinToInterrupt(pin_), isrThunk, CHANGE);
}

void IRAM_ATTR Mhz19cPwm::isrThunk() {
  if (self_) self_->onEdge();
}

void IRAM_ATTR Mhz19cPwm::onEdge() {
  const uint32_t now = micros();
  portENTER_CRITICAL_ISR(&s_mux);
  if (digitalRead(pin_)) {
    // Rising edge: a full cycle ended at the previous rise.
    if (riseUs_) cycleUs_ = now - riseUs_;
    riseUs_ = now;
  } else {
    // Falling edge: high duration is now - rise.
    if (riseUs_) highUs_ = now - riseUs_;
  }
  portEXIT_CRITICAL_ISR(&s_mux);
  lastEdgeMs_ = millis();  // 32-bit aligned write; fine without the lock
}

uint16_t Mhz19cPwm::readPpm(bool& ok) {
  uint32_t highUs, cycleUs;
  portENTER_CRITICAL(&s_mux);
  highUs = highUs_;
  cycleUs = cycleUs_;
  portEXIT_CRITICAL(&s_mux);

  // Require a plausible ~1004 ms cycle seen within the last ~3 s.
  if (cycleUs < 500000 || cycleUs > 1500000 ||
      (millis() - lastEdgeMs_) > 3000) {
    ok = false;
    return 0;
  }

  const float th = highUs / 1000.0f;              // ms
  const float tl = (cycleUs - highUs) / 1000.0f;  // ms
  const float denom = th + tl - 4.0f;
  if (denom <= 0.0f) { ok = false; return 0; }

  float ppm = range_ * (th - 2.0f) / denom;
  if (ppm < 0.0f) ppm = 0.0f;
  if (ppm > 65535.0f) ppm = 65535.0f;
  ok = true;
  return static_cast<uint16_t>(ppm + 0.5f);
}
