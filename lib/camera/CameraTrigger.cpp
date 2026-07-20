#include "CameraTrigger.h"

void CameraTrigger::begin() {
  pinMode(pin_, OUTPUT);
  digitalWrite(pin_, LOW);
}

void CameraTrigger::trigger() {
  digitalWrite(pin_, HIGH);
  delay(pulseMs_);
  digitalWrite(pin_, LOW);
}

void CameraTrigger::makeFilename(uint32_t epoch, uint16_t seq, char* out, size_t outLen) {
  // e.g. IMG_0001743512_00042.JPG — reconstructable on the ESP32-CAM side.
  snprintf(out, outLen, "IMG_%010lu_%05u.JPG",
           static_cast<unsigned long>(epoch), static_cast<unsigned>(seq));
}
