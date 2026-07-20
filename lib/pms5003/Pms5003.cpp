#include "Pms5003.h"

namespace {
inline uint16_t be16(const uint8_t* p) {
  return (static_cast<uint16_t>(p[0]) << 8) | p[1];
}
}  // namespace

bool Pms5003::update() {
  bool completed = false;
  while (io_.available()) {
    const int c = io_.read();
    if (c < 0) break;
    const uint8_t b = static_cast<uint8_t>(c);

    // Resynchronise on the two-byte header.
    if (idx_ == 0) { if (b == 0x42) buf_[idx_++] = b; continue; }
    if (idx_ == 1) { if (b == 0x4D) buf_[idx_++] = b; else idx_ = 0; continue; }

    buf_[idx_++] = b;
    if (idx_ < 32) continue;

    // Full frame captured: verify checksum (sum of bytes 0..29).
    idx_ = 0;
    uint16_t sum = 0;
    for (int i = 0; i < 30; ++i) sum += buf_[i];
    if (sum != be16(&buf_[30])) continue;   // corrupt frame, drop

    // Atmospheric-environment concentrations: data4..6 at offsets 10,12,14.
    pm1_  = be16(&buf_[10]);
    pm25_ = be16(&buf_[12]);
    pm10_ = be16(&buf_[14]);
    valid_ = true;
    completed = true;
  }
  return completed;
}
