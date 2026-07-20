#include "GpsTap.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

constexpr uint32_t GpsTap::kBauds[];

namespace {

// Convert an NMEA "ddmm.mmmm" coordinate field + hemisphere to deg * 1e7.
int32_t nmeaToDeg1e7(const char* field, char hemi) {
  if (!field || !*field) return 0;
  const double v = atof(field);
  const int deg = static_cast<int>(v / 100.0);
  const double minutes = v - deg * 100.0;
  double dec = deg + minutes / 60.0;
  if (hemi == 'S' || hemi == 'W') dec = -dec;
  return static_cast<int32_t>(llround(dec * 1e7));
}

// Days from the civil date to unix epoch seconds (Howard Hinnant's algorithm).
uint32_t civilToEpoch(int y, int m, int d, int hh, int mm, int ss) {
  y -= (m <= 2);
  const int era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = static_cast<unsigned>(y - era * 400);
  const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  const long days = static_cast<long>(era) * 146097 + static_cast<long>(doe) - 719468;
  return static_cast<uint32_t>(days * 86400L + hh * 3600 + mm * 60 + ss);
}

// Verify an NMEA line's "*HH" trailing checksum. s excludes the leading '$'.
bool nmeaChecksumOk(const char* s, uint8_t len) {
  int star = -1;
  for (int i = 0; i < len; ++i)
    if (s[i] == '*') { star = i; break; }
  if (star < 0 || star + 2 >= len) return false;
  uint8_t cs = 0;
  for (int i = 0; i < star; ++i) cs ^= static_cast<uint8_t>(s[i]);
  auto hex = [](uint8_t c) -> int {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
  };
  const int h = hex(s[star + 1]), l = hex(s[star + 2]);
  return h >= 0 && l >= 0 && ((h << 4) | l) == cs;
}

inline int32_t le32(const uint8_t* p) {
  return static_cast<int32_t>(static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
                              (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24));
}
inline uint16_t le16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0] | (p[1] << 8));
}
}  // namespace

void GpsTap::begin() {
  baudIdx_ = 0;
  uart_.begin(kBauds[baudIdx_], SERIAL_8N1, rxPin_, -1);  // RX only (passive tap)
  lastValidMs_ = millis();
  lastSwitchMs_ = millis();
}

void GpsTap::markValid() {
  locked_ = true;
  lastValidMs_ = millis();
}

void GpsTap::update() {
  while (uart_.available()) {
    const int c = uart_.read();
    if (c < 0) break;
    feed(static_cast<uint8_t>(c));
  }

  // Auto-baud: if we have never locked and see nothing valid for a while, try
  // the next candidate rate. Once locked we stay put.
  const uint32_t now = millis();
  if (!locked_ && (now - lastValidMs_) > 3000 && (now - lastSwitchMs_) > 3000) {
    baudIdx_ = (baudIdx_ + 1) % kNumBauds;
    uart_.begin(kBauds[baudIdx_], SERIAL_8N1, rxPin_, -1);
    mode_ = Mode::Idle;
    lastSwitchMs_ = now;
    lastValidMs_ = now;
  }
}

void GpsTap::feed(uint8_t b) {
  switch (mode_) {
    case Mode::Idle:
      if (b == 0xB5) { mode_ = Mode::UbxWait62; }
      else if (b == '$') { mode_ = Mode::Nmea; nmeaLen_ = 0; }
      break;

    case Mode::Nmea:
      if (b == '\r' || b == '\n') {
        if (nmeaLen_ > 0) { nmea_[nmeaLen_] = '\0'; handleNmeaLine(); }
        mode_ = Mode::Idle;
      } else if (nmeaLen_ < sizeof(nmea_) - 1) {
        nmea_[nmeaLen_++] = static_cast<char>(b);
      } else {
        mode_ = Mode::Idle;  // overrun
      }
      break;

    case Mode::UbxWait62:
      mode_ = (b == 0x62) ? Mode::UbxHdr : Mode::Idle;
      ubxIdx_ = 0;
      break;

    case Mode::UbxHdr:
      ubxHdr_[ubxIdx_++] = b;
      if (ubxIdx_ == 4) {
        ubxLen_ = le16(&ubxHdr_[2]);
        ubxPos_ = 0;
        if (ubxLen_ > sizeof(ubxPayload_)) { mode_ = Mode::Idle; }   // unsupported msg
        else { mode_ = (ubxLen_ == 0) ? Mode::UbxCkA : Mode::UbxPayload; }
      }
      break;

    case Mode::UbxPayload:
      ubxPayload_[ubxPos_++] = b;
      if (ubxPos_ >= ubxLen_) mode_ = Mode::UbxCkA;
      break;

    case Mode::UbxCkA:
      ubxCkA_ = b;
      mode_ = Mode::UbxCkB;
      break;

    case Mode::UbxCkB: {
      // Fletcher-8 checksum over class, id, lenL, lenH, payload.
      uint8_t a = 0, ck = 0;
      for (int i = 0; i < 4; ++i) { a += ubxHdr_[i]; ck += a; }
      for (uint16_t i = 0; i < ubxLen_; ++i) { a += ubxPayload_[i]; ck += a; }
      if (a == ubxCkA_ && ck == b) handleUbxFrame();
      mode_ = Mode::Idle;
      break;
    }
  }
}

void GpsTap::handleNmeaLine() {
  if (!nmeaChecksumOk(nmea_, nmeaLen_)) return;

  char* f[24];
  int n = 0;
  f[n++] = nmea_;
  for (uint8_t i = 0; i < nmeaLen_ && n < 24; ++i) {
    if (nmea_[i] == ',') { nmea_[i] = '\0'; f[n++] = &nmea_[i + 1]; }
    else if (nmea_[i] == '*') { nmea_[i] = '\0'; break; }
  }
  if (n < 1 || strlen(f[0]) < 5) return;
  const char* type = f[0] + 2;   // skip talker id (GP/GN/GA...)

  if (memcmp(type, "GGA", 3) == 0 && n >= 10) {
    // 6=fix quality, 7=sats, 9=alt
    fix_  = atoi(f[6]) > 0;
    sats_ = static_cast<uint8_t>(atoi(f[7]));
    if (fix_) {
      lat_ = nmeaToDeg1e7(f[2], f[3][0]);
      lon_ = nmeaToDeg1e7(f[4], f[5][0]);
      alt_ = static_cast<int16_t>(atof(f[9]));
    }
    markValid();
  } else if (memcmp(type, "RMC", 3) == 0 && n >= 10) {
    // 1=time(hhmmss) 2=status 3=lat 4=NS 5=lon 6=EW 9=date(ddmmyy)
    fix_ = (f[2][0] == 'A');
    if (fix_) {
      lat_ = nmeaToDeg1e7(f[3], f[4][0]);
      lon_ = nmeaToDeg1e7(f[5], f[6][0]);
      const char* d = f[9];
      const char* t = f[1];
      if (strlen(d) >= 6 && strlen(t) >= 6) {
        auto d2 = [](const char* s) { return (s[0] - '0') * 10 + (s[1] - '0'); };
        epoch_ = civilToEpoch(2000 + d2(d + 4), d2(d + 2), d2(d),
                              d2(t), d2(t + 2), d2(t + 4));
      }
    }
    markValid();
  }
}

void GpsTap::handleUbxFrame() {
  // Only NAV-PVT (class 0x01, id 0x07) carries everything we need.
  if (ubxHdr_[0] != 0x01 || ubxHdr_[1] != 0x07 || ubxLen_ < 92) { markValid(); return; }

  const uint8_t* p = ubxPayload_;
  const uint16_t year = le16(&p[4]);
  const uint8_t  mon = p[6], day = p[7], hh = p[8], mm = p[9], ss = p[10];
  const uint8_t  fixType = p[20];
  const uint8_t  flags = p[21];
  const uint8_t  numSV = p[23];
  const int32_t  lon = le32(&p[24]);
  const int32_t  lat = le32(&p[28]);
  const int32_t  hMSL_mm = le32(&p[36]);

  fix_  = (fixType >= 2) && (flags & 0x01);   // 2D/3D and gnssFixOK
  sats_ = numSV;
  if (fix_) {
    lat_ = lat;
    lon_ = lon;
    alt_ = static_cast<int16_t>(hMSL_mm / 1000);
    if (year >= 2020) epoch_ = civilToEpoch(year, mon, day, hh, mm, ss);
  }
  markValid();
}
