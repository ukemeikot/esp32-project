// aqm_protocol.h — Over-the-air LoRa packet contract (SOURCE OF TRUTH).
//
// This header is the single canonical definition of the packet exchanged between
// the drone node (this repo) and the ground node. It is vendored BYTE-IDENTICALLY
// into the aqm-ground-station repo (firmware/lib/aqm_protocol/). If you change it
// here, copy it there in the same commit. See docs/IMPLEMENTATION_PLAN.md §5 / D6.
//
// Header-only and dependency-free on purpose, so it copies cleanly between repos.
//
// Wire format: little-endian, packed, fixed 39 bytes. CRC-16/CCITT-FALSE over all
// preceding bytes. The ESP32 is little-endian, so the packed struct maps 1:1 to the
// wire bytes and needs no per-field serialisation.

#ifndef AQM_PROTOCOL_H
#define AQM_PROTOCOL_H

#include <stdint.h>
#include <string.h>

namespace aqm {

static constexpr uint8_t kMagic = 0xA6;      // 'AQM' marker
static constexpr uint8_t kVersion = 1;       // bump on any layout change

// Fixed-point scaling of transmitted fields (reversed identically on the ground).
//   lat/lon : degrees * 1e7   (i32)
//   alt     : metres          (i16)
//   co2     : ppm             (u16)
//   pmX     : ug/m^3          (u16)
//   vbat    : millivolts      (u16)
// Gas channels use a per-channel multiplier so each fits in u16 across its range:
//   [0]=H2, [1]=O3, [2]=SO2 -> ppb  (x1000, good to ~65 ppm)
//   [3]=CO                  -> 0.1 ppm units (x10, good to ~6553 ppm; sensor is 0-1000)
static constexpr uint16_t kGasTxMul[4] = {1000, 1000, 1000, 10};

// flags bitfield
enum Flags : uint8_t {
  FLAG_GPS_FIX   = 1 << 0,
  FLAG_EVENT     = 1 << 1,  // threshold exceedance / alert packet
  FLAG_WARMUP    = 1 << 2,  // electrochemical cells still stabilising
  FLAG_FAULT_ADC = 1 << 3,
  FLAG_FAULT_GPS = 1 << 4,
  FLAG_FAULT_CO2 = 1 << 5,
  FLAG_FAULT_PM  = 1 << 6,
  FLAG_FAULT_SD  = 1 << 7,
};

#pragma pack(push, 1)
struct Packet {
  uint8_t  magic;      // = kMagic
  uint8_t  version;    // = kVersion
  uint16_t seq;        // monotonic packet counter
  uint32_t epoch;      // seconds (GPS-derived when fixed, else uptime)
  int32_t  lat;        // deg * 1e7
  int32_t  lon;        // deg * 1e7
  int16_t  alt;        // metres
  uint16_t gas[4];     // H2, O3, SO2, CO  (see kGasTxMul)
  uint16_t co2;        // ppm
  uint16_t pm1;        // ug/m^3
  uint16_t pm2_5;      // ug/m^3
  uint16_t pm10;       // ug/m^3
  uint16_t vbat;       // mV
  uint8_t  flags;      // Flags bitfield
  uint16_t crc16;      // CRC-16/CCITT-FALSE over bytes [0 .. sizeof-3]
};
#pragma pack(pop)

static constexpr uint16_t kPacketSize = sizeof(Packet);
static_assert(sizeof(Packet) == 39, "AQM packet must be 39 bytes on the wire");

// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, xorout 0).
// Table-free; this is the profiled hotspot candidate (plan §3.2) — swap for
// esp_rom_crc16_le or a hand-tuned Xtensa loop only if a profile shows it matters.
static inline uint16_t crc16_ccitt(const uint8_t* data, uint32_t len) {
  uint16_t crc = 0xFFFF;
  for (uint32_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

// Stamp magic/version/crc into an otherwise-populated packet, then copy the 41
// wire bytes into out (must hold >= kPacketSize). Returns bytes written.
static inline uint16_t finalize(Packet& p, uint8_t* out) {
  p.magic = kMagic;
  p.version = kVersion;
  p.crc16 = crc16_ccitt(reinterpret_cast<const uint8_t*>(&p), kPacketSize - 2);
  memcpy(out, &p, kPacketSize);
  return kPacketSize;
}

// Validate a received 41-byte buffer and copy it into p. Returns true if the
// length, magic, version and CRC all check out.
static inline bool parse(const uint8_t* in, uint32_t len, Packet& p) {
  if (len < kPacketSize) return false;
  memcpy(&p, in, kPacketSize);
  if (p.magic != kMagic || p.version != kVersion) return false;
  const uint16_t want = crc16_ccitt(in, kPacketSize - 2);
  return want == p.crc16;
}

}  // namespace aqm

#endif  // AQM_PROTOCOL_H
