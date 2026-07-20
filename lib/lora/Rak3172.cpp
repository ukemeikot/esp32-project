#include "Rak3172.h"

bool Rak3172::begin(uint32_t baud, int rxPin, int txPin) {
  uart_.begin(baud, SERIAL_8N1, rxPin, txPin);
  delay(100);
  // Flush any boot banner, then probe.
  while (uart_.available()) uart_.read();
  return sendAt("AT");
}

bool Rak3172::configureP2P(uint32_t freqHz, uint8_t sf, uint8_t bwCode, uint8_t crCode,
                           uint8_t preamble, uint8_t txDbm) {
  char cmd[48];
  bool ok = true;

  ok &= sendAt("AT+NWM=0");                               // work mode: P2P/LoRa
  snprintf(cmd, sizeof(cmd), "AT+PFREQ=%lu", static_cast<unsigned long>(freqHz));
  ok &= sendAt(cmd);
  snprintf(cmd, sizeof(cmd), "AT+PSF=%u", static_cast<unsigned>(sf));
  ok &= sendAt(cmd);
  snprintf(cmd, sizeof(cmd), "AT+PBW=%u", static_cast<unsigned>(bwCode));
  ok &= sendAt(cmd);
  snprintf(cmd, sizeof(cmd), "AT+PCR=%u", static_cast<unsigned>(crCode));
  ok &= sendAt(cmd);
  snprintf(cmd, sizeof(cmd), "AT+PPL=%u", static_cast<unsigned>(preamble));
  ok &= sendAt(cmd);
  snprintf(cmd, sizeof(cmd), "AT+PTP=%u", static_cast<unsigned>(txDbm));
  ok &= sendAt(cmd);
  return ok;
}

bool Rak3172::send(const uint8_t* data, uint16_t len) {
  // AT+PSEND takes an even-length hex string of the payload.
  static const char kHex[] = "0123456789ABCDEF";
  char cmd[16 + 2 * 128];
  int n = snprintf(cmd, sizeof(cmd), "AT+PSEND=");
  for (uint16_t i = 0; i < len && n + 2 < static_cast<int>(sizeof(cmd)); ++i) {
    cmd[n++] = kHex[data[i] >> 4];
    cmd[n++] = kHex[data[i] & 0x0F];
  }
  cmd[n] = '\0';
  return sendAt(cmd, 3000);   // transmit can take a while at high SF
}

bool Rak3172::sendAt(const char* cmd, uint32_t timeoutMs) {
  uart_.print(cmd);
  uart_.print("\r\n");

  // Scan the reply stream for a line that is exactly OK / +... OK / ERROR.
  char line[64];
  uint8_t len = 0;
  const uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    while (uart_.available()) {
      const int c = uart_.read();
      if (c < 0) break;
      if (c == '\n' || c == '\r') {
        if (len > 0) {
          line[len] = '\0';
          if (strncmp(line, "OK", 2) == 0) return true;
          if (strncmp(line, "ERROR", 5) == 0) return false;
        }
        len = 0;
      } else if (len < sizeof(line) - 1) {
        line[len++] = static_cast<char>(c);
      }
    }
  }
  return false;   // timed out
}
