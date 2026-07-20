#include "SdLog.h"

#include <SD.h>

bool SdLog::begin(SPIClass& spi) {
  if (!SD.begin(csPin_, spi)) {
    healthy_ = false;
    return false;
  }

  // Pick the next free LOGnnnnn.BIN so we never overwrite a previous flight.
  char name[16];
  for (uint32_t i = 0; i < 100000; ++i) {
    snprintf(name, sizeof(name), "/LOG%05lu.BIN", static_cast<unsigned long>(i));
    if (!SD.exists(name)) break;
  }
  file_ = SD.open(name, FILE_WRITE);
  healthy_ = static_cast<bool>(file_);
  return healthy_;
}

bool SdLog::append(const uint8_t* data, uint16_t len) {
  if (!healthy_ || len != recordSize_) return false;

  // Flush early if this record would overflow the batch buffer.
  if (batchLen_ + len > kMaxBatchBytes && !flush()) return false;

  memcpy(&batch_[batchLen_], data, len);
  batchLen_ += len;
  if (++pending_ >= flushEveryN_) return flush();
  return true;
}

bool SdLog::flush() {
  if (!healthy_) return false;
  if (batchLen_ == 0) return true;

  const uint16_t want = batchLen_;
  const size_t wrote = file_.write(batch_, want);
  file_.flush();                 // fsync so at most the current batch is at risk
  batchLen_ = 0;
  pending_  = 0;

  if (wrote != want) { healthy_ = false; return false; }
  return true;
}
