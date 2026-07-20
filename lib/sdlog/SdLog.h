/**
 * @file SdLog.h
 * @brief Binary, batched SD-card logger for fixed-size records (Decision D4).
 *
 * Records are buffered in RAM and flushed to the card in batches on a 512-byte
 * boundary, so the (high-latency, blocking) SD write never happens on the
 * acquisition path and card wear stays low. Runs on its own FreeRTOS task.
 */

#ifndef SD_LOG_H
#define SD_LOG_H

#include <Arduino.h>
#include <FS.h>
#include <SPI.h>

class SdLog {
 public:
  /**
   * @param csPin     SPI chip-select GPIO for the card module.
   * @param flushEveryN  Number of records to accumulate before writing to disk.
   * @param recordSize   Size in bytes of each fixed-length record.
   */
  SdLog(int csPin, uint16_t flushEveryN, uint16_t recordSize)
      : csPin_(csPin), flushEveryN_(flushEveryN), recordSize_(recordSize) {}

  /**
   * @brief Mount the card (on the given SPI bus) and open a fresh log file.
   * @param spi  Configured SPIClass instance (bus pins already set by caller).
   * @return true if the card mounted and a log file was opened.
   */
  bool begin(SPIClass& spi);

  /**
   * @brief Append one record to the RAM batch buffer.
   * @param data  Pointer to @c recordSize bytes.
   * @param len   Length of @p data; must equal @c recordSize.
   * @return true if buffered (or flushed) successfully; false on overflow/IO error.
   */
  bool append(const uint8_t* data, uint16_t len);

  /**
   * @brief Force any buffered records to be written and fsync'd to the card.
   * @return true on success.
   */
  bool flush();

  /** @return true if the card is mounted and the log file is open. */
  bool healthy() const { return healthy_; }

 private:
  static constexpr uint16_t kMaxBatchBytes = 4096;

  int      csPin_;
  uint16_t flushEveryN_;
  uint16_t recordSize_;
  bool     healthy_ = false;

  File     file_;
  uint8_t  batch_[kMaxBatchBytes];
  uint16_t batchLen_ = 0;
  uint16_t pending_  = 0;
};

#endif  // SD_LOG_H
