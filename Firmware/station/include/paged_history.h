#pragma once
#include <Arduino.h>
#include <array>
#include "app_config.h"
#include "recording_protocol.h"
#include "filesystem_guard.h"

namespace station {
struct HistorySample;
struct NormalHistoryPoint {
  uint32_t timestamp = 0;
  lil::protocol::TelemetryPayload telemetry{};
};
struct NormalHistoryInfo {
  uint32_t revision = 0, points = 0, durablePoints = 0, pendingPoints = 0;
  uint32_t oldest = 0, newest = 0, capacityPoints = 0, intervalSeconds = 0;
  uint32_t dropped = 0;
  bool available = false, writeFailed = false;
};

// Small append-only files form an adaptive ring. No complete history is kept
// in RAM, and only normal-history pages may be recycled to recover space.
class PagedHistory {
 public:
  static constexpr size_t kPageBytes = 4096, kHeaderBytes = 64;
  static constexpr size_t kMaxPages = 256;
  static constexpr size_t kReserveBytes = 256U * 1024U;
  static constexpr uint32_t kFlushMs = 30000;
  bool begin(bool mounted);
  void configure(const SensorConfig* configs, size_t count);
  void setLegacyBytes(size_t bytes);
  bool appendLegacy(const uint8_t mac[6], const HistorySample& sample, uint32_t intervalSeconds);
  bool append(const uint8_t mac[6], uint32_t timestamp,
              const lil::protocol::TelemetryPayload& telemetry, uint32_t intervalSeconds,
              bool batchImport = false);
  bool flush(const uint8_t mac[6]);
  void flushDue();
  bool reserveForArchive();
  bool clear(const uint8_t mac[6]);
  NormalHistoryInfo info(const uint8_t mac[6]);
  size_t read(const uint8_t mac[6], uint32_t after, uint32_t until,
              uint32_t oldest, NormalHistoryPoint* output, size_t capacity,
              bool& stale, bool& readError);
 private:
  struct Page {
    uint8_t mac[6]{};
    uint32_t generation = 0, first = 0, last = 0, interval = 0;
    uint16_t count = 0, width = 0;
    uint8_t kind = 0;
  };
  struct Node {
    uint8_t mac[6]{};
    bool enabled = false, writeFailed = false;
    uint8_t kind = 0;
    uint16_t width = 0, pending = 0, bufferRows = 0;
    uint8_t* buffer = nullptr;
    uint32_t interval = 0, startedMs = 0, revision = 1, lastTimestamp = 0, dropped = 0;
  };
  int nodeIndex(const uint8_t mac[6]) const;
  size_t quotaPages();
  size_t pagesFor(const uint8_t mac[6]) const;
  bool erasePage(size_t index);
  bool makeSpace(const uint8_t mac[6], bool newPage);
  bool flushNode(Node& node);
  NormalHistoryInfo infoLocked(const Node& node) const;
  std::array<Page, kMaxPages> pages_{};
  std::array<Node, kMaxSensors> nodes_{};
  size_t pageCount_ = 0;
  size_t legacyBytes_ = 0;
  size_t cachedQuota_ = 0, budgetPages_ = kMaxPages, flushCursor_ = 0;
  uint32_t generation_ = 0;
  SemaphoreHandle_t mutex_ = nullptr;
  bool mounted_ = false;
};
extern PagedHistory pagedHistory;
}
