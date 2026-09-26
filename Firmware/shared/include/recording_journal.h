#pragma once

#include <vector>
#include "recording_protocol.h"

namespace lil::recording {
class Flash {
 public:
  virtual ~Flash() = default;
  virtual size_t bytes() const = 0;
  virtual bool read(size_t offset, void* data, size_t length) = 0;
  virtual bool write(size_t offset, const void* data, size_t length) = 0;
  virtual bool erase(size_t offset, size_t length) = 0;
};

// Append-only pages. A slot is committed only after its CRC passes readback.
// Acknowledgements clear a separate bit; only pages without pending data recycle.
class Journal {
 public:
  static constexpr size_t kPageBytes = 4096, kHeaderBytes = 64;
  bool begin(Flash& flash, uint64_t sessionFilter = 0);
  bool replaceSession(uint64_t session) { return flash_ && begin(*flash_, session); }
  bool clear();
  bool append(const Record& record);
  bool peek(Record& record);
  bool acknowledge(const Record& record);
  uint32_t pending() const { return pending_; }
  bool fault() const { return fault_; }
  uint32_t lastSampleMs() const { return lastSampleMs_; }
  size_t capacity(protocol::EnvironmentalSensorType type) const {
    const size_t width = size(type);
    return width ? pages_.size() * ((kPageBytes - kHeaderBytes) / width) : 0;
  }
 private:
  struct Page {
    uint32_t generation = 0;
    uint16_t width = 0;
    uint8_t next = 0, pending = 0;
    bool usable = true;
  };
  struct Header {
    uint32_t magic, generation;
    uint16_t width, version;
    uint32_t crc;
    uint8_t committed[16], acknowledged[16], reserved[16];
  };
  static_assert(sizeof(Header) == kHeaderBytes);
  bool clearBit(size_t page, size_t field, uint8_t slot);
  bool readHeader(size_t page, Header& header);
  bool initialize(size_t page, size_t width);
  bool locate(size_t& page, uint8_t& slot);
  Flash* flash_ = nullptr;
  std::vector<Page> pages_;
  uint32_t generation_ = 0, pending_ = 0;
  uint32_t lastSampleMs_ = 0;
  bool fault_ = false;
};
}  // namespace lil::recording
