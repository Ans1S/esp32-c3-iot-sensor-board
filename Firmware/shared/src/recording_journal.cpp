#include "recording_journal.h"
#if defined(ESP_PLATFORM)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif

namespace lil::recording {
namespace {
constexpr uint32_t kMagic = 0x52454331;
bool erased(const void* data, size_t count) {
  const auto* bytes = static_cast<const uint8_t*>(data);
  size_t i = 0;
  for (; i + 4 <= count; i += 4) {
    uint32_t word;
    memcpy(&word, bytes + i, sizeof(word));
    if (word != 0xFFFFFFFFUL) return false;
  }
  for (; i < count; ++i) if (bytes[i] != 0xff) return false;
  return true;
}
bool bit(const uint8_t* bits, uint8_t slot) { return bits[slot / 8] & (1U << (slot % 8)); }
bool pageFullyAcknowledged(const uint8_t* committed, const uint8_t* acknowledged, uint8_t slots) {
  size_t fullWords = slots / 32;
  for (size_t w = 0; w < fullWords; ++w) {
    uint32_t c, a;
    memcpy(&c, committed + w * 4, sizeof(c));
    memcpy(&a, acknowledged + w * 4, sizeof(a));
    if ((c | a) != 0U) return false;
  }
  const uint8_t rem = slots % 32;
  if (rem) {
    uint32_t c, a;
    memcpy(&c, committed + fullWords * 4, sizeof(c));
    memcpy(&a, acknowledged + fullWords * 4, sizeof(a));
    const uint32_t mask = (1UL << rem) - 1UL;
    if (((c | a) & mask) != 0U) return false;
  }
  return true;
}
}
bool Journal::readHeader(size_t page, Header& header) {
  return flash_->read(page * kPageBytes, &header, sizeof(header));
}
bool Journal::clearBit(size_t page, size_t field, uint8_t slot) {
  const size_t offset = page * kPageBytes + field + (slot / 32) * 4;
  uint32_t word;
  if (!flash_->read(offset, &word, 4)) return false;
  word &= ~(1UL << (slot % 32));
  uint32_t check;
  return flash_->write(offset, &word, 4) && flash_->read(offset, &check, 4) && check == word;
}
bool Journal::begin(Flash& flash, uint64_t sessionFilter, bool freshSession) {
  flash_ = &flash; fault_ = false; pending_ = generation_ = lastSampleMs_ = 0;
  ackOffset_ = 0;
  const size_t count = flash.bytes() / kPageBytes;
  if (count == 0) {
    pages_.reset();
    pageCount_ = 0;
    return false;
  }
  if (pageCount_ != count || !pages_) {
    pages_.reset(new (std::nothrow) Page[count]());
    if (!pages_) {
      pageCount_ = 0;
      return false;
    }
    pageCount_ = count;
  } else {
    for (size_t i = 0; i < pageCount_; ++i) pages_[i] = Page{};
  }
  auto failBegin = [&]() {
    fault_ = true;
    pending_ = 0;
    pageCount_ = 0;
    pages_.reset();
    flash_ = nullptr;
    return false;
  };
  for (size_t page = 0; page < pageCount_; ++page) {
#if defined(ESP_PLATFORM)
    if ((page & 15U) == 15U) vTaskDelay(1);
#endif
    Header header;
    auto& state = pages_[page];
    if (!readHeader(page, header)) return failBegin();
    if (erased(&header, sizeof(header))) continue;
    if (header.magic != kMagic || header.version != kFormat ||
        (header.width != 32 && header.width != 64 && header.width != 128) ||
        header.crc != protocol::crc32(reinterpret_cast<const uint8_t*>(&header), 12)) {
      state.usable = false; fault_ = true; continue;
    }
    state.generation = header.generation; state.width = header.width;
    if (state.generation > generation_) generation_ = state.generation;
    const uint8_t slots = (kPageBytes - kHeaderBytes) / state.width;
    // A brand-new user-started session replaces prior pages; headers alone
    // establish the highest generation without scanning all 1.375 MiB of data.
    if (freshSession) {
      state.width = 0;
      continue;
    }
    // A full page with every slot committed and acknowledged has no pending
    // data and can skip the 4 KiB payload scan on reboot.
    if (pageFullyAcknowledged(header.committed, header.acknowledged, slots)) {
      state.next = slots;
      continue;
    }
    bool belongs = false;
    // Read consecutive records together to avoid a flash transaction for
    // every slot during startup recovery. Each slot is still validated
    // separately before its commit/ACK state is considered.
    constexpr size_t kBatchSlots = 4;
    alignas(uint32_t) uint8_t batch[kBatchSlots * sizeof(Record)];
    for (size_t first = 0; first < slots; first += kBatchSlots) {
      const size_t batchSlots = slots - first < kBatchSlots ? slots - first : kBatchSlots;
      if (!flash.read(page * kPageBytes + kHeaderBytes + first * state.width,
                      batch, batchSlots * state.width)) return failBegin();
      for (size_t index = 0; index < batchSlots; ++index) {
        const uint8_t slot = static_cast<uint8_t>(first + index);
        Record record{};
        memcpy(&record, batch + index * state.width, state.width);
        if (erased(&record, state.width)) {
          if (!bit(header.committed, slot)) { fault_ = true; state.usable = false; }
          continue;
        }
        state.next = slot + 1; // Torn writes burn a slot, never overwrite it.
        const bool recordValid = valid(record);
        const bool pendingCommitted = !bit(header.committed, slot) && bit(header.acknowledged, slot);
        if (sessionFilter && record.session != sessionFilter) {
          // A bit flip inside bytes 0..7 corrupts both session and CRC. Never
          // silently recycle a committed, unacknowledged corrupted slot.
          if (pendingCommitted && !recordValid) {
            fault_ = true;
            state.usable = false;
          }
          continue;
        }
        belongs = true;
        if (recordValid && record.sampleMs > lastSampleMs_) lastSampleMs_ = record.sampleMs;
        if (bit(header.committed, slot) && size(record.type) == state.width && recordValid) {
          if (!clearBit(page, offsetof(Header, committed), slot)) return failBegin();
          header.committed[slot / 8] &= ~(1U << (slot % 8));
        }
        if (!bit(header.committed, slot) && bit(header.acknowledged, slot)) {
          ++state.pending; ++pending_;
          if (size(record.type) != state.width || !recordValid) fault_ = true;
        }
      }
    }
    // A new user-started session logically replaces the old one. Erase pages
    // lazily as needed, not all 1.375 MiB before the first measurement.
    if (sessionFilter && !belongs && state.usable) {
      state.width = 0;
      state.next = 0;
      state.pending = 0;
    }
  }
  return true;
}
bool Journal::clear() {
  if (!flash_) return false;
  for (size_t page = 0; page < pageCount_; ++page)
    if (!flash_->erase(page * kPageBytes, kPageBytes)) { fault_ = true; return false; }
  return begin(*flash_);
}
bool Journal::initialize(size_t page, size_t width) {
  if (pages_[page].pending || !pages_[page].usable || generation_ >= UINT32_MAX - 1) return false;
  if (!flash_->erase(page * kPageBytes, kPageBytes)) { fault_ = true; return false; }
  Header header;
  memset(&header, 0xff, sizeof(header));
  header.magic = kMagic; header.generation = ++generation_;
  header.width = width; header.version = kFormat;
  header.crc = protocol::crc32(reinterpret_cast<const uint8_t*>(&header), 12);
  uint8_t check[16];
  if (!flash_->write(page * kPageBytes, &header, 16) ||
      !flash_->read(page * kPageBytes, check, 16) ||
      memcmp(&header, check, 16) != 0) {
    fault_ = true; pages_[page].usable = false; return false;
  }
  pages_[page] = {}; pages_[page].width = width; pages_[page].generation = generation_;
  return true;
}
bool Journal::append(const Record& record) {
  if (!flash_ || !valid(record)) return false;
  if (!reconcileAcknowledgement()) return false;
  const size_t width = size(record.type);
  size_t chosen = pageCount_;
  uint32_t newest = 0;
  for (size_t i = 0; i < pageCount_; ++i) {
    const auto& page = pages_[i];
    if (page.usable && page.width == width && page.next < (kPageBytes - kHeaderBytes) / width && page.generation > newest) {
      chosen = i; newest = page.generation;
    }
  }
  if (chosen == pageCount_) {
    uint32_t oldest = UINT32_MAX;
    for (size_t i = 0; i < pageCount_; ++i)
      if (pages_[i].usable && !pages_[i].pending && pages_[i].generation < oldest) {
        chosen = i; oldest = pages_[i].generation;
      }
    if (chosen == pageCount_ || !initialize(chosen, width)) return false;
  }
  auto& page = pages_[chosen];
  const uint8_t slot = page.next++;
  const size_t offset = chosen * kPageBytes + kHeaderBytes + slot * width;
  Record check{};
  if (!flash_->write(offset, &record, width) || !flash_->read(offset, &check, width) ||
      memcmp(&record, &check, width) || !clearBit(chosen, offsetof(Header, committed), slot)) {
    // Reopen before further writes: a failed commit may have reached flash.
    page.usable = false; fault_ = true; return false;
  }
  ++page.pending; ++pending_; lastSampleMs_ = record.sampleMs; return true;
}
bool Journal::reconcileAcknowledgement() {
  if (!ackOffset_) return true;
  uint32_t word;
  if (!flash_->read(ackOffset_, &word, sizeof(word))) return false;
  if (word == ackAfter_) {
    --pages_[ackOffset_ / kPageBytes].pending;
    --pending_;
  } else if (word != ackBefore_) {
    fault_ = true;
    return false;
  }
  ackOffset_ = 0;
  return true;
}
bool Journal::locate(size_t& page, uint8_t& slot, Header& header) {
  if (!reconcileAcknowledgement()) return false;
  page = pageCount_; uint32_t oldest = UINT32_MAX;
  for (size_t i = 0; i < pageCount_; ++i)
    if (pages_[i].pending && pages_[i].generation < oldest) { page = i; oldest = pages_[i].generation; }
  if (page == pageCount_) return false;
  if (!readHeader(page, header)) { fault_ = true; return false; }
  for (slot = 0; slot < pages_[page].next; ++slot)
    if (!bit(header.committed, slot) && bit(header.acknowledged, slot)) return true;
  fault_ = true; return false;
}
bool Journal::peek(Record& record) {
  size_t page; uint8_t slot;
  Header header;
  record = {};
  if (!locate(page, slot, header)) return false;
  if (!flash_->read(page * kPageBytes + kHeaderBytes + slot * pages_[page].width, &record, pages_[page].width) ||
      size(record.type) != pages_[page].width || !valid(record)) { fault_ = true; return false; }
  return true;
}
bool Journal::acknowledge(const Record& record) {
  size_t page; uint8_t slot;
  Header header;
  if (!locate(page, slot, header)) return false;
  Record oldest{};
  if (!flash_->read(page * kPageBytes + kHeaderBytes + slot * pages_[page].width,
                    &oldest, pages_[page].width) ||
      size(oldest.type) != pages_[page].width || !valid(oldest)) {
    fault_ = true; return false;
  }
  if (record.session != oldest.session || record.sampleMs != oldest.sampleMs ||
      checksum(record) != checksum(oldest)) return false;
  const size_t wordOffset = (slot / 32) * sizeof(uint32_t);
  memcpy(&ackBefore_, header.acknowledged + wordOffset, sizeof(ackBefore_));
  ackAfter_ = ackBefore_ & ~(uint32_t(1) << (slot % 32));
  ackOffset_ = page * kPageBytes + offsetof(Header, acknowledged) + wordOffset;
  const uint32_t before = pending_;
  // The bit can reach flash even if the write or its readback reports an
  // error. Resolve that word before advancing to another record.
  flash_->write(ackOffset_, &ackAfter_, sizeof(ackAfter_));
  return reconcileAcknowledgement() && pending_ < before;
}
}  // namespace lil::recording
