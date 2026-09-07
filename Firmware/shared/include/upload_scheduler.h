#pragma once

#include <stddef.h>
#include <stdint.h>

namespace lil {

// The owner serializes these operations. No allocation, I/O, or waiting occurs
// while accessing the scheduler; the HTTPS request runs after take() returns.
template <typename Job, size_t Capacity>
class UploadScheduler {
 public:
  static constexpr uint32_t kIntervalMs = 15000;
  static constexpr uint32_t kLifetimeMs = 24UL * 60UL * 60UL * 1000UL;
  struct Entry {
    Job job{};
    uint64_t order = 0;
    uint32_t bornMs = 0;
    uint32_t retryStartedMs = 0;
    uint8_t attempts = 0;
  };

  void push(const Job& job, uint32_t now) {
    Entry entry{};
    entry.job = job;
    entry.order = ++order_;
    entry.bornMs = now;
    insert(entry);
  }

  void expire(uint32_t now) {
    for (size_t i = 0; i < count_;) {
      if (now - entries_[i].bornMs >= kLifetimeMs) {
        erase(i);
        ++dropped_;
      } else {
        ++i;
      }
    }
  }

  bool take(uint32_t now, Entry& output) {
    expire(now);
    for (size_t i = 0; i < count_; ++i) {
      const Entry& candidate = entries_[i];
      bool earlierOnChannel = false;
      for (size_t j = 0; j < i; ++j) {
        earlierOnChannel |= entries_[j].job.channelId == candidate.job.channelId;
      }
      if (earlierOnChannel ||
          (candidate.attempts != 0 &&
           now - candidate.retryStartedMs < candidate.attempts * kIntervalMs) ||
          !channelReady(candidate.job.channelId, now)) continue;
      output = candidate;
      erase(i);
      return true;
    }
    return false;
  }

  void complete(Entry entry, uint32_t now, bool success, bool retryable) {
    size_t slot = Capacity;
    for (size_t i = 0; i < Capacity; ++i) {
      if (rates_[i].channel == entry.job.channelId) { slot = i; break; }
    }
    if (slot == Capacity) {
      for (size_t i = 0; i < Capacity; ++i) {
        if (rates_[i].channel == 0 || now - rates_[i].finishedMs >= kIntervalMs) {
          slot = i;
          break;
        }
      }
    }
    // take() reserves an eligible channel; only one request may be in flight.
    if (slot == Capacity) { ++dropped_; return; }
    rates_[slot] = Rate{entry.job.channelId, now};
    if (success) return;
    ++entry.attempts;
    if (!retryable || entry.attempts >= 3 || now - entry.bornMs >= kLifetimeMs) {
      ++dropped_;
      return;
    }
    entry.retryStartedMs = now;
    insert(entry);
  }

  uint32_t dropped() const { return dropped_; }
  size_t size() const { return count_; }

 private:
  struct Rate { uint32_t channel = 0; uint32_t finishedMs = 0; };
  bool channelReady(uint32_t channel, uint32_t now) const {
    bool freeSlot = false;
    for (const auto& rate : rates_) {
      if (rate.channel == channel) return now - rate.finishedMs >= kIntervalMs;
      freeSlot |= rate.channel == 0 || now - rate.finishedMs >= kIntervalMs;
    }
    return freeSlot;
  }
  void erase(size_t index) {
    for (size_t i = index + 1; i < count_; ++i) entries_[i - 1] = entries_[i];
    --count_;
  }
  void insert(const Entry& entry) {
    if (count_ == Capacity) {
      ++dropped_;
      if (entry.order < entries_[0].order) return;
      erase(0);
    }
    size_t i = count_++;
    while (i > 0 && entries_[i - 1].order > entry.order) {
      entries_[i] = entries_[i - 1];
      --i;
    }
    entries_[i] = entry;
  }
  Entry entries_[Capacity]{};
  Rate rates_[Capacity]{};
  size_t count_ = 0;
  uint64_t order_ = 0;
  uint32_t dropped_ = 0;
};
}  // namespace lil
