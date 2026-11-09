#pragma once

#include <algorithm>
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
    size_t retained = 0;
    for (size_t i = 0; i < count_; ++i) {
      if (now - entries_[i].bornMs >= kLifetimeMs) continue;
      if (retained != i) entries_[retained] = entries_[i];
      ++retained;
    }
    dropped_ += count_ - retained;
    count_ = retained;
  }

  // When no job can run, report the first time a queued job can become
  // eligible or expire. A newly queued job can always interrupt that wait.
  bool take(uint32_t now, Entry& output, uint32_t* waitMs = nullptr) {
    expire(now);
    uint32_t nextWait = UINT32_MAX;
    for (size_t i = 0; i < count_; ++i) {
      const Entry& candidate = entries_[i];
      nextWait = std::min(nextWait, kLifetimeMs - (now - candidate.bornMs));
      size_t earlier = 0;
      while (earlier < i && entries_[earlier].job.channelId != candidate.job.channelId) ++earlier;
      if (earlier != i) continue;
      const uint32_t retryWait = remaining(now - candidate.retryStartedMs, candidate.attempts * kIntervalMs);
      const uint32_t wait = std::max(retryWait, channelWait(candidate.job.channelId, now));
      if (wait != 0) {
        nextWait = std::min(nextWait, wait);
        continue;
      }
      output = candidate;
      erase(i);
      if (waitMs) *waitMs = 0;
      return true;
    }
    if (waitMs) *waitMs = nextWait;
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
  static uint32_t remaining(uint32_t elapsed, uint32_t duration) {
    return elapsed < duration ? duration - elapsed : 0;
  }
  uint32_t channelWait(uint32_t channel, uint32_t now) const {
    uint32_t slotWait = kIntervalMs;
    for (const auto& rate : rates_) {
      const uint32_t wait = remaining(now - rate.finishedMs, kIntervalMs);
      if (rate.channel == channel) return wait;
      slotWait = std::min(slotWait, rate.channel == 0 ? 0U : wait);
    }
    return slotWait;
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
