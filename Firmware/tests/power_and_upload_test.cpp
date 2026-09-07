#ifdef NDEBUG
#error "Regression assertions must be enabled"
#endif
#include <assert.h>
#include <initializer_list>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "power_policy.h"
#include "upload_scheduler.h"
#include "lil_protocol.h"

struct Job { uint32_t channelId; uint32_t value; };

int main() {
  using namespace lil::power;
  assert(discoverySleepSeconds(599) == 10);
  assert(discoverySleepSeconds(600) == 300);
  assert(discoverySleepSeconds(360000) == 300);
  assert(discoverySleepSeconds(4200, 3600) == 600);
  assert(discoverySleepSeconds(999999, 3600) == 3600);
  assert(reportDue(false, 0, 0, 3600));
  assert(!reportDue(true, 300000, 0, 3600));
  assert(!reportDue(true, 3599999, 0, 3600));
  assert(reportDue(true, 3600000, 0, 3600));
  assert(scheduledSleepSeconds(false, 123000, 10000, 300) == 300);
  assert(scheduledSleepSeconds(true, 123000, 10000, 300) == 300);
  assert(scheduledSleepSeconds(true, 123000, 123001, 300) == 1);
  assert(scheduledSleepSeconds(true, 123000, 423001, 300) == 300);
  assert(scheduledSleepSeconds(true, 123000, 133001, 300) == 11);

  // Exhaust every possible 12-sample sum for both dividers at representative
  // factors, then every representable permitted float at a maximum ADC sum.
  unsigned largestDifference = 0;
  const auto check = [&](uint32_t sum, float factor, uint32_t n, uint32_t d) {
    uint32_t bits;
    memcpy(&bits, &factor, 4);
    const uint32_t gain = calibrationQ20(bits);
    assert(gain != 0);
    const uint32_t value = batteryMillivolts(sum, gain, n, d);
    const float old = (float(sum) / 12.0F) * (float(n) / float(d)) * factor;
    const unsigned difference = unsigned(abs(int(value) - int(lroundf(old))));
    if (difference > largestDifference) largestDifference = difference;
    assert(difference <= 1);
  };
  for (uint32_t sum = 0; sum <= 12 * 3300; ++sum) {
    for (float factor : {0.7F, 0.75F, 0.9F, 1.0F, 1.1F, 1.25F, 1.3F}) {
      check(sum, factor, 167, 100);
      check(sum, factor, 5, 3);
    }
  }
  uint32_t low, high;
  float lower = 0.7F, upper = 1.3F;
  memcpy(&low, &lower, 4); memcpy(&high, &upper, 4);
  for (uint32_t bits = low; bits <= high; ++bits) {
    float factor;
    memcpy(&factor, &bits, 4);
    check(39600, factor, 167, 100);
    check(39600, factor, 5, 3);
  }
  assert(calibrationQ20(0x7fc00000) == 0);
  assert(calibrationQ20(0x7f800000) == 0);
  assert(calibrationQ20(0xbf800000) == 0);
  assert(calibrationQ20(0) == 0);

  assert(lil::protocol::crc32(nullptr, 0) == 0);
  assert(lil::protocol::crc32(reinterpret_cast<const uint8_t*>("123456789"), 9)
         == 0xcbf43926);
  lil::protocol::TelemetryPacket packet{};
  packet.payload.batteryMillivolts = 3789;
  lil::protocol::finalize(packet, lil::protocol::MessageType::kTelemetry, 123);
  assert(lil::protocol::validate(packet, sizeof(packet),
                                 lil::protocol::MessageType::kTelemetry));
  auto* bytes = reinterpret_cast<uint8_t*>(&packet);
  for (size_t i = 0; i < sizeof(packet); ++i) {
    bytes[i] ^= 1;
    assert(!lil::protocol::validate(packet, sizeof(packet),
                                    lil::protocol::MessageType::kTelemetry));
    bytes[i] ^= 1;
  }

  uint8_t legacy[sizeof(lil::protocol::PacketHeader) + lil::protocol::kLegacyTelemetryPayloadSize]{};
  memcpy(legacy, &packet, sizeof(legacy));
  lil::protocol::finalizePacket(legacy, sizeof(legacy), lil::protocol::MessageType::kTelemetry,
                              124, lil::protocol::kLegacyTelemetryPayloadSize);
  assert(lil::protocol::validatePacket(legacy, sizeof(legacy), lil::protocol::MessageType::kTelemetry,
                                       lil::protocol::kLegacyTelemetryPayloadSize));
  legacy[sizeof(legacy) - 1] ^= 1;
  assert(!lil::protocol::validatePacket(legacy, sizeof(legacy), lil::protocol::MessageType::kTelemetry,
                                        lil::protocol::kLegacyTelemetryPayloadSize));
  using Scheduler = lil::UploadScheduler<Job, 4>;
  Scheduler scheduler;
  Scheduler::Entry entry;
  scheduler.push({1, 10}, 0);
  scheduler.push({1, 11}, 0);
  scheduler.push({2, 20}, 0);
  assert(scheduler.take(0, entry) && entry.job.value == 10);
  scheduler.complete(entry, 0, true, false);
  assert(scheduler.take(1, entry) && entry.job.value == 20);
  scheduler.complete(entry, 1, true, false);
  assert(!scheduler.take(14999, entry));
  assert(scheduler.take(15000, entry) && entry.job.value == 11);
  scheduler.complete(entry, 15000, false, true);
  scheduler.push({1, 12}, 15001);
  assert(!scheduler.take(29999, entry));
  assert(scheduler.take(30000, entry) && entry.job.value == 11);
  scheduler.complete(entry, 30000, false, true);
  assert(!scheduler.take(59999, entry));
  assert(scheduler.take(60000, entry) && entry.job.value == 11);
  scheduler.complete(entry, 60000, false, true);
  assert(scheduler.dropped() == 1);
  assert(scheduler.take(75000, entry) && entry.job.value == 12);
  scheduler.complete(entry, 75000, true, false);

  Scheduler full;
  for (unsigned i = 1; i <= 5; ++i) full.push({i, i}, i);
  assert(full.size() == 4 && full.dropped() == 1);
  assert(full.take(5, entry) && entry.job.value == 2);
  full.complete(entry, 5, false, true);
  full.expire(5 + Scheduler::kLifetimeMs);
  assert(full.size() == 0 && full.dropped() == 5);

  Scheduler wrap;
  const uint32_t nearWrap = UINT32_MAX - 100;
  wrap.push({1, 1}, nearWrap);
  assert(wrap.take(nearWrap, entry));
  wrap.complete(entry, nearWrap, true, false);
  wrap.push({1, 2}, 0);
  wrap.push({2, 3}, 0);
  assert(wrap.take(0, entry) && entry.job.value == 3);
  wrap.complete(entry, 0, true, false);
  assert(!wrap.take(14898, entry));
  assert(wrap.take(14899, entry) && entry.job.value == 2);
  printf("Power, CRC and upload scheduler tests passed; ADC maximum delta: %u mV\n",
         largestDifference);
}
