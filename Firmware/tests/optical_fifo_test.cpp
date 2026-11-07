#ifdef NDEBUG
#error "Regression assertions must be enabled"
#endif
#include <Arduino.h>
#include <Wire.h>
#include <cassert>
#include <cstdio>
#include "precision_sensors.h"

#ifndef EXPECT_OPTICAL_BURST
#define EXPECT_OPTICAL_BURST 1
#endif
void testRadioTick() { ++testMillis; }
int main() {
  using namespace lil::protocol;
  for (unsigned count = 0; count < 32; ++count) {
    Wire = PrecisionWire{}; Wire.present = 0x57;
    sensor::PrecisionSensors driver;
    assert(driver.begin(EnvironmentalSensorType::kMax30102, 0x57));
    for (unsigned i = 0; i < count; ++i) Wire.sample(0xC0000U | (100000 + i), 0xC0000U | (120000 + i));
    const auto result = driver.read();
    assert(result.valid == (count != 0) && Wire.fifo.empty());
    const unsigned retained = count < kOpticalBatchSize ? count : kOpticalBatchSize;
    assert(result.live.count == retained);
    assert(Wire.fifoReadCalls == (EXPECT_OPTICAL_BURST ? (count + 7) / 8 : count));
    for (unsigned i = 0; i < retained; ++i) {
      const auto& frame = result.live.optical[i];
      assert(frame.red == 100000 + count - retained + i);
      assert(frame.infrared == 120000 + count - retained + i);
      assert(frame.ageMs == (retained - 1 - i) * lil::timing::kOpticalSampleMs);
    }
    if (count > kOpticalBatchSize) assert(result.live.flags & kLiveGap);
    if (count == 5 || count == 8 || count == 31) {
      size_t payload = 0, wireBits = 0;
      for (const auto bytes : Wire.fifoRequests) { payload += bytes; wireBits += (3 + bytes) * 9; }
      assert(payload == count * 6);
      printf("Optical FIFO count=%u: %u read transactions, %zu payload bytes, %zu address/register/data clock bits\n",
             count, Wire.fifoReadCalls, payload, wireBits);
    }
  }
  for (unsigned failAt = 1; failAt <= 4; ++failAt) {
    Wire = PrecisionWire{}; Wire.present = 0x57;
    sensor::PrecisionSensors driver;
    assert(driver.begin(EnvironmentalSensorType::kMax30102, 0x57));
    for (unsigned i = 0; i < 31; ++i) Wire.sample(100000 + i);
    Wire.shortFifoAt = failAt;
    const auto failed = driver.read();
    assert(!failed.valid && !failed.live.count && (failed.live.flags & kLiveGap));
    assert(!driver.poll());
    assert(driver.begin(EnvironmentalSensorType::kMax30102, 0x57));
    assert(Wire.fifo.empty());
    Wire.sample(100000);
    assert(driver.read().valid);
  }
  puts("Optical FIFO: every 0..31-frame snapshot preserves masked values/order/ages; short batches invalidate capture and require reinitialization");
}
