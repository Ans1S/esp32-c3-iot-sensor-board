#pragma once

#include "lil_protocol.h"

#if defined(SENSOR_RADIO_BENCHMARK) && SENSOR_RADIO_BENCHMARK
#include <esp_timer.h>
#include "sensor_log.h"
#if !SENSOR_DIAGNOSTIC_LOGGING
#error "The radio benchmark requires SENSOR_DIAGNOSTIC_LOGGING=1"
#endif
#endif

namespace sensor {
#if defined(SENSOR_RADIO_BENCHMARK) && SENSOR_RADIO_BENCHMARK
// Only the sensor's radio owner task sends telemetry and updates this count.
inline uint32_t benchmarkTelemetryFrames = 0;
inline void noteBenchmarkTelemetryFrame() { ++benchmarkTelemetryFrames; }

class RadioBenchmark {
 public:
  void finish(const lil::protocol::TelemetryPacket& packet, bool acknowledged) const {
    const auto elapsedUs = esp_timer_get_time() - startedUs_;
    SENSOR_LOG_PRINTF("WCH_RADIO,1,%u,%lu,%lu,%u,%lu,%u,%llu\n",
                      unsigned(PCB_VERSION), static_cast<unsigned long>(packet.payload.bootCount),
                      static_cast<unsigned long>(packet.header.sequence), unsigned(packet.payload.operatingMode),
                      static_cast<unsigned long>(benchmarkTelemetryFrames - initialFrames_),
                      unsigned(acknowledged), static_cast<unsigned long long>(elapsedUs));
  }

 private:
  const int64_t startedUs_ = esp_timer_get_time();
  const uint32_t initialFrames_ = benchmarkTelemetryFrames;
};
#endif
}  // namespace sensor

// Remove the declarations as well as the calls in ordinary builds so the
// compiler can keep the same stack slots and register allocation.
#if defined(SENSOR_RADIO_BENCHMARK) && SENSOR_RADIO_BENCHMARK
#define WCH_RADIO_BENCHMARK_BEGIN() const sensor::RadioBenchmark radioBenchmark
#define WCH_RADIO_BENCHMARK_FRAME() sensor::noteBenchmarkTelemetryFrame()
#define WCH_RADIO_BENCHMARK_FINISH(ready, packet, result) \
  do { if (ready) radioBenchmark.finish(packet, result.configReceived); } while (false)
#else
#define WCH_RADIO_BENCHMARK_BEGIN()
#define WCH_RADIO_BENCHMARK_FRAME()
#define WCH_RADIO_BENCHMARK_FINISH(ready, packet, result)
#endif
