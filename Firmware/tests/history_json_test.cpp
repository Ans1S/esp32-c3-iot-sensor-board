#include <cassert>
#include <cmath>
#include <cstdio>
#include <chrono>
#include <string>
#include <ArduinoJson.h>
#include "history_json.h"

int main() {
  using namespace lil::protocol;
  station::NormalHistoryPoint points[4]{};
  for (unsigned i = 0; i < 4; ++i) {
    points[i].timestamp = 2208988800UL + i; // Beyond signed 32-bit epoch seconds.
    points[i].telemetry.sensorType = EnvironmentalSensorType::kBme280;
  }
  auto& a = points[0].telemetry;
  a.pcbVersion = 4; a.capabilities = kTemperature | kHumidity | kPressure | kIaq | kGasResistance | kBattery;
  a.temperatureC = -2.5F; a.humidityPercent = 0; a.pressureHpa = 1000.5F;
  a.iaq = 35.5F; a.iaqAccuracy = 3; a.gasResistanceOhms = 12345; a.batteryMillivolts = 3805;
  auto& b = points[1].telemetry;
  b.sensorType = EnvironmentalSensorType::kTmp117; b.capabilities = kTemperature;
  b.temperatureC = 36.123047F; b.live.flags = kLiveGap;
  auto& c = points[2].telemetry;
  c.sensorType = EnvironmentalSensorType::kLsm6dsox; c.capabilities = kMotion;
  c.motion.accelerationG[0] = -0.125F; c.motion.accelerationG[2] = 1.0F;
  c.motion.angularRateDps[0] = 2.25F; c.motion.peakAccelerationG = 1.1F;
  auto& d = points[3].telemetry;
  d.capabilities = kIaq | kBattery; d.iaqAccuracy = 0; d.batteryMillivolts = 0;
  const uint32_t mask = station::historyColumnMask(points, 4);
  size_t legacyBytes = 0, compactBytes = 0;
  for (const auto& point : points) {
    JsonDocument legacy, compact, decoded;
    station::historyPointJson(legacy, point);
    station::compactHistoryPointJson(compact, legacy, mask);
    const JsonArrayConst values = compact.as<JsonArrayConst>();
    size_t column = 0;
    for (size_t i = 0; i < station::kHistoryColumnCount; ++i) {
      if (!(mask & (1UL << i))) continue;
      const JsonVariantConst value = values[column++];
      if (!value.isNull()) decoded[station::kHistoryColumns[i]] = value;
    }
    assert(column == values.size() && !legacy.overflowed() && !compact.overflowed());
    std::string expected, actual, array;
    serializeJson(legacy, expected); serializeJson(decoded, actual); serializeJson(compact, array);
    assert(expected == actual);
    assert(decoded["t"].as<uint64_t>() == uint64_t(point.timestamp) * 1000);
    legacyBytes += expected.size(); compactBytes += array.size();
  }
  JsonDocument zeroAccuracy; station::historyPointJson(zeroAccuracy, points[3]);
  assert(zeroAccuracy["iaq"].isNull());
  assert(!zeroAccuracy["battery"].isNull() && zeroAccuracy["battery"].as<float>() == 0);
  printf("History JSON: four mixed point types round-trip exactly; objects %zu bytes, arrays %zu bytes\n", legacyBytes, compactBytes);

  // Host-only serializer comparison with one homogeneous environmental page.
  station::NormalHistoryPoint page[512]{};
  for (size_t i=0; i<512; ++i) {page[i]=points[0];page[i].timestamp+=i;}
  const uint32_t pageMask=station::historyColumnMask(page,512);
  auto measure=[&](bool arrays) {
    const auto start=std::chrono::steady_clock::now();size_t bytes=0;
    for(unsigned repeat=0;repeat<20;++repeat)for(const auto& p:page){
      JsonDocument row,small;station::historyPointJson(row,p);
      if(arrays)station::compactHistoryPointJson(small,row,pageMask);
      std::string text;serializeJson(arrays?small:row,text);bytes+=text.size();
    }
    const auto elapsed=std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-start).count();
    return std::pair<size_t,long long>{bytes,elapsed};
  };
  const auto objects=measure(false),arrays=measure(true);
  printf("Host serializer (20 x 512 points): objects %zu bytes / %lld us; arrays %zu bytes / %lld us; header excluded\n",objects.first,objects.second,arrays.first,arrays.second);
  assert(arrays.first < objects.first);
}
