#pragma once
#include "ota_protocol.h"
#include "sensor_config_store.h"
#include "adc_reader.h"
namespace sensor {
class EspNowTransport;
void beginOtaBootGuard();
bool otaBootPending();
void finishOtaBootGuard();
void checkOta(EspNowTransport& radio, const SensorRuntimeConfig& config,
              AdcReader& adc, uint16_t batteryMv);
}
