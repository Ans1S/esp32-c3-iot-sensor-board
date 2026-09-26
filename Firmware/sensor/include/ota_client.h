#pragma once
#include "ota_protocol.h"
#include "sensor_config_store.h"
#include "adc_reader.h"
namespace sensor {
// Confirm a trial image after a reply matched to the paired station and request,
// without requesting an update or extending a low-battery reporting window.
bool confirmOtaBootAfterContact();
class EspNowTransport;
void beginOtaBootGuard();
bool otaBootPending();
void finishOtaBootGuard();
void checkOta(EspNowTransport& radio, const SensorRuntimeConfig& config,
              AdcReader& adc, uint16_t batteryMv);
}
