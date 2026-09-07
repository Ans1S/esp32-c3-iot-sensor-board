#include "ota_test_platform.h"
#include <cstdio>
bool lil::ota::verifyManifest(const Manifest& m) { return validManifest(m) && sensor::signatureValid; }
using namespace sensor;
void reset() {
  testTime=0; readDelayMs=0;
  kHardware.pcbVersion = 4; updateBattery = {}; writeSizes.clear();
  values.clear(); states.clear(); failures.clear(); image.resize(70000);
  for(size_t i=0;i<image.size();++i) image[i]=uint8_t(i*17);
  offer={}; offer.magic=lil::ota::kPackageMagic; offer.format=1; offer.pcb=4;
  offer.protocol=5; offer.release=lil::ota::kRelease+1; offer.imageSize=image.size();
  strcpy(offer.version,"4.0.1"); offer.signatureSize=1;
  mbedtls_sha256_context hash; mbedtls_sha256_update(&hash,image.data(),image.size()); mbedtls_sha256_finish(&hash,offer.sha256);
  began=0; resumed=breakAt=UINT32_MAX; selected=aborted=failWrite=failEnd=false;
  signatureValid=offerEnabled=true; bootState=ESP_OTA_IMG_VALID; bootConfirmed=rolledBack=false;
  beginOtaBootGuard();
}
void run(uint16_t battery=4000) { EspNowTransport radio; SensorRuntimeConfig config; AdcReader adc; checkOta(radio,config,adc,battery); }
void complete() { bool restarted=false; try {run();} catch(Restart&) {restarted=true;} assert(restarted&&selected); assert(std::equal(image.begin(),image.end(),flash.begin())); }
int main() {
  reset(); complete(); assert(began==1 && !values["attempted"].empty());
  assert(writeSizes.size() == (image.size() + 4095) / 4096);
  for (size_t i = 0; i + 1 < writeSizes.size(); ++i) assert(writeSizes[i] == 4096);
  assert(writeSizes.back() == image.size() % 4096);
  printf("Buffered OTA: %zu writes for %zu bytes (previously at least %zu packet writes)\n",
         writeSizes.size(), image.size(), (image.size() + 191) / 192);
  // An interruption with a partly filled RAM sector must resume from the
  // preceding durable checkpoint, never from the acknowledged RAM tail.
  reset(); breakAt=21000; run(); assert(aborted && cursor == 20480);
  breakAt=UINT32_MAX; complete(); assert(resumed==16384);

  reset(); readDelayMs=600; complete(); assert(testTime > 180000 && testTime < 600000);
  reset(); readDelayMs=2000; run();
  assert(aborted && !selected && failures.back()==lil::ota::Failure::SessionTimeout);
  readDelayMs=0; complete(); assert(resumed > 0);
  reset(); breakAt=20000; run(); assert(aborted&&!selected);
  breakAt=UINT32_MAX; complete(); assert(resumed==16384 && began==1);
  reset(); breakAt=20000; run(); flash[10]^=1; breakAt=UINT32_MAX; complete(); assert(began==2 && resumed==UINT32_MAX);
  reset(); signatureValid=false; run(); assert(!began&&!selected && states.back()==lil::ota::State::Rejected && failures.back()==lil::ota::Failure::ManifestInvalid);
  reset(); offer.pcb=3; run(); assert(!began&&!selected);
  for (uint8_t pcb : {3, 4}) {
    const uint16_t minimum = pcb == 4 ? 2800 : 3350;
    reset(); kHardware.pcbVersion=pcb; offer.pcb=pcb;
    run(minimum - 1);
    assert(!began&&!selected && states.back()==lil::ota::State::LowBattery && failures.back()==lil::ota::Failure::LowBattery);
    reset(); kHardware.pcbVersion=pcb; offer.pcb=pcb;
    updateBattery.millivolts=minimum;
    bool restarted=false; try { run(minimum); } catch(Restart&) { restarted=true; }
    assert(restarted && selected); // Equality is accepted at start and during transfer.
    reset(); kHardware.pcbVersion=pcb; offer.pcb=pcb;
    updateBattery.millivolts=minimum - 1;
    run(minimum);
    assert(began && aborted && !selected && states.back()==lil::ota::State::LowBattery);
  }
  reset(); failWrite=true; run(); assert(aborted&&!selected && failures.back()==lil::ota::Failure::FlashWrite);
  reset(); failEnd=true; run(); assert(!selected && states.back()==lil::ota::State::Rejected && failures.back()==lil::ota::Failure::ImageValidation);
  reset(); offer.sha256[0]^=1; run(); assert(aborted&&!selected && failures.back()==lil::ota::Failure::DigestMismatch);
  reset(); bootState=ESP_OTA_IMG_PENDING_VERIFY; beginOtaBootGuard();
  try {finishOtaBootGuard();} catch(Restart&) {} assert(rolledBack&&!bootConfirmed);
  reset(); bootState=ESP_OTA_IMG_PENDING_VERIFY; beginOtaBootGuard(); offerEnabled=false;
  Preferences p; p.putUInt("attempted",lil::ota::kRelease); run(); assert(bootConfirmed&&!otaBootPending());
  reset(); bootState=ESP_OTA_IMG_PENDING_VERIFY; beginOtaBootGuard();
  run(); assert(otaBootPending() && !bootConfirmed && !began);
  try {finishOtaBootGuard();} catch(Restart&) {} assert(rolledBack);
  puts("OTA production client: transfer, resume, corrupt checkpoint, rejection, low battery, flash failure and boot guard passed");
}
