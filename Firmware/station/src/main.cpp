#include <Arduino.h>
#include <time.h>

#include "app_config.h"
#include "config_store.h"
#include "espnow_gateway.h"
#include "sensor_registry.h"
#include "thingspeak_service.h"
#include "web_portal.h"
#include "wifi_service.h"

namespace {
station::ConfigStore configStore;
station::StationConfig stationConfig;
station::WifiService wifiService;
station::SensorRegistry sensorRegistry;
station::ThingSpeakService thingSpeakService;
station::EspNowGateway espNowGateway;
station::WebPortal webPortal;

[[noreturn]] void fatal(const char* message) {
  Serial.printf("[FATAL] %s\n", message);
  for (;;) {
    delay(1000);
  }
}
}  // namespace

void setup() {
  Serial.begin(115200);
  const uint32_t serialStarted = millis();
  while (!Serial && millis() - serialStarted < 2000) {
    delay(10);
  }

  Serial.println();
  Serial.println("========================================");
  Serial.println("W-Charger Station firmware 3.0");
  Serial.println("========================================");

  if (!configStore.begin()) {
    fatal("NVS configuration storage could not be opened");
  }
  stationConfig = configStore.loadStationConfig();

  if (!sensorRegistry.begin(configStore,
                            stationConfig.defaultSleepSeconds)) {
    fatal("Sensor registry could not be initialized");
  }
  if (!wifiService.begin(stationConfig)) {
    fatal("Wi-Fi could not be started");
  }
  // History buckets use UTC Unix timestamps. SNTP keeps running across Wi-Fi
  // reconnects; no browser has to remain open for the station to timestamp
  // and retain measurements.
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  if (!thingSpeakService.begin()) {
    fatal("ThingSpeak task could not be started");
  }
  if (!espNowGateway.begin(sensorRegistry, thingSpeakService, wifiService)) {
    fatal("ESP-NOW gateway could not be started");
  }
  if (!webPortal.begin(stationConfig, configStore, wifiService, sensorRegistry,
                       thingSpeakService, espNowGateway)) {
    fatal("Web interface could not be started");
  }

  if (wifiService.setupPortalActive()) {
    Serial.printf("Setup Wi-Fi active: %s\n",
                  wifiService.accessPointSsid().c_str());
    Serial.println("Setup-Passwort: im lokalen Webinterface sichtbar");
    Serial.printf("Setup-Adresse: http://%s/\n",
                  wifiService.accessPointIp().c_str());
    Serial.println("Setup through the captive portal does not require a web login.");
  } else {
    Serial.println("Setup Wi-Fi disabled (home Wi-Fi is already configured).");
    Serial.println("Waiting for home Wi-Fi. If unavailable, recovery access starts automatically after 30 seconds.");
  }
}

void loop() {
  wifiService.loop();
  if (wifiService.consumeSetupPortalCompletion()) {
    stationConfig.setupPortalRequired = false;
    if (!configStore.saveStationConfig(stationConfig)) {
      Serial.println("[WARN] Setup portal state could not be saved");
    } else {
      Serial.println("Home Wi-Fi connected; setup Wi-Fi was disabled.");
    }
  }
  webPortal.loop();
  delay(2);
}
