#include "batteryMenager.h"

WiFiClient client;
WiFiManager wm;

void checkWifi(void *parametrs) {
  static bool firstConnect = true;  // Track initial connection
  
  for (;;) {
    // Moved WiFiManager configuration inside the loop for dynamic adjustment
    wm.setConnectTimeout(30);     // Increased connect timeout
    wm.setConfigPortalTimeout(180);

    bool res = wm.autoConnect("W-Charger");
    if (!res) {
      set_status(3);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      ESP.restart();
    }
    else {
      if(WiFi.status() == WL_CONNECTED && firstConnect) {
        firstConnect = false;  // Nur beim ersten Mal ausführen
        
        xTaskCreate(
          blinkTask,
          "LED Blink",
          2048,
          NULL,
          3,
          NULL
        );
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        set_status(0);
      } else if(WiFi.status() == WL_CONNECTED) {
        set_status(0);  // Normaler Betrieb ohne Blinken
      }
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  setCpuFrequencyMhz(80);  // Zurück zu 80MHz für stabile WiFi-Kommunikation
  btStop();
  
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  WiFi.setSleep(false);  // WiFi Sleep deaktivieren für stabile Verbindung
  WiFi.setTxPower(WIFI_POWER_19_5dBm);  // Set maximum WiFi transmit power
  
  #if defined(SERIAL_DEBUG)
  Serial.begin(9600);
  #endif

  pinMode(rotPin, OUTPUT);
  pinMode(gruenPin, OUTPUT);
  pinMode(adcPin, INPUT);
  pinMode(blauPin, OUTPUT);

  digitalWrite(rotPin, HIGH);
  digitalWrite(blauPin, HIGH);
  digitalWrite(gruenPin, HIGH);

  #if defined(RST_SETTING)
  wm.resetSettings();
  #endif
  
  wm.setConfigPortalTimeout(180);
  wm.setConnectTimeout(20);
  wm.setClass("invert");
  
  xTaskCreate(
    checkWifi,
    "check WIFI",
    4096,    // Increased stack size for WiFi task
    NULL,
    5,       // Increased priority for WiFi task
    NULL
  );

  xTaskCreate(
    batteryMenagement,
    "Battery Monitor",
    2048,
    NULL, 
    3,       // Adjusted priority for battery management task
    NULL
  );

  ThingSpeak.begin(client);
}

void loop() {
  vTaskDelete(NULL);
}