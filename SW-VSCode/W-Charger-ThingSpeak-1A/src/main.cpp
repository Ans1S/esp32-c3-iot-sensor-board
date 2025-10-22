#include <Arduino.h>
#include <WiFi.h>
#include <ThingSpeak.h>
#include "esp_adc_cal.h"
#include "config.h"  // Include configuration file with WiFi and ThingSpeak credentials

WiFiClient client;
TaskHandle_t deepSleep = NULL;
TaskHandle_t sended = NULL;

int volt = 0;
int battery_p = 0;

void checkWifi(void * parametrs){
  for(;;){
    if(WiFi.status() == WL_CONNECTED){
      Serial.println("Wifi still connected!");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }
    if(WiFi.status() != WL_CONNECTED && deepSleep != NULL){
        vTaskSuspend(deepSleep);
    }

    Serial.println("WiFi Connecting");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PSWD);

    unsigned long startAttemptTime = millis();

    // Keep looping while we're not connected and haven't r
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT_MS) {

        // When we couldn't make a WiFi connection
      if(WiFi.status() != WL_CONNECTED){  
      Serial.println("[WIFI] FAILED");
      vTaskDelay(500 / portTICK_PERIOD_MS);
      continue;
      }
    Serial.println("[WIFI] Connected:"+WiFi.localIP());
    }

  }
}

uint32_t readADC_Cal(int ADC_Raw){
  esp_adc_cal_characteristics_t adc_chars;
  
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  return(esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}

void batteryMenagement(void * parametrs){
  for(;;){
    Serial.println(volt);
    Serial.println(battery_p);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    //Batterie Spannung und Prozent ausrechnen
    volt = map(readADC_Cal(analogRead(adcPin)),0, 2520, 0, 4200);
    battery_p = map(volt, 3300, 4200, 0, 100);
    ThingSpeak.setField(1, volt);
    ThingSpeak.setField(2, battery_p);
    ThingSpeak.setField(3, WiFi.RSSI());

    if(ThingSpeak.writeFields(CHANNEL_ID, CHANNEL_API_KEY) == 200 && deepSleep != NULL){
      vTaskResume(deepSleep);
    }
  }
}

void DeepSleepMenager(void * parametrs){
  for(;;){
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    esp_sleep_enable_timer_wakeup(10000000);
    esp_deep_sleep_start();
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(rotPin, OUTPUT);  //rotPin
  pinMode(gruenPin, OUTPUT);
  pinMode(adcPin, INPUT);

  digitalWrite(rotPin, HIGH);
  digitalWrite(blauPin, HIGH);
  digitalWrite(gruenPin, HIGH);

  xTaskCreate(
    checkWifi,
    "check WIFI",
    2082,
    NULL,
    1,
    NULL
  );

  xTaskCreate(
    batteryMenagement,
    "ADC Value",
    2084,
    NULL,
    2,
    NULL
  );

  xTaskCreate(
    DeepSleepMenager,
    "Enabeling DeepSleep",
    2084,
    NULL,
    3,
    &deepSleep
  );

  ThingSpeak.begin(client);

}

void loop() {
  // put your main code here, to run repeatedly:
}