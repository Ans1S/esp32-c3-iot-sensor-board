#include "batteryMenager.h"

uint32_t readADC_Cal() {
  static esp_adc_cal_characteristics_t adc_chars;
  static bool initialized = false;
  
  if (!initialized) {
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, 1100, &adc_chars);
    initialized = true;
  }
  
  const uint8_t samples = 10;
  uint32_t sum = 0;
  
  for (uint8_t i = 0; i < samples; i++) {
    int adc_raw = analogRead(adcPin); // Read ADC value inside the loop
    sum += esp_adc_cal_raw_to_voltage(adc_raw, &adc_chars);
    vTaskDelay(10 / portTICK_PERIOD_MS); // Reduced delay
  }
  
  return sum / samples;
}

void batteryMenagement(void *parametrs) {
  const TickType_t xDelay = pdMS_TO_TICKS(15000); // Changed to 15 seconds
  static uint16_t lastVolt = 0;
  static uint8_t stableCount = 0;
  
  for (;;) {
    uint16_t newVolt = map(readADC_Cal(), 0, 2520, 0, 4200); // Updated function call
    
    // Beschleunigte Spannungsglättung
    if (abs(newVolt - volt) < 50) {
      volt = (volt + newVolt) >> 1;
    } else {
      volt = newVolt;
    }
    
    // Verbesserte Batterie-Prozent Berechnung beibehalten
    if (volt <= 3300) {
      battery_p = 0;
    } else if (volt >= 4200) {
      battery_p = 100;
    } else {
      battery_p = map(volt, 3300, 4200, 0, 100);
    }
    
    // Daten senden wenn verbunden
    if(WiFi.status() == WL_CONNECTED) {
      ThingSpeak.setField(1, volt);
      ThingSpeak.setField(2, battery_p);
      ThingSpeak.setField(3, WiFi.RSSI());
      
      int status = ThingSpeak.writeFields(CHANNEL_ID, CHANNEL_API_KEY);
      if (status == 200) {
        // Kurzes Aufblinken der grünen LED
        digitalWrite(gruenPin, LOW);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        digitalWrite(gruenPin, HIGH);
      } else {
        set_status(3);
      }
    }
    
    vTaskDelay(xDelay);
  }
}

void blinkTask(void *parameter) {
  // 3 mal blinken
  for(int i = 0; i < 6; i++) {
    digitalWrite(rotPin, HIGH);
    digitalWrite(blauPin, HIGH);
    digitalWrite(gruenPin, i % 2 == 0 ? LOW : HIGH);
    vTaskDelay(300 / portTICK_PERIOD_MS);
  }
  // Task beendet sich selbst
  vTaskDelete(NULL);
}

void set_status(int8_t s) {
  switch(s) {
    case 0: // Normaler Betrieb - LED aus
      digitalWrite(rotPin, HIGH);
      digitalWrite(blauPin, HIGH);
      digitalWrite(gruenPin, HIGH);
      break;

    case 3: // ERROR - Dauerhaft rot
      digitalWrite(rotPin, LOW);
      digitalWrite(blauPin, HIGH);
      digitalWrite(gruenPin, HIGH);
      break;

    default: // Alle aus
      digitalWrite(rotPin, HIGH);
      digitalWrite(blauPin, HIGH);
      digitalWrite(gruenPin, HIGH);
      break;
  }
}
