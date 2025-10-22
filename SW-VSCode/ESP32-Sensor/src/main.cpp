#include <Wire.h>
#include <Adafruit_BME280.h>
#include <esp_adc_cal.h>
#include <esp_now.h>
#include <WiFi.h>
#include "esp_pm.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"
#include "esp_wifi.h"
#include <esp_random.h>
#include "driver/gpio.h"

// ADC DMA using ESP-IDF v4 (Arduino ESP32 2.0.17)
#include "driver/adc.h"
#include "esp_adc_cal.h"

// RTC-Speicher-Signatur für Konsistenzprüfung
#define RTC_SIGNATURE 0xA53C9D7F

// Pin definitions
#define adcPin 3
#define sensorVccPin 10
#define sdaPin 5
#define sclPin 4

// Battery constants
#define BATTERY_MIN 3300
#define BATTERY_MAX 4200

// Sleep settings - deep sleep for 1 minute between transmissions
#define SEND_INTERVAL_MIN 10     // Send data every 10 minutes
#define SEND_INTERVAL_SEC (SEND_INTERVAL_MIN * 60) 
#define SEND_INTERVAL_US (SEND_INTERVAL_SEC * 1000000ULL) // Convert to microseconds

// Radio configuration
#define WIFI_CHANNEL 1
// TX power in quarter-dBm steps (e.g., 84 = 20 dBm). Adjust if you need more/less range.
#define ESP_NOW_TX_POWER_QDBM 80

// Send timing
#define SEND_TIMEOUT_MS 200
#define RETRY_BACKOFF_BASE_MS 100

// ADC sampling constants
#define ADC_MAX_SAMPLES 16
#define ADC_SAMPLE_DELAY_INTERVAL 8
#define ADC_SAMPLE_DELAY_US 50
#define ADC_DEFAULT_MIN_SAMPLES 16

// Battery voltage scaling
#define VOLTAGE_DIVIDER_NUMERATOR 167
#define VOLTAGE_DIVIDER_DENOMINATOR 100

// Debug mode (only for development, disable for production)
//#define DEBUG_MODE
//#define SKIP_DEEP_SLEEP  // Uncomment to disable deep sleep in debug mode

#ifdef DEBUG_MODE
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

// Structure to send data - updated to match station's expected format
typedef struct sensor_data {
  uint8_t sensorId;     // Identifier for this sensor
  float temperature;    // Temperature in Celsius
  float humidity;       // Humidity in %
  float iaq;            // Air Quality Index (for sensors 3+, not used for regular sensors)
  int voltage;          // Battery voltage in mV
  uint8_t battery_p;    // Battery percentage
} sensor_data;

// Create a structured object
sensor_data sensorData;

// MAC Address of the receiver (ESP32-S3 Station)
uint8_t receiverAddress[] = {0x24, 0x58, 0x7C, 0xE4, 0x13, 0xB8}; // ESP32-S3 Station MAC

// ESP-NOW variables
esp_now_peer_info_t peerInfo;
bool sendSuccess = false;

// BME280 sensor
Adafruit_BME280 bme;

// ADC calibration
esp_adc_cal_characteristics_t adc_chars;

// This is a unique ID for each sensor - change for each device
const uint8_t SENSOR_ID = 1;  // Change for each sensor (1 = Schlafzimmer, 2 = Wohnzimmer, etc.)

// Store data in RTC memory to persist across sleep cycles
RTC_DATA_ATTR uint32_t rtcSignature = 0;        // Für Konsistenzprüfung
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int transmissionFailures = 0;
RTC_DATA_ATTR uint32_t lastRandomJitter = 0;
// Precomputed, persisted values to avoid recomputing each cycle
RTC_DATA_ATTR uint64_t idOffsetUs = 0;

// ESP-IDF v4 ADC DMA implementation
static bool initAdcDma() {
  // Configure ADC1 for continuous mode
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_12);
  return true;
}

static int readBatteryMvDma(uint16_t min_samples, uint16_t max_samples) {
  // Static buffer for samples - no dynamic allocation
  static uint16_t raws[ADC_MAX_SAMPLES];
  const int MAX_SAMP = (max_samples > ADC_MAX_SAMPLES) ? ADC_MAX_SAMPLES : max_samples; // Limit to buffer size
  
  // Take rapid samples up to max_samples
  for (int i = 0; i < MAX_SAMP; i++) {
    raws[i] = adc1_get_raw(ADC1_CHANNEL_3);
    if (i % ADC_SAMPLE_DELAY_INTERVAL == 0) delayMicroseconds(ADC_SAMPLE_DELAY_US);
  }
  
  // Quality check: ensure we have minimum required samples
  if (MAX_SAMP < (int)min_samples) {
    return -1;
  }
  
  // Simple averaging of all samples (replaces median calculation)
  uint32_t sum = 0;
  for (int i = 0; i < MAX_SAMP; i++) {
    sum += raws[i];
  }
  uint16_t average = sum / MAX_SAMP;
  
  uint32_t mv = esp_adc_cal_raw_to_voltage(average, &adc_chars);
  return (int)mv;
}

// Callback function for ESP-NOW send operation
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  sendSuccess = (status == ESP_NOW_SEND_SUCCESS);
  
  #ifdef DEBUG_MODE
    DEBUG_PRINT("Send Status: ");
    DEBUG_PRINTLN(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
  #endif
}

// Simple delay function - removed problematic light sleep that wastes power
void energySaveDelay(uint32_t ms) {
  delay(ms);
}

void enterDeepSleep() {
  
  #if defined(DEBUG_MODE) && defined(SKIP_DEEP_SLEEP)
    DEBUG_PRINTLN("DEBUG MODE: Skipping deep sleep");
    DEBUG_PRINTLN("Waiting 15 seconds before restarting...");
    
    // Simulate shutdown
    digitalWrite(sensorVccPin, LOW);
    Wire.end();
    
    // Wait instead of sleep
    for (int i = 15; i > 0; i--) {
      DEBUG_PRINT(i);
      DEBUG_PRINT("... ");
      energySaveDelay(1000);
    }
    DEBUG_PRINTLN("Restarting!");
    
    // Reboot instead of sleep
    esp_restart();
    return;
  #endif

  DEBUG_PRINTLN("Preparing for deep sleep...");
  
  // Simplified sleep calculation
  uint64_t sleepTime = SEND_INTERVAL_US;

  // Larger random jitter (±15 seconds) for better collision avoidance
  int32_t randomJitter = (esp_random() % 30000 - 15000) * 1000ULL;
  lastRandomJitter = (uint32_t)(randomJitter / 1000ULL);
  
  // Simple linear backoff for failures (3 seconds per failure, max 30 seconds)
  uint64_t backoffTime = 0;
  if (transmissionFailures > 0) {
    backoffTime = min(transmissionFailures * 3000ULL, 30000ULL) * 1000ULL;
    DEBUG_PRINT("Backoff time (sec): ");
    DEBUG_PRINTLN(backoffTime / 1000000ULL);
  }
  
  // Apply all offsets (use precomputed ID offset)
  sleepTime += idOffsetUs + randomJitter + backoffTime;
  
  // Ensure minimum and maximum sleep times
  const uint64_t MIN_SLEEP = 30 * 1000 * 1000ULL;     // 30 seconds minimum
  const uint64_t MAX_SLEEP = 20 * 60 * 1000 * 1000ULL; // 20 minutes maximum
  
  sleepTime = max(sleepTime, MIN_SLEEP);
  sleepTime = min(sleepTime, MAX_SLEEP);
  
  #ifdef DEBUG_MODE
  DEBUG_PRINT("ID offset (sec): ");
  DEBUG_PRINTLN(idOffsetUs / 1000000ULL);
    DEBUG_PRINT("Jitter (sec): ");
    DEBUG_PRINTLN(randomJitter / 1000000.0);
    DEBUG_PRINT("Total sleep time (min): ");
    DEBUG_PRINTLN(sleepTime / 60000000.0);
    energySaveDelay(50); // Reduced debug finish delay
  #endif
  
  // Clean shutdown of peripherals - improved radio shutdown
  esp_now_deinit();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();
  esp_wifi_deinit();  // Complete WiFi shutdown
  
  // Turn off sensor and I2C
  digitalWrite(sensorVccPin, LOW);
  Wire.end();
  
  // ADC power management - only release if needed
  // Note: adc_power_release() can cause crashes if called improperly
  // The ESP-IDF handles ADC power automatically during deep sleep

  // Configure for deep sleep with minimal power
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF);

  // Disable GPIO hold to save power - sensor will be re-initialized on wake
  gpio_hold_dis((gpio_num_t)sensorVccPin);
  gpio_deep_sleep_hold_dis();
  
  // Enable timer wakeup
  esp_sleep_enable_timer_wakeup(sleepTime);
  
  // Enter deep sleep
  esp_deep_sleep_start();
}

void readSensorData() {
  // Force BME280 to take a reading (uses less power than continuous mode)
  bme.takeForcedMeasurement();
  delay(10); // Allow time for measurement to complete
  
  // Read temperature and humidity
  sensorData.temperature = bme.readTemperature();
  sensorData.humidity = bme.readHumidity();
  
  // Set IAQ to 0 for regular sensors (only used by air quality sensors)
  sensorData.iaq = 0.0;
  
  #ifdef DEBUG_MODE
    DEBUG_PRINT("Temperature: ");
    DEBUG_PRINTLN(sensorData.temperature);
    DEBUG_PRINT("Humidity: ");
    DEBUG_PRINTLN(sensorData.humidity);
  #endif
  
  // Note: sensorVccPin will be turned off later in the main sequence
}

void readBatteryStatus() {
  // Read battery voltage using DMA only - no fallback
  int mv_in = readBatteryMvDma(ADC_DEFAULT_MIN_SAMPLES /*min samples*/, ADC_MAX_SAMPLES /*max samples*/);
  
  if (mv_in < 0) {
    DEBUG_PRINTLN("ADC DMA failed - using default voltage");
    sensorData.voltage = 3700; // Default fallback voltage
    sensorData.battery_p = 0;   // Set to 0 since percentage is not calculated
    return;
  }

  // Scale by voltage divider (predefined factor)
  // Note: Only API-based calibration is used (esp_adc_cal_*), no manual calibration.
  const int scaled_mv = (mv_in * VOLTAGE_DIVIDER_NUMERATOR) / VOLTAGE_DIVIDER_DENOMINATOR; // 1.67x
  sensorData.voltage = scaled_mv;

  // Battery percentage is not calculated anymore - set to 0
  sensorData.battery_p = 0;
  
  #ifdef DEBUG_MODE
    DEBUG_PRINT("Battery Voltage: ");
    DEBUG_PRINTLN(sensorData.voltage);
    DEBUG_PRINTLN("Battery percentage calculation disabled");
  #endif
}

void initWiFiAndESPNow() {
  DEBUG_PRINTLN("Starting WiFi and ESP-NOW initialization...");
  
  // Initialize WiFi in a more reliable way
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  
  // Small delay to ensure WiFi is ready - use energy-saving delay
  energySaveDelay(50);
  
  #ifdef DEBUG_MODE
    DEBUG_PRINT("MAC Address: ");
    DEBUG_PRINTLN(WiFi.macAddress());
  #endif

  // Lock channel and set TX power before ESP-NOW to avoid scans and reduce on-air time
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_max_tx_power(ESP_NOW_TX_POWER_QDBM);
  
  // Initialize ESP-NOW with error handling and multiple retries
  esp_err_t result = ESP_FAIL;
  for (int attempts = 0; attempts < 3; attempts++) {
    result = esp_now_init();
    if (result == ESP_OK) {
      DEBUG_PRINTLN("ESP-NOW init successful");
      break;
    }
    DEBUG_PRINT("ESP-NOW init failed, attempt ");
    DEBUG_PRINT(attempts + 1);
    DEBUG_PRINT("/3, error: ");
    DEBUG_PRINTLN(result);
    
    if (attempts < 2) {
      // Clean up before retry
      esp_now_deinit();
      energySaveDelay(50);
    }
  }
  
  if (result != ESP_OK) {
    DEBUG_PRINTLN("ESP-NOW init failed after 3 attempts, going to sleep");
    enterDeepSleep();
    return;
  }
  
  // Register callback
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer with explicit channel
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = WIFI_CHANNEL;  // Use explicit channel
  peerInfo.encrypt = false;
  
  // Add peer with error handling and retries
  for (int attempts = 0; attempts < 3; attempts++) {
    result = esp_now_add_peer(&peerInfo);
    if (result == ESP_OK) {
      DEBUG_PRINTLN("Peer added successfully");
      break;
    }
    DEBUG_PRINT("Failed to add peer, attempt ");
    DEBUG_PRINT(attempts + 1);
    DEBUG_PRINT("/3, error: ");
    DEBUG_PRINTLN(result);
    
    if (attempts < 2) {
      energySaveDelay(30);
    }
  }
  
  if (result != ESP_OK) {
    DEBUG_PRINTLN("Failed to add peer after 3 attempts, going to sleep");
    enterDeepSleep();
    return;
  }
  
  DEBUG_PRINTLN("ESP-NOW initialized successfully");
}

void setup() {
  // Initialize system and peripherals only - routine moved to loop()
  
  // Complete WiFi/Bluetooth shutdown at startup
  btStop();                 
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();
  esp_wifi_deinit();

  // Check RTC memory consistency
  if (rtcSignature != RTC_SIGNATURE) {
    // RTC memory was reset
    rtcSignature = RTC_SIGNATURE;
    bootCount = 0;
    transmissionFailures = 0;
    lastRandomJitter = 0;
    idOffsetUs = 0;
  }
  
  // Increment boot counter
  bootCount++;
  
  // Disable any GPIO holds from previous deep sleep
  gpio_deep_sleep_hold_dis();
  gpio_hold_dis((gpio_num_t)sensorVccPin);
  
  #ifdef DEBUG_MODE
    Serial.begin(115200);
    energySaveDelay(50); // Reduced debug delay
    Serial.println("\n\nESP32-C3 Sensor Node Starting");
    Serial.printf("Boot count: %d\n", bootCount);
    Serial.printf("Sensor ID: %d\n", SENSOR_ID);
    Serial.printf("RTC memory: %s\n", rtcSignature == RTC_SIGNATURE ? "Valid" : "Reset detected");
    Serial.printf("Failures: %d\n", transmissionFailures);
  #endif
  
  // Configure pins
  pinMode(sensorVccPin, OUTPUT);
  pinMode(adcPin, INPUT);

  // Power up the sensor
  digitalWrite(sensorVccPin, HIGH);
  
  // Initialize I2C with faster clock for shorter active time  
  Wire.begin(sdaPin, sclPin);
  Wire.setClock(100000);
  
  // Initialize BME280 with up to 3 retries before deep sleep
  bool sensorInitialized = false;
  for (int attempts = 0; attempts < 3; attempts++) {
    if (bme.begin(0x76)) {
      sensorInitialized = true;
      break;
    }
    DEBUG_PRINT("BME280 init failed, retry ");
    DEBUG_PRINTLN(attempts + 1);
    energySaveDelay(30); // Reduced retry delay
  }
  if (!sensorInitialized) {
    DEBUG_PRINTLN("Could not find BME280 sensor after 3 attempts, going to deep sleep");
    enterDeepSleep();
    return;
  }
  
  // Configure BME280 for optimal power consumption
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1,      // Temperature oversampling (high precision)
                  Adafruit_BME280::SAMPLING_NONE,    // Pressure disabled for power saving
                  Adafruit_BME280::SAMPLING_X1,      // Humidity oversampling (high precision)
                  Adafruit_BME280::FILTER_OFF,       // No filter for faster readings
                  Adafruit_BME280::STANDBY_MS_1000); // Standby time (not used in forced mode)
  
  // Initialize ADC for battery monitoring
  {
    esp_adc_cal_value_t cal_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, 1100, &adc_chars);
    #ifdef DEBUG_MODE
      if (cal_type == ESP_ADC_CAL_VAL_EFUSE_TP) DEBUG_PRINTLN("ADC cal: eFuse Two Point");
      else if (cal_type == ESP_ADC_CAL_VAL_EFUSE_VREF) DEBUG_PRINTLN("ADC cal: eFuse Vref");
      else DEBUG_PRINTLN("ADC cal: Default Vref");
    #endif
  }
  analogSetPinAttenuation(adcPin, ADC_11db);

  // Try ADC init up to 3 times before deep sleep
  bool adcInitOk = false;
  for (int attempts = 0; attempts < 3; attempts++) {
    if (initAdcDma()) {
      adcInitOk = true;
      break;
    }
    DEBUG_PRINT("ADC init failed, retry ");
    DEBUG_PRINTLN(attempts + 1);
    energySaveDelay(30);
  }
  if (!adcInitOk) {
    #ifdef DEBUG_MODE
      DEBUG_PRINTLN("ADC init failed after 3 attempts - critical error");
    #endif
    enterDeepSleep();
    return;
  } else {
    #ifdef DEBUG_MODE
      DEBUG_PRINTLN("ADC initialized");
    #endif
  }

  // Precompute static offsets once
  if (idOffsetUs == 0) {
    idOffsetUs = (uint64_t)(SENSOR_ID > 0 ? (SENSOR_ID - 1) : 0) * 15000ULL * 1000ULL;
    #ifdef DEBUG_MODE
      DEBUG_PRINT("Precomputed ID offset (sec): ");
      DEBUG_PRINTLN(idOffsetUs / 1000000ULL);
    #endif
  }
  
  // Initialize the sensor data structure
  sensorData.sensorId = SENSOR_ID;
  
  DEBUG_PRINTLN("Initialization complete - starting main routine");
}

// Empty loop() function required by Arduino framework (never called due to deep sleep)
void loop() {
  // ========== MAIN EXECUTION SEQUENCE ==========
  // Complete sensor routine: Read Sensors -> Send Data -> Sleep
  
  // 1. Read sensor data
  DEBUG_PRINTLN("Reading sensor data...");
  readSensorData();
  
  // 2. Read battery status
  DEBUG_PRINTLN("Reading battery status...");
  readBatteryStatus();
  
  // 2.5. Turn off sensor power after all readings are complete
  digitalWrite(sensorVccPin, LOW);
  DEBUG_PRINTLN("Sensor power turned off");
  
  // 3. Initialize WiFi and ESP-NOW fresh each time
  DEBUG_PRINTLN("Initializing WiFi and ESP-NOW...");
  initWiFiAndESPNow();
  
  // 4. Send data with retry mechanism
  DEBUG_PRINTLN("Sending data...");
  bool sendAttemptSuccessful = false;
  
  for (int attempts = 0; attempts < 3 && !sendAttemptSuccessful; attempts++) {
    if (attempts > 0) {
      DEBUG_PRINT("Retry attempt ");
      DEBUG_PRINTLN(attempts);
      // Use energy-saving delay for retry backoffs (100ms+)
      energySaveDelay(RETRY_BACKOFF_BASE_MS * attempts);
    }
    
    // Send data
    esp_err_t result = esp_now_send(receiverAddress, (uint8_t *)&sensorData, sizeof(sensorData));
    
    if (result != ESP_OK) {
      DEBUG_PRINTLN("Send initiation failed");
      continue;
    }
    
    // Wait for callback with optimized timeout
    sendSuccess = false;
    unsigned long startTime = millis();
    while (!sendSuccess && (millis() - startTime < SEND_TIMEOUT_MS)) {
      energySaveDelay(2); // Reduced polling delay for faster response
    }
    
    if (sendSuccess) {
      sendAttemptSuccessful = true;
      DEBUG_PRINTLN("Transmission successful!");
      DEBUG_PRINT("Sent - Temp: ");
      DEBUG_PRINT(sensorData.temperature);
      DEBUG_PRINT("°C, Humidity: ");
      DEBUG_PRINT(sensorData.humidity);
      DEBUG_PRINT("%, Battery: ");
      DEBUG_PRINT(sensorData.voltage);
      DEBUG_PRINTLN("mV");
    } else {
      DEBUG_PRINTLN("Transmission failed or timed out");
    }
  }  
  
  // 5. Update transmission success/failure counters
  if (sendAttemptSuccessful) {
    transmissionFailures = 0;
  } else {
    transmissionFailures = min(transmissionFailures + 1, 10);
    DEBUG_PRINT("Total transmission failures: ");
    DEBUG_PRINTLN(transmissionFailures);
  }
  
  // 6. Ensure all serial output is sent (minimal delay in debug)
  #ifdef DEBUG_MODE
    Serial.flush();
    energySaveDelay(50); // Use energy-saving delay for debug output
  #endif
  
  // 7. Enter deep sleep until the next interval
  DEBUG_PRINTLN("Entering deep sleep...");
  enterDeepSleep();
}