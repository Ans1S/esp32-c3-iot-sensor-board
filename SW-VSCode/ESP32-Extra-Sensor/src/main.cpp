#include <Wire.h>
#include <Arduino.h>
#include "bme68x.h"
#include <esp_adc_cal.h>
#include <esp_now.h>
#include <WiFi.h>
#include "esp_pm.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"
#include "esp_wifi.h"
#include <esp_random.h>
#include <math.h> // Added for pow() and exp()
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ADC DMA using ESP-IDF v4 (Arduino ESP32 2.0.17)
#include "driver/adc.h"
#include "esp_adc_cal.h"

// Simple delay function - removed problematic light sleep that wastes power
void energySaveDelay(uint32_t ms) {
  delay(ms);
}

// RTC-Speicher-Signatur für Konsistenzprüfung
#define RTC_SIGNATURE 0xA53C9D7F

// Pin definitions
#define adcPin 3
#define sensorVccPin 10
#define sdaPin 5
#define sclPin 4
#define BME68X_I2C_ADDR 0x77

// Battery constants
#define BATTERY_MIN 3300
#define BATTERY_MAX 4200

// Sleep settings - deep sleep for 10 minutes between transmissions
#define SEND_INTERVAL_MIN 10     // Send data every 10 minutes
#define SEND_INTERVAL_SEC (SEND_INTERVAL_MIN * 60) 
#define SEND_INTERVAL_US (SEND_INTERVAL_SEC * 1000000ULL)

// Radio configuration
#define WIFI_CHANNEL 1
// TX power in quarter-dBm steps (e.g., 44 = 11 dBm). Adjust if you need more/less range.
#define ESP_NOW_TX_POWER_QDBM 60

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
//#define SKIP_DEEP_SLEEP

// Force professional multi-temperature measurement mode in production
#define FORCE_PROFESSIONAL_MODE

// Enable continuous sensor reading every second in debug mode
//#define CONTINUOUS_READING_MODE

#ifdef DEBUG_MODE
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINTF(x, y) Serial.print(x, y)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(x, y)
#endif

// Structure to send data - matching ThingSpeak format
typedef struct sensor_data {
  uint8_t sensorId;     // Identifier for this sensor
  float temperature;    // Not used for extra sensors, but needed for struct compatibility
  float humidity;       // Not used for extra sensors, but needed for struct compatibility
  float iaq;            // Air Quality Index (matches ThingSpeak)
  int voltage;          // Battery voltage in mV (matches ThingSpeak)
  uint8_t battery_p;    // Not used for extra sensors, but needed for struct compatibility
} sensor_data;

// Create a structured object
sensor_data sensorData;

// MAC Address of the receiver (ESP32-S3 Station)
uint8_t receiverAddress[] = {0x24, 0x58, 0x7C, 0xE4, 0x13, 0xB8}; // ESP32-S3 Station MAC

// ESP-NOW variables
esp_now_peer_info_t peerInfo;
bool sendSuccess = false;

// ADC DMA handle (removed since we're using ESP-IDF v4)

// BME68x sensor structures
struct bme68x_dev bme;
struct bme68x_data data;
struct bme68x_conf conf;
struct bme68x_heatr_conf heatr_conf;

// ADC calibration
esp_adc_cal_characteristics_t adc_chars;

// This is a unique ID for each sensor - change for each device
const uint8_t SENSOR_ID = 3;  // Change for each extra sensor (3 = Extra-Sensor 1, 4 = Extra-Sensor 2, etc.)

// Optimized RTC data structure (30 bytes total for multi-temp support)
typedef struct {
    uint32_t signature;           // 4 bytes
    uint16_t boot_count;          // 2 bytes (65k boots sufficient)
    uint8_t failures;             // 1 byte (max 255)
    uint8_t flags;                // 1 byte (bit flags: warmed_up, baseline_established, baseline_calibrated)
    float gas_baseline;           // 4 bytes
    float gas_history[3];         // 12 bytes (reduced from 10 to 3 samples)
    float voc_baselines[3];       // 12 bytes (baseline for each temperature profile)
    uint8_t voc_detection_count;  // 1 byte (counter for VOC detections)
    uint8_t reserved;             // 1 byte reserved for future use (reduced from 3)
} rtc_data_t;

RTC_DATA_ATTR rtc_data_t rtc_data = {0};

// Bit flag definitions for rtc_data.flags
#define FLAG_WARMED_UP        0x01
#define FLAG_BASELINE_EST     0x02
#define FLAG_BASELINE_CAL     0x04
#define FLAG_MULTI_TEMP_MODE  0x08

// Helper functions for RTC data access
#define sensorWarmedUp        (rtc_data.flags & FLAG_WARMED_UP)
#define baselineEstablished   (rtc_data.flags & FLAG_BASELINE_EST)
#define multiTempModeActive   (rtc_data.flags & FLAG_MULTI_TEMP_MODE)
#define bootCount             rtc_data.boot_count
#define transmissionFailures  rtc_data.failures
#define gasBaseline           rtc_data.gas_baseline

// Helper functions to set/clear flags
inline void setSensorWarmedUp(bool value) { 
    if (value) rtc_data.flags |= FLAG_WARMED_UP; 
    else rtc_data.flags &= ~FLAG_WARMED_UP; 
}
inline void setBaselineEstablished(bool value) { 
    if (value) rtc_data.flags |= FLAG_BASELINE_EST; 
    else rtc_data.flags &= ~FLAG_BASELINE_EST; 
}
inline void setBaselineCalibrated(bool value) { 
    if (value) rtc_data.flags |= FLAG_BASELINE_CAL; 
    else rtc_data.flags &= ~FLAG_BASELINE_CAL; 
}
inline bool isBaselineCalibrated() { 
    return rtc_data.flags & FLAG_BASELINE_CAL; 
}
inline void setMultiTempMode(bool value) { 
    if (value) rtc_data.flags |= FLAG_MULTI_TEMP_MODE; 
    else rtc_data.flags &= ~FLAG_MULTI_TEMP_MODE; 
}

// Konstanten for optimized sensor operation
const int MIN_BOOTS_FOR_BASELINE = 10; // Reduced from 20 boots (~2.5h)

// Multi-Temperature VOC Detection Configuration
const bool ENABLE_MULTI_TEMP_SENSING = true; // Enable professional multi-temp sensing
const int VOC_TEMP_COUNT = 3; // Number of different heater temperatures

// VOC temperature profiles for different compound groups
typedef struct {
    uint16_t temperature;    // Heater temperature in °C
    uint16_t duration;       // Heating duration in ms
    const char* voc_group;   // Description of VOC group detected
    float energy_cost;       // Relative energy cost (1.0 = baseline)
} voc_temp_profile_t;

// Professional VOC detection profiles - optimized for battery operation
const voc_temp_profile_t voc_profiles[VOC_TEMP_COUNT] = {
    {280, 80,  "Light VOCs/Alcohols",     0.8f},  // Low energy for alcohols, ketones
    {320, 100, "Standard VOCs/Aromatics", 1.0f},  // Standard for benzene, toluene
    {370, 60,  "Heavy Hydrocarbons",      1.3f}   // Higher temp but shorter duration
};

// Adaptive multi-temp sensing based on conditions
bool shouldUseMultiTempSensing() {
    #ifdef FORCE_PROFESSIONAL_MODE
        // Force professional mode regardless of other conditions
        return ENABLE_MULTI_TEMP_SENSING;
    #endif
    
    #ifdef DEBUG_MODE
        // In debug mode: ALWAYS use professional multi-temp sensing for testing
        return ENABLE_MULTI_TEMP_SENSING;
    #endif
    
    // Production mode: Use multi-temp sensing when:
    // 1. Sensor is well warmed up (stable baseline)
    // 2. Battery level is adequate (>30%) - checked later after battery reading
    // 3. Not in failure recovery mode
    // 4. Every 3rd measurement cycle (energy conservation)
    
    bool stable_baseline = baselineEstablished && (bootCount > MIN_BOOTS_FOR_BASELINE);
    bool low_failures = (transmissionFailures < 3);
    bool cycle_match = ((bootCount % 3) == 0); // Every 3rd cycle
    
    // Note: Battery check is performed later in readSensorData() after readBatteryStatus()
    return ENABLE_MULTI_TEMP_SENSING && stable_baseline && low_failures && cycle_match;
}

// Enhanced IAQ calculation with multi-temperature VOC analysis
float calculateAdvancedIAQ(float gas_resistance, float humidity, float temperature, 
                          float voc_results[VOC_TEMP_COUNT]) {
    // Input validation
    if (gas_resistance <= 0 || gasBaseline <= 0) {
        DEBUG_PRINTLN("⚠️ calculateAdvancedIAQ: Invalid gas resistance or baseline");
        return -1.0f;
    }
    
    if (humidity < 0 || humidity > 100) {
        DEBUG_PRINTLN("⚠️ calculateAdvancedIAQ: Invalid humidity value");
        return -1.0f;
    }
    
    if (temperature < -40 || temperature > 85) {
        DEBUG_PRINTLN("⚠️ calculateAdvancedIAQ: Invalid temperature value");
        return -1.0f;
    }
    
    // Standard gas score calculation
    float gas_ratio = gas_resistance / gasBaseline;
    float gas_score = 0.0f;
    
    if (gas_ratio >= 1.0f) {
        gas_score = 100.0f;
    } else {
        gas_score = 100.0f * log10(gas_ratio + 0.1f) / log10(1.1f);
        gas_score = constrain(gas_score, 0.0f, 100.0f);
    }
    
    // Multi-temperature VOC enhancement
    float voc_enhancement = 0.0f;
    float total_voc_impact = 0.0f;
    
    if (multiTempModeActive && voc_results != nullptr) {
        int valid_voc_readings = 0;
        
        for (int i = 0; i < VOC_TEMP_COUNT; i++) {
            if (voc_results[i] > 0 && rtc_data.voc_baselines[i] > 0) {
                float voc_ratio = voc_results[i] / rtc_data.voc_baselines[i];
                
                // Different weights for different VOC groups
                float weight = (i == 0) ? 0.3f : (i == 1) ? 0.5f : 0.2f; // Emphasize aromatics
                
                if (voc_ratio < 0.8f) // Significant pollution detected
                    {
                    float voc_pollution = (0.8f - voc_ratio) * 100.0f * weight;
                    total_voc_impact += voc_pollution;
                }
                valid_voc_readings++;
            }
        }
        
        // Only apply VOC enhancement if we have at least 2 valid readings
        if (valid_voc_readings >= 2) {
            voc_enhancement = constrain(total_voc_impact, 0.0f, 50.0f);
            gas_score = max(0.0f, gas_score - voc_enhancement);
            
            DEBUG_PRINT("VOC enhancement applied: "); DEBUG_PRINTF(voc_enhancement, 1);
            DEBUG_PRINT(" (from "); DEBUG_PRINT(valid_voc_readings); DEBUG_PRINTLN(" readings)");
        } else {
            DEBUG_PRINTLN("⚠️ Insufficient valid VOC readings for enhancement");
        }
    }
    
    // Comfort score (unchanged)
    float hum_score = 100.0f;
    if (humidity < 30.0f || humidity > 70.0f) {
        float hum_deviation = min(abs(humidity - 30.0f), abs(humidity - 70.0f));
        hum_score = max(0.0f, 100.0f - (hum_deviation * 2.0f));
    } else if (humidity >= 40.0f && humidity <= 60.0f) {
        hum_score = 100.0f;
    } else {
        hum_score = 90.0f;
    }
    
    float temp_score = 100.0f;
    if (temperature < 16.0f || temperature > 28.0f) {
        float temp_deviation = min(abs(temperature - 16.0f), abs(temperature - 28.0f));
        temp_score = max(0.0f, 100.0f - (temp_deviation * 3.0f));
    } else if (temperature >= 20.0f && temperature <= 25.0f) {
        temp_score = 100.0f;
    } else {
        temp_score = 90.0f;
    }
    
    float comfort_score = (hum_score + temp_score) / 2.0f;
    
    // Weighted combination with VOC enhancement
    float weight_gas = multiTempModeActive ? 0.80f : 0.75f; // More emphasis on gas when using multi-temp
    float weight_comfort = 1.0f - weight_gas;
    float combined_score = (gas_score * weight_gas) + (comfort_score * weight_comfort);
    
    // Convert to IAQ scale with bounds checking
    float iaq = (100.0f - combined_score) * 5.0f;
    iaq = constrain(iaq, 0.0f, 500.0f);
    
    // Additional sanity check
    if (isnan(iaq) || isinf(iaq)) {
        DEBUG_PRINTLN("⚠️ IAQ calculation resulted in invalid value");
        return -1.0f;
    }
    
    return iaq;
}

// Simplified IAQ calculation (fallback for single-temperature mode)
float calculateIAQ(float gas_resistance, float humidity, float temperature) {
    return calculateAdvancedIAQ(gas_resistance, humidity, temperature, nullptr);
}

// Simplified baseline update using reduced gas history
void updateGasBaseline(float gas_resistance) {
    // Add to simplified gas history (3 samples)
    static uint8_t history_index = 0;
    rtc_data.gas_history[history_index] = gas_resistance;
    history_index = (history_index + 1) % 3;

    float sum = 0.0f;
    int valid_samples = 0;
    for (int i = 0; i < 3; i++) {
        if (rtc_data.gas_history[i] > 0) {
            sum += rtc_data.gas_history[i];
            valid_samples++;
        }
    }

    if (valid_samples >= 2) {
        float mean = sum / valid_samples;

        if (!baselineEstablished || rtc_data.gas_baseline < 1000.0f || rtc_data.gas_baseline > 2000000.0f) {
            // Nur beim ersten Mal oder bei Ausreißern initialisieren
            rtc_data.gas_baseline = mean * 1.15f;
            setBaselineEstablished(true);
            DEBUG_PRINTLN("📏 Gas baseline calibrated");
        } else {
            // Langsame Anpassung NUR nach oben (wie Bosch BSEC)
            if (mean > rtc_data.gas_baseline) {
                rtc_data.gas_baseline = rtc_data.gas_baseline * 0.995f + mean * 0.005f;
            }
            // Nach unten KEINE Anpassung!
        }
    }
}

// Multi-temperature VOC baseline management
void updateVocBaselines(float voc_results[VOC_TEMP_COUNT]) {
    if (!multiTempModeActive || voc_results == nullptr) return;
    
    for (int i = 0; i < VOC_TEMP_COUNT; i++) {
        if (voc_results[i] > 0) {
            if (rtc_data.voc_baselines[i] <= 0) {
                // Initialize baseline
                rtc_data.voc_baselines[i] = voc_results[i] * 1.1f; // 10% above current
                DEBUG_PRINT("📏 VOC "); DEBUG_PRINT(i); 
                DEBUG_PRINT(" baseline: "); DEBUG_PRINTF(rtc_data.voc_baselines[i]/1000, 0); DEBUG_PRINTLN("k");
            } else {
                // Adaptive update (slower than main baseline)
                if (voc_results[i] > rtc_data.voc_baselines[i]) {
                    rtc_data.voc_baselines[i] = rtc_data.voc_baselines[i] * 0.99f + voc_results[i] * 0.01f;
                }
            }
        }
    }
}

// Perform multi-temperature sensing measurement
bool performMultiTempMeasurement(float voc_results[VOC_TEMP_COUNT]) {
    bool success = false;
    
    DEBUG_PRINTLN("🌡️ Multi-temp VOC scan:");
    
    // Store original heater config to restore later
    uint16_t original_temp = heatr_conf.heatr_temp;
    uint16_t original_dur = heatr_conf.heatr_dur;
    
    for (int profile_idx = 0; profile_idx < VOC_TEMP_COUNT; profile_idx++) {
        const voc_temp_profile_t* profile = &voc_profiles[profile_idx];
        
        DEBUG_PRINT("  "); DEBUG_PRINT(profile->temperature); DEBUG_PRINT("°C ");
        
        // Configure heater for this profile
        heatr_conf.heatr_temp = profile->temperature;
        heatr_conf.heatr_dur = profile->duration;
        
        int8_t rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);
        if (rslt != BME68X_OK) {
            DEBUG_PRINTLN("❌");
            continue;
        }
        
        // Perform measurement
        rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
        if (rslt != BME68X_OK) continue;
        
        // Wait for measurement with profile-specific timing
        uint32_t del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme) + 
                             (profile->duration * 1000);
        bme.delay_us(del_period, bme.intf_ptr);
        
        // Additional stabilization delay for higher temperatures
        if (profile->temperature > 330) {
            energySaveDelay(100); // Extra stability for high-temp measurements
        }
        
        // Read data
        uint8_t n_fields;
        rslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme);
        
        if (rslt == BME68X_OK && n_fields > 0) {
            bool gasm_valid = (data.status & BME68X_GASM_VALID_MSK);
            bool heat_stab = (data.status & BME68X_HEAT_STAB_MSK);
            
            if (gasm_valid && heat_stab && data.gas_resistance > 0) {
                voc_results[profile_idx] = data.gas_resistance;
                success = true;
                
                DEBUG_PRINTF(data.gas_resistance/1000, 0); DEBUG_PRINTLN("k✅");
            } else {
                DEBUG_PRINTLN("❌");
                voc_results[profile_idx] = 0;
            }
        } else {
            DEBUG_PRINTLN("❌");
            voc_results[profile_idx] = 0;
        }
        
        // Small delay between profiles to prevent sensor heating issues
        energySaveDelay(50);
    }
    
    // CRITICAL FIX: Restore original heater configuration
    heatr_conf.heatr_temp = original_temp;
    heatr_conf.heatr_dur = original_dur;
    int8_t rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);
    
    return success;
}

// ESP-IDF v4 ADC DMA implementation
static bool initAdcDma() {
  // Configure ADC1 for continuous mode
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_12);
  return true;
}

static int readBatteryMvDma(uint16_t min_samples, uint16_t max_samples) {
  // Static buffer for samples - no dynamic allocation, limited to 16 samples
  static uint16_t raws[16];
  const int MAX_SAMP = min((int)max_samples, 16); // Limit to 16 samples
  
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

void readBatteryStatus() {
    // Read battery voltage using DMA only - no fallback, limited to 16 samples
    int mv_in = readBatteryMvDma(16 /*min samples*/, 16 /*max samples*/);
    
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


// Optimized I2C functions for accuracy
int8_t bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    Wire.beginTransmission(BME68X_I2C_ADDR);
    Wire.write(reg_addr);
    if (Wire.endTransmission() != 0) return -1;
    
    if (Wire.requestFrom(BME68X_I2C_ADDR, len) != len) return -1;
    
    for (uint32_t i = 0; i < len; i++) {
        if (Wire.available()) {
            reg_data[i] = Wire.read();
        } else {
            return -1;
        }
    }
    return 0;
}

int8_t bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    Wire.beginTransmission(BME68X_I2C_ADDR);
    Wire.write(reg_addr);
    for (uint32_t i = 0; i < len; i++) {
        Wire.write(reg_data[i]);
    }
    return (Wire.endTransmission() == 0) ? 0 : -1;
}

void bme68x_delay_us(uint32_t period, void *intf_ptr) {
    delayMicroseconds(period);
}

// Callback function for ESP-NOW send operation
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    sendSuccess = (status == ESP_NOW_SEND_SUCCESS);
    
    #ifdef DEBUG_MODE
        DEBUG_PRINT("Send Status: ");
        DEBUG_PRINTLN(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
    #endif
}

void enterDeepSleep() {

    #if defined(DEBUG_MODE) && defined(SKIP_DEEP_SLEEP)
        DEBUG_PRINTLN("⏰ 10s restart...");
        
        // Simulate shutdown
        digitalWrite(sensorVccPin, LOW);
        Wire.end();
        
        // Wait instead of sleep
        for (int i = 10; i > 0; i--) {
            DEBUG_PRINT(i); DEBUG_PRINT("...");
            energySaveDelay(1000);
        }
        DEBUG_PRINTLN("🔄");
        
        // Reboot instead of sleep
        esp_restart();
        return;
    #endif

    DEBUG_PRINTLN("Preparing for deep sleep...");
    
    // Mark sensor as warmed up for next cycle
    setSensorWarmedUp(true);
    
    // Sleep calculation with ID offset for extra sensors
    uint64_t sleepTime = SEND_INTERVAL_US;
    
    // ID offset for extra sensors (15 seconds per sensor ID to avoid collision with main sensors)
    uint64_t idOffset = (SENSOR_ID - 1) * 15000ULL * 1000ULL;
    
    // Random jitter (±15 seconds)
    int32_t randomJitter = (esp_random() % 30000 - 15000) * 1000ULL;

    // Backoff for failures (3 seconds per failure, max 30 seconds)
    uint64_t backoffTime = 0;
    if (transmissionFailures > 0) {
        backoffTime = min((uint64_t)(transmissionFailures * 3000ULL), 30000ULL) * 1000ULL;
    }
    
    sleepTime += idOffset + randomJitter + backoffTime;
    
    // Ensure reasonable bounds
    const uint64_t MIN_SLEEP = 30 * 1000 * 1000ULL;     // 30 seconds minimum
    const uint64_t MAX_SLEEP = 20 * 60 * 1000 * 1000ULL; // 20 minutes maximum
    
    sleepTime = max(sleepTime, MIN_SLEEP);
    sleepTime = min(sleepTime, MAX_SLEEP);
    
    #ifdef DEBUG_MODE
        DEBUG_PRINT("Total sleep time (min): ");
        DEBUG_PRINTLN(sleepTime / 60000000.0);
        energySaveDelay(50);
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

    // Configure for deep sleep with minimal power
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF);

    // Disable GPIO hold to save power - sensor will be re-initialized on wake
    gpio_hold_dis((gpio_num_t)sensorVccPin);
    gpio_deep_sleep_hold_dis();
    
    esp_sleep_enable_timer_wakeup(sleepTime);
    esp_deep_sleep_start();
}

void readSensorData() {
    // Check if we should use multi-temperature sensing
    // Note: Battery level check moved to main execution sequence
    bool use_multi_temp = shouldUseMultiTempSensing();
    
    #ifndef DEBUG_MODE
        // In production mode: Battery level will be checked after this function
        // For now, use multi-temp based on other criteria
    #endif
    
    setMultiTempMode(use_multi_temp);
    
    if (use_multi_temp) {
        DEBUG_PRINTLN("🔬 PROFESSIONAL MODE");
        
        // First, do standard measurement for baseline data
        float temp_sum = 0, hum_sum = 0, gas_sum = 0;
        int valid_count = 0;
        const int NUM_READINGS = 3; // Reduced for multi-temp to save energy
        
        // Configure optimized standard heater settings (higher temp, shorter duration)
        heatr_conf.heatr_temp = 325; // Increased from 320°C for better baseline sensitivity
        heatr_conf.heatr_dur = 85;   // Reduced from 100ms for energy efficiency
        bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);
        
        // Take standard measurements
        for (int i = 0; i < NUM_READINGS; i++) {
            int8_t rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
            if (rslt != BME68X_OK) continue;
            
            uint32_t del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme) + 
                                 (heatr_conf.heatr_dur * 1000);
            bme.delay_us(del_period, bme.intf_ptr);
            
            uint8_t n_fields;
            rslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme);
            
            if (rslt == BME68X_OK && n_fields > 0) {
                bool gasm_valid = (data.status & BME68X_GASM_VALID_MSK);
                bool heat_stab = (data.status & BME68X_HEAT_STAB_MSK);
                
                if (gasm_valid && heat_stab && data.gas_resistance > 0 &&
                    data.humidity >= 0 && data.humidity <= 100 &&
                    data.temperature >= -40 && data.temperature <= 85) {
                    
                    temp_sum += data.temperature;
                    hum_sum += data.humidity;
                    gas_sum += data.gas_resistance;
                    valid_count++;
                }
            }
            
            if (i < NUM_READINGS - 1) energySaveDelay(120); // Optimized from 150ms (faster without filter)
        }
        
        if (valid_count > 0) {
            float avg_temp = temp_sum / valid_count;
            float avg_hum = hum_sum / valid_count;
            float avg_gas = gas_sum / valid_count;
            
            // Update standard baseline
            updateGasBaseline(avg_gas);
            
            // Perform multi-temperature VOC analysis
            float voc_results[VOC_TEMP_COUNT] = {0};
            bool multi_temp_success = performMultiTempMeasurement(voc_results);
            
            if (multi_temp_success) {
                // Update VOC baselines
                updateVocBaselines(voc_results);
                
                // Calculate enhanced IAQ with VOC data
                sensorData.iaq = calculateAdvancedIAQ(avg_gas, avg_hum, avg_temp, voc_results);
                rtc_data.voc_detection_count++;
                
                DEBUG_PRINT("🧪 VOC: ");
                for (int i = 0; i < VOC_TEMP_COUNT; i++) {
                    if (i > 0) DEBUG_PRINT(" | ");
                    DEBUG_PRINTF(voc_results[i]/1000, 0); DEBUG_PRINT("k");
                }
                DEBUG_PRINTLN(" Ω");
            } else {
                // Fallback to standard IAQ
                sensorData.iaq = calculateIAQ(avg_gas, avg_hum, avg_temp);
                DEBUG_PRINTLN("⚠️ Multi-temp failed, using standard");
            }
            
            sensorData.temperature = avg_temp;
            sensorData.humidity = avg_hum;
            
        } else {
            sensorData.iaq = -1;
            DEBUG_PRINTLN("❌ No valid readings");
        }
    } else {
        // Standard single-temperature measurement (energy-efficient mode)
        DEBUG_PRINTLN("🔋 STANDARD MODE");
        
        float temp_sum = 0, hum_sum = 0, gas_sum = 0;
        uint8_t min_accuracy = 10;
        int valid_count = 0;
        const int NUM_READINGS = 4;  // Reduced from 6 for efficiency
        const int SKIP_READINGS = 1;
        
        // Configure optimized standard heater (higher temp, shorter duration)
        heatr_conf.heatr_temp = 320; // Increased from 310°C for better gas sensitivity  
        heatr_conf.heatr_dur = 90;   // Reduced from 125ms for energy efficiency
        bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);
        
        // Take multiple measurements for stability
        for (int i = 0; i < NUM_READINGS; i++) {
            int8_t rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
            if (rslt != BME68X_OK) continue;
            
            uint32_t del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme) + 
                                 (heatr_conf.heatr_dur * 1000);
            bme.delay_us(del_period, bme.intf_ptr);
            
            uint8_t n_fields;
            rslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme);
            
            if (rslt == BME68X_OK && n_fields > 0) {
                uint8_t current_accuracy = 0;
                bool gasm_valid = (data.status & BME68X_GASM_VALID_MSK);
                bool heat_stab = (data.status & BME68X_HEAT_STAB_MSK);

                if (gasm_valid && heat_stab) current_accuracy = 8;
                else if (gasm_valid || heat_stab) current_accuracy = 5;
                else current_accuracy = 2;
                
                if (i >= SKIP_READINGS) {
                    min_accuracy = min(min_accuracy, current_accuracy);
                }
                
                if (i >= SKIP_READINGS && data.gas_resistance > 0 && current_accuracy >= 3 &&
                    data.humidity >= 0 && data.humidity <= 100 &&
                    data.temperature >= -40 && data.temperature <= 85) {
                    
                    temp_sum += data.temperature;
                    hum_sum += data.humidity;
                    gas_sum += data.gas_resistance;
                    valid_count++;
                }
            }
            
            if (i < NUM_READINGS - 1) energySaveDelay(150); // Optimized from 200ms (faster without filter)
        }

        // Calculate averages and IAQ
        if (valid_count > 0) {
            float avg_temp = temp_sum / valid_count;
            float avg_hum = hum_sum / valid_count;
            float avg_gas = gas_sum / valid_count;

            updateGasBaseline(avg_gas);
            sensorData.iaq = calculateIAQ(avg_gas, avg_hum, avg_temp);
            sensorData.temperature = avg_temp;
            sensorData.humidity = avg_hum;

        } else {
            sensorData.iaq = -1;
            DEBUG_PRINTLN("❌ No valid readings");
        }
    }
    
    // Display final results
    #ifdef DEBUG_MODE
        DEBUG_PRINT("📊 IAQ: "); DEBUG_PRINTF(sensorData.iaq, 1);
        if (sensorData.iaq >= 0) {
            if (sensorData.iaq <= 50) DEBUG_PRINT(" (EXCELLENT)");
            else if (sensorData.iaq <= 100) DEBUG_PRINT(" (GOOD)");
            else if (sensorData.iaq <= 150) DEBUG_PRINT(" (LIGHTLY POLLUTED)");
            else if (sensorData.iaq <= 200) DEBUG_PRINT(" (MODERATELY POLLUTED)");
            else if (sensorData.iaq <= 250) DEBUG_PRINT(" (HEAVILY POLLUTED)");
            else DEBUG_PRINT(" (SEVERELY POLLUTED)");
            
            if (use_multi_temp) DEBUG_PRINT(" [PRO]");
            DEBUG_PRINT(" | "); DEBUG_PRINT(sensorData.voltage); DEBUG_PRINTLN("mV");
        } else {
            DEBUG_PRINTLN(" (INVALID)");
        }
    #endif
}

void initWiFiAndESPNow() {
    DEBUG_PRINTLN("Starting WiFi and ESP-NOW initialization...");
    
    // Safer WiFi initialization - avoid aggressive reset that can cause crashes
    // esp_wifi_stop();  // Comment out aggressive reset
    // esp_wifi_deinit(); // Comment out aggressive reset
    // delay(10);
    
    // Initialize WiFi from scratch only if not already initialized
    // wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    // esp_wifi_init(&cfg);  // Comment out low-level init
    esp_wifi_start();
    
    setCpuFrequencyMhz(80);

    // Initialize WiFi in a more reliable way
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    
    // Small delay to ensure WiFi is ready
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
    
    DEBUG_PRINTLN("📡 ESP-NOW ready");
}

void setup() {
  // Complete boot sequence: Initialize -> Read Sensors -> Send Data -> Sleep
  // No loop() function needed - everything happens in setup()
  
  // Complete WiFi/Bluetooth shutdown at startup
  btStop();                 
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();
  esp_wifi_deinit();
    
  // Check RTC memory consistency
  if (rtc_data.signature != RTC_SIGNATURE) {
      DEBUG_PRINTLN("Initializing RTC data structure...");
      memset(&rtc_data, 0, sizeof(rtc_data)); // Clear entire structure
      rtc_data.signature = RTC_SIGNATURE;
      rtc_data.boot_count = 0;
      rtc_data.failures = 0;
      rtc_data.flags = 0; // Clear all flags
      rtc_data.gas_baseline = 50000.0; // Initial baseline value
      rtc_data.voc_detection_count = 0;
      // Initialize gas history to zero - already done by memset
      // Initialize VOC baselines to zero - already done by memset
      DEBUG_PRINTLN("RTC data structure initialized");
  }
  
  rtc_data.boot_count++;

  // Disable any GPIO holds from previous deep sleep
  gpio_deep_sleep_hold_dis();
  gpio_hold_dis((gpio_num_t)sensorVccPin);
  
  #ifdef DEBUG_MODE
      Serial.begin(115200);
      energySaveDelay(20);
      Serial.printf("\n🚀 ESP32-C3 Sensor #%d | Boot: %d\n", SENSOR_ID, bootCount);
      
      if (baselineEstablished) {
          Serial.printf("📊 Baseline: %.0fΩ | VOC detections: %d\n", gasBaseline, rtc_data.voc_detection_count);
      } else {
          Serial.println("📊 Calibrating baseline...");
      }
  #endif
  
  // Configure pins
  pinMode(sensorVccPin, OUTPUT);
  pinMode(adcPin, INPUT);
  
  // Power up the sensor
  digitalWrite(sensorVccPin, HIGH);
  
  // Initialize I2C with enhanced stability settings
  Wire.begin(sdaPin, sclPin);
  Wire.setClock(400000);
  
  // Initialize BME68x
  bme.read = bme68x_i2c_read;
  bme.write = bme68x_i2c_write;
  bme.delay_us = bme68x_delay_us;
  bme.intf = BME68X_I2C_INTF;
  bme.intf_ptr = NULL;
  bme.amb_temp = 25;

  setCpuFrequencyMhz(20);
  
  int8_t rslt = bme68x_init(&bme);
  if (rslt != BME68X_OK) {
      DEBUG_PRINT("⚠️ BME68x init failed, retrying... ");
      // Try a sensor reset before giving up
      digitalWrite(sensorVccPin, LOW);
      energySaveDelay(100);
      digitalWrite(sensorVccPin, HIGH);
      energySaveDelay(200);
      
      // Retry initialization
      rslt = bme68x_init(&bme);
      if (rslt != BME68X_OK) {
          DEBUG_PRINTLN("❌ Failed");
          enterDeepSleep();
          return;
      } else {
          DEBUG_PRINTLN("✅ OK");
      }
  } else {
      DEBUG_PRINTLN("🔧 BME68x initialized");
  }
  
  // Configure sensor with maximum oversampling and optimized filtering
  rslt = bme68x_get_conf(&conf, &bme);
  if (rslt == BME68X_OK) {
      conf.filter = BME68X_FILTER_OFF;        // Filter OFF - redundant with OS_8X oversampling
      conf.odr = BME68X_ODR_NONE;
      conf.os_hum = BME68X_OS_8X;             // Maximized humidity oversampling (noise reduction)
      conf.os_pres = BME68X_OS_4X;            // Optimized pressure oversampling
      conf.os_temp = BME68X_OS_8X;            // Maximized temperature oversampling (noise reduction)
      
      rslt = bme68x_set_conf(&conf, &bme);
      DEBUG_PRINTLN(rslt == BME68X_OK ? "🔧 Optimized precision mode (Filter OFF, OS_8X)" : "⚠️ Config failed");
  }
  
  // Configure heater with optimized settings (higher temp, shorter duration for same energy)
  heatr_conf.enable = BME68X_ENABLE;
  heatr_conf.heatr_temp = 320;    // Increased from 310°C - +10°C for better IAQ sensitivity
  heatr_conf.heatr_dur = 90;      // Reduced from 125ms - same energy, better gas detection
  
  rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);
  DEBUG_PRINTLN(rslt == BME68X_OK ? "🔥 Optimized heater configured" : "⚠️ Heater config failed");
  
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

#if USE_ADC_CONTINUOUS
  // Try ADC DMA init up to 3 times before deep sleep
  bool adcInitOk = false;
  for (int attempts = 0; attempts < 3; attempts++) {
      if (initAdcDma()) {
          adcInitOk = true;
          break;
      }
      DEBUG_PRINT("ADC DMA init failed, retry ");
      DEBUG_PRINTLN(attempts + 1);
      energySaveDelay(20);
  }
  if (!adcInitOk) {
      #ifdef DEBUG_MODE
          DEBUG_PRINTLN("ADC DMA init failed after 3 attempts - critical error");
      #endif
      enterDeepSleep();
      return;
  } else {
      #ifdef DEBUG_MODE
          DEBUG_PRINTLN("ADC DMA initialized");
      #endif
  }
#else
  // Try ADC init up to 3 times before deep sleep (legacy)
  bool adcInitOk = false;
  for (int attempts = 0; attempts < 3; attempts++) {
      if (initAdcDma()) {
          adcInitOk = true;
          break;
      }
      DEBUG_PRINT("ADC init failed, retry ");
      DEBUG_PRINTLN(attempts + 1);
      energySaveDelay(20);
  }
  if (!adcInitOk) {
      #ifdef DEBUG_MODE
          DEBUG_PRINTLN("ADC init failed after 3 attempts - critical error");
      #endif
      enterDeepSleep();
      return;
  } else {
      #ifdef DEBUG_MODE
          DEBUG_PRINTLN("ADC (legacy) initialized");
      #endif
  }
#endif
  
  // Initialize the sensor data structure
  sensorData.sensorId = SENSOR_ID;
  sensorData.temperature = 0;
  sensorData.humidity = 0;
  sensorData.battery_p = 0;
  
  // Optimized sensor warmup for better energy efficiency
  DEBUG_PRINT("⏳ Warmup ");
  unsigned long warmupTime = sensorWarmedUp ? 8000 : 20000; // 8s if warmed, 20s if cold (optimized)
  unsigned long startTime = millis();
  
  while (millis() - startTime < warmupTime) {
      // Simple warmup - just wait and do one test reading
      if (millis() - startTime > warmupTime / 2) {
          // Try one reading halfway through warmup
          bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
          uint32_t del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme) + (heatr_conf.heatr_dur * 1000);
          bme.delay_us(del_period, bme.intf_ptr);
          
          uint8_t n_fields;
          bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme);
          
          if (n_fields > 0 && (data.status & (BME68X_GASM_VALID_MSK | BME68X_HEAT_STAB_MSK))) {
              DEBUG_PRINTLN("⚡ Ready");
              break;
          }
      }
      energySaveDelay(500);
  }
  
  setSensorWarmedUp(true);
  DEBUG_PRINTLN("✅ Setup complete\n");

  // Additional safety delay before main execution
  energySaveDelay(100);

  // ========== MAIN EXECUTION SEQUENCE ==========
  #ifdef DEBUG_MODE
      DEBUG_PRINT("🔄 Cycle #"); DEBUG_PRINTLN(bootCount);
  #endif

  // 1. SENSOR MEASUREMENT PHASE (Optimized for efficiency)
  // Optimized: 6 measurements total, use last 4 for better accuracy/efficiency balance
  const int NUM_MEASUREMENTS = 6;  // Reduced from 10 for energy efficiency
  const int IGNORE_FIRST = 2;      // Reduced from 5 for faster convergence

  float iaq_sum = 0, temp_sum = 0, hum_sum = 0;
  int valid_count = 0;

  for (int i = 0; i < NUM_MEASUREMENTS; i++) {
      readSensorData();
      DEBUG_PRINT("Messung "); DEBUG_PRINTLN(i + 1);

      // Use the last 4 measurements for averaging
      if (i >= IGNORE_FIRST && sensorData.iaq >= 0) {
          iaq_sum += sensorData.iaq;
          temp_sum += sensorData.temperature;
          hum_sum += sensorData.humidity;
          valid_count++;
      }
      energySaveDelay(200); // Optimized from 200ms (faster convergence without filter)
  }

  // Durchschnitt berechnen, falls genug gültige Messungen
  if (valid_count > 0) {
      sensorData.iaq = iaq_sum / valid_count;
      sensorData.temperature = temp_sum / valid_count;
      sensorData.humidity = hum_sum / valid_count;
  } else {
      sensorData.iaq = -1;
  }

  // 2. BATTERY MEASUREMENT PHASE (moved before sensor shutdown)
  readBatteryStatus();
  
  // 2.5. SENSOR SHUTDOWN PHASE - Turn off sensor after all readings are complete
  digitalWrite(sensorVccPin, LOW);
  DEBUG_PRINTLN("Sensor power turned off");
  
  // 3. COMMUNICATION PHASE - Initialize WiFi/ESP-NOW just-in-time to minimize radio on-time
  bool transmissionSuccess = false;
  initWiFiAndESPNow();

  for (int attempts = 0; attempts < 3 && !transmissionSuccess; attempts++) {
      if (attempts > 0) {
          DEBUG_PRINT("🔄 Retry "); DEBUG_PRINTLN(attempts);
          delay(RETRY_BACKOFF_BASE_MS * attempts); // Minimal increasing delay between retries
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
          delay(2); // Reduced polling delay for faster response
      }
      
      if (sendSuccess) {
          transmissionSuccess = true;
          DEBUG_PRINTLN("📤 Sent OK");
          DEBUG_PRINT("Sent - Temp: ");
          DEBUG_PRINT(sensorData.temperature);
          DEBUG_PRINT("°C, Humidity: ");
          DEBUG_PRINT(sensorData.humidity);
          DEBUG_PRINT("%, IAQ: ");
          DEBUG_PRINT(sensorData.iaq);
          DEBUG_PRINT(", Battery: ");
          DEBUG_PRINT(sensorData.voltage);
          DEBUG_PRINTLN("mV");
      } else {
          DEBUG_PRINTLN("📤 Send failed");
      }
  }
  
  // Update transmission success/failure counters
  if (transmissionSuccess) {
      rtc_data.failures = 0;
  } else {
      rtc_data.failures = min(rtc_data.failures + 1, 10);
      DEBUG_PRINT("⚠️ Failures: "); DEBUG_PRINTLN(rtc_data.failures);
  }

  #ifdef DEBUG_MODE
      Serial.flush();
      delay(50);
  #endif

  // 5. DEEP SLEEP PHASE
  enterDeepSleep();
}

// Empty loop() function required by Arduino framework (never called due to deep sleep)
void loop() {
  // This function is never reached because we go into deep sleep at the end of setup()
  // But the Arduino framework requires it to be defined for the linker
}