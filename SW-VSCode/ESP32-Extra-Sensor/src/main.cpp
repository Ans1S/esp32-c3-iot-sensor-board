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
// !! Increment this value whenever the rtc_data_t layout changes to force re-init !!
#define RTC_SIGNATURE 0xB64DAE81

// Pin definitions
#define adcPin 3
#define sensorVccPin 10
#define sdaPin 5
#define sclPin 4
#define BME68X_I2C_ADDR 0x77

// Battery constants
#define BATTERY_MIN 3300
#define BATTERY_MAX 4200

// Sleep settings - deep sleep for 5 minutes between transmissions
#define SEND_INTERVAL_MIN 5     // Send data every 5 minutes
#define SEND_INTERVAL_SEC (SEND_INTERVAL_MIN * 60) 
#define SEND_INTERVAL_US (SEND_INTERVAL_SEC * 1000000ULL)

// Radio configuration
#define WIFI_CHANNEL 1
// TX power in quarter-dBm steps. ESP32-C3 max = 84 (21 dBm).
#define ESP_NOW_TX_POWER_QDBM 84

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
uint8_t receiverAddress[] = {}; // ESP32-S3 Station MAC

// ESP-NOW variables
esp_now_peer_info_t peerInfo;
bool sendSuccess = false;

// Current cycle measurement counter (reset each boot)
// Used to determine when sensor is stable enough for baseline initialization
uint8_t currentCycleMeasurement = 0;

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

// RTC data structure (48 bytes, zero implicit padding - fields ordered largest-first)
//
// OVERFLOW ANALYSIS for multi-year operation (@5 min cycle = 288 boots/day):
//   boot_count  uint32_t → overflows after 4,294,967,295 boots ≈ 40,000 years  ✅
//   voc_detection_count uint32_t → same                                         ✅
//   failures / consecutive_tx_failures  clamped at 10                           ✅
//   persistent_offset_ms max 119,999 ms, never incremented                      ✅
//   gas_baseline float adapter 0.1%/call, stays in [1k, 2M] Ω range            ✅
typedef struct {
    // --- 4-byte fields first (no padding needed between them) ---
    uint32_t signature;              // 4 bytes  offset  0
    uint32_t boot_count;             // 4 bytes  offset  4  (was uint16_t: overflowed in ~7.5 months!)
    uint32_t persistent_offset_ms;   // 4 bytes  offset  8  (stable TX-slot offset [0,120000] ms)
    uint32_t voc_detection_count;    // 4 bytes  offset 12  (was uint8_t: overflowed in ~22 h!)
    // --- float fields (4-byte aligned, no padding) ---
    float    gas_baseline;           // 4 bytes  offset 16
    float    gas_history[3];         // 12 bytes offset 20
    float    voc_baselines[3];       // 12 bytes offset 32
    // --- 1-byte fields at end (no trailing padding needed, struct size = 48 = multiple of 4) ---
    uint8_t  failures;               // 1 byte   offset 44  (total TX failures, clamped at 10)
    uint8_t  consecutive_tx_failures;// 1 byte   offset 45  (resets on success; triggers re-randomization)
    uint8_t  flags;                  // 1 byte   offset 46  (bit flags: warmed_up, baseline_est, ...)
    uint8_t  _pad;                   // 1 byte   offset 47  (explicit pad – keeps size at 48, a mult. of 4)
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

// ============================================================================
// OPTIMAL HEATER TIMING CONFIGURATION (Based on BME680 Datasheet Section 3.3.5)
// 
// Datasheet specifications:
// - Heater needs 20-30ms to reach target temperature
// - heat_stab_r bit indicates when target temperature is reached
// - BSEC recommends 320°C @ 150ms for optimal gas measurement
// - Response time (τ33-63%): LP mode = 1.4s, Continuous = 0.75s
// - Humidity response time (τ0-63%): ~8s
// 
// For accurate measurements:
// 1. Heater duration must be long enough for stable temperature
// 2. Gas sensor needs time to respond after heater stabilizes
// 3. Minimum recommended: 100ms for heater + gas measurement
// ============================================================================

// Measurement timing constants (in milliseconds)
const uint16_t HEATER_STABILIZATION_MS = 30;   // Time for heater to reach target temp
const uint16_t MIN_GAS_MEASUREMENT_MS = 100;   // Minimum for reliable gas reading
const uint16_t OPTIMAL_HEATER_DURATION = 150;  // BSEC recommended duration
const uint16_t POST_READING_STABILIZATION_MS = 50; // Time between consecutive readings

// Professional VOC detection profiles - optimized for accuracy (Datasheet compliant)
// Duration = heater stabilization (30ms) + gas measurement time
const voc_temp_profile_t voc_profiles[VOC_TEMP_COUNT] = {
    {280, 120, "Light VOCs/Alcohols",     0.9f},  // 280°C for alcohols, ketones (longer for accuracy)
    {320, 150, "Standard VOCs/Aromatics", 1.0f},  // 320°C @ 150ms = BSEC standard
    {370, 100, "Heavy Hydrocarbons",      1.2f}   // 370°C shorter (sensor responds faster at high temp)
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

// ============================================================================
// SCIENTIFICALLY CORRECT IAQ CALCULATION
// Based on: BME680 Datasheet, BSEC methodology, and scientific MOX sensor research
// 
// Key improvements:
// 1. Humidity compensation using absolute humidity (Magnus equation)
// 2. Logarithmic gas resistance relationship (MOX sensor physics)
// 3. Proper weighting: 75% gas, 25% humidity (per BME680 datasheet Table 4)
// 4. No temperature in IAQ (temp affects comfort, not air quality measurement)
// ============================================================================

// Humidity compensation slope - experimentally determined for 320°C heater profile
// Reference: https://github.com/thstielow/raspi-bme680-iaq
const float HUM_COMP_SLOPE = 0.04f;

// Temperature compensation - MOX sensors show temperature dependency
// Reference: BME680 datasheet Section 3.4 - gas resistance varies with temperature
// Compensation factor normalizes to 25°C reference temperature
const float TEMP_REFERENCE = 25.0f;  // Reference temperature for normalization
const float TEMP_COMP_FACTOR = 0.02f; // ~2% resistance change per °C

// Calculate absolute humidity in g/m³ using Magnus equation
// Reference: Ideal gas law + Magnus approximation for water vapor pressure
float calculateAbsoluteHumidity(float temperature, float relHumidity) {
    // Magnus equation constants (Tetens approximation)
    const float A = 17.62f;
    const float B = 243.12f;
    
    // Saturation vapor pressure calculation (hPa) via Magnus formula
    float satVaporPressure = 6.112f * expf((A * temperature) / (B + temperature));
    
    // Absolute humidity: rho = (RH * P_sat * M_water) / (R * T)
    // M_water/R = 2.1674 (combined constant for g/m³)
    float absHumidity = (relHumidity * satVaporPressure * 2.1674f) / (273.15f + temperature);
    
    return absHumidity;
}

// Compensate gas resistance for humidity influence
// MOX sensors show exponential dependency on water vapor concentration
float compensateGasResistance(float gas_resistance, float absHumidity) {
    // R_comp = R_gas * exp(slope * abs_humidity)
    // This removes the humidity-induced resistance reduction
    return gas_resistance * expf(HUM_COMP_SLOPE * absHumidity);
}

// Full compensation for both humidity AND temperature effects
// Temperature affects MOX sensor catalytic activity
float compensateGasResistanceFull(float gas_resistance, float absHumidity, float temperature) {
    // Step 1: Humidity compensation
    float hum_compensated = gas_resistance * expf(HUM_COMP_SLOPE * absHumidity);
    
    // Step 2: Temperature compensation (normalize to 25°C reference)
    // Higher temperature = lower resistance (inverse relationship)
    // R_comp = R_hum * (1 + factor * (T - T_ref))
    float temp_factor = 1.0f + TEMP_COMP_FACTOR * (temperature - TEMP_REFERENCE);
    temp_factor = constrain(temp_factor, 0.5f, 2.0f); // Safety bounds
    
    return hum_compensated * temp_factor;
}

// Median calculation for 3 values (robust against outliers)
float median3(float a, float b, float c) {
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;
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
    
    // ========== STEP 1: HUMIDITY & TEMPERATURE COMPENSATION ==========
    // Calculate absolute humidity (g/m³) for proper gas resistance compensation
    float absHumidity = calculateAbsoluteHumidity(temperature, humidity);
    
    // Compensate gas resistance for BOTH humidity AND temperature influence
    float compensatedGas = compensateGasResistanceFull(gas_resistance, absHumidity, temperature);
    
    // ========== STEP 2: GAS SCORE CALCULATION (75% weight) ==========
    // Use logarithmic relationship (MOX sensor physics: resistance vs concentration is exponential)
    // Reference: BME680 datasheet - higher resistance = cleaner air
    float gas_ratio = compensatedGas / gasBaseline;
    float gas_score;
    
    if (gas_ratio >= 1.0f) {
        // Cleaner than baseline = excellent air quality
        gas_score = 100.0f;
    } else if (gas_ratio <= 0.0f) {
        gas_score = 0.0f;
    } else {
        // Logarithmic mapping: small resistance changes at low pollution levels
        // are more significant than at high pollution levels
        // gas_score = 100 * (1 + log10(ratio)) for ratio in [0.1, 1.0] maps to [0, 100]
        gas_score = 100.0f * (1.0f + log10f(gas_ratio)) / 1.0f;
        gas_score = constrain(gas_score, 0.0f, 100.0f);
    }
    
    // ========== STEP 3: MULTI-TEMP VOC ENHANCEMENT ==========
    float voc_penalty = 0.0f;
    
    if (multiTempModeActive && voc_results != nullptr) {
        int valid_voc_readings = 0;
        float voc_deviation_sum = 0.0f;
        
        for (int i = 0; i < VOC_TEMP_COUNT; i++) {
            if (voc_results[i] > 0 && rtc_data.voc_baselines[i] > 0) {
                float voc_ratio = voc_results[i] / rtc_data.voc_baselines[i];
                
                // Weights for different VOC temperature profiles
                // Higher temps detect heavier hydrocarbons (more harmful)
                float weight = (i == 0) ? 0.25f : (i == 1) ? 0.45f : 0.30f;
                
                // Use absolute deviation from baseline (both directions contribute)
                // ratio > 1: cleaner air, ratio < 1: more pollution
                // Both cases indicate air quality variation
                float log_deviation = fabsf(log10f(voc_ratio)) * weight;
                
                // Pollution (ratio < 1) is weighted 3x more than "cleaner" deviation
                if (voc_ratio < 1.0f) {
                    log_deviation *= 3.0f;
                }
                voc_deviation_sum += log_deviation;
                valid_voc_readings++;
            }
        }
        
        if (valid_voc_readings >= 2) {
            // Convert log deviation to score penalty (max 30 points)
            voc_penalty = constrain(voc_deviation_sum * 30.0f, 0.0f, 30.0f);
            DEBUG_PRINT("VOC penalty: "); DEBUG_PRINTF(voc_penalty, 1); DEBUG_PRINTLN("");
        }
    }
    
    // Apply VOC penalty to gas score
    gas_score = max(0.0f, gas_score - voc_penalty);
    
    // ========== STEP 4: HUMIDITY SCORE (25% weight) ==========
    // Per BME680 datasheet: optimal indoor humidity is 40-60%
    // Contribution based on deviation from optimal range
    float hum_score;
    
    if (humidity >= 40.0f && humidity <= 60.0f) {
        // Optimal range - full score
        hum_score = 100.0f;
    } else if (humidity < 40.0f) {
        // Too dry: linear decrease, 0% humidity = 0 score
        hum_score = (humidity / 40.0f) * 100.0f;
    } else {
        // Too humid: linear decrease, 100% humidity = 0 score
        hum_score = ((100.0f - humidity) / 40.0f) * 100.0f;
    }
    hum_score = constrain(hum_score, 0.0f, 100.0f);
    
    // ========== STEP 5: COMBINED SCORE & IAQ CONVERSION ==========
    // BME680 datasheet recommends: 75% gas, 25% humidity
    // Temperature is NOT included in IAQ (it's for comfort, not air quality)
    const float GAS_WEIGHT = 0.75f;
    const float HUM_WEIGHT = 0.25f;
    
    float combined_score = (gas_score * GAS_WEIGHT) + (hum_score * HUM_WEIGHT);
    
    // Convert to IAQ scale: 0-500 (0=excellent, 500=hazardous)
    // Per BSEC/BME680 convention: IAQ = (100 - score) * 5
    float iaq = (100.0f - combined_score) * 5.0f;
    
    // Minimum IAQ floor: Even excellent air shows ~5-15 IAQ (realism)
    // This prevents misleading 0.0 readings and matches BSEC behavior
    const float IAQ_MIN_FLOOR = 5.0f;
    iaq = constrain(iaq, IAQ_MIN_FLOOR, 500.0f);
    
    // Sanity check
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

// Baseline update with humidity AND temperature compensation
// Uses fully compensated gas resistance for stable baseline tracking
void updateGasBaseline(float gas_resistance, float humidity, float temperature) {
    // Full compensation for both humidity AND temperature
    // This ensures baseline is independent of ambient conditions
    float absHumidity = calculateAbsoluteHumidity(temperature, humidity);
    float compensatedGas = compensateGasResistanceFull(gas_resistance, absHumidity, temperature);
    
    // Add to simplified gas history (3 samples)
    static uint8_t history_index = 0;
    rtc_data.gas_history[history_index] = compensatedGas;
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
        // Use median instead of mean for robustness against outliers
        float median;
        if (valid_samples == 3) {
            median = median3(rtc_data.gas_history[0], rtc_data.gas_history[1], rtc_data.gas_history[2]);
        } else {
            // Only 2 samples: use average
            median = sum / valid_samples;
        }

        if (!baselineEstablished || rtc_data.gas_baseline < 1000.0f || rtc_data.gas_baseline > 2000000.0f) {
            // Initialize baseline on first run or outliers
            rtc_data.gas_baseline = median;
            setBaselineEstablished(true);
            DEBUG_PRINTLN("📏 Gas baseline calibrated");
        } else {
            // VERY slow upward-only adaptation (per BSEC methodology)
            // Baseline represents "cleanest observed air"
            // Only adapt 0.1% per call to prevent rapid convergence
            // With 6 readings per cycle: max 0.6% change per cycle
            if (median > rtc_data.gas_baseline) {
                rtc_data.gas_baseline = rtc_data.gas_baseline * 0.999f + median * 0.001f;
            }
            // NO downward adjustment - prevents pollution from corrupting baseline
        }
    }
}

// Multi-temperature VOC baseline management
// Uses currentCycleMeasurement to ensure sensor is stable before setting baseline
// This works correctly regardless of deep sleep interval (1 min or 10 min)
void updateVocBaselines(float voc_results[VOC_TEMP_COUNT]) {
    if (!multiTempModeActive || voc_results == nullptr) return;
    
    // Only initialize baselines after measurement 4+ in current cycle
    // This ensures sensor has fully stabilized after warmup
    // (Measurements 1-3 may still be unstable due to sensor heating)
    const uint8_t MIN_STABLE_MEASUREMENT = 4;
    bool sensorStable = (currentCycleMeasurement >= MIN_STABLE_MEASUREMENT);
    
    for (int i = 0; i < VOC_TEMP_COUNT; i++) {
        if (voc_results[i] > 0) {
            if (rtc_data.voc_baselines[i] <= 0 && sensorStable) {
                // Initialize baseline only after sensor has stabilized in current cycle
                // Works with any sleep interval (1 min, 10 min, etc.)
                rtc_data.voc_baselines[i] = voc_results[i];
                DEBUG_PRINT("📏 VOC "); DEBUG_PRINT(i); 
                DEBUG_PRINT(" baseline: "); DEBUG_PRINTF(rtc_data.voc_baselines[i]/1000, 0); DEBUG_PRINTLN("k");
            } else if (rtc_data.voc_baselines[i] > 0) {
                // Slow bidirectional adaptation - 0.2% per reading
                // Allows baseline to track both cleaner and slightly polluted air over time
                // This prevents baseline getting stuck at initial (cold) values
                rtc_data.voc_baselines[i] = rtc_data.voc_baselines[i] * 0.998f + voc_results[i] * 0.002f;
            }
            // Note: If sensor not yet stable and no baseline exists, we skip
            // to allow sensor to fully stabilize during current measurement cycle
        }
    }
}

// Perform multi-temperature sensing measurement
// Based on BME680 Datasheet Section 3.4: heat_stab_r indicates heater reached target
bool performMultiTempMeasurement(float voc_results[VOC_TEMP_COUNT]) {
    bool success = false;
    
    DEBUG_PRINTLN("🌡️ Multi-temp VOC scan:");
    
    // Store original heater config to restore later
    uint16_t original_temp = heatr_conf.heatr_temp;
    uint16_t original_dur = heatr_conf.heatr_dur;
    
    for (int profile_idx = 0; profile_idx < VOC_TEMP_COUNT; profile_idx++) {
        const voc_temp_profile_t* profile = &voc_profiles[profile_idx];
        
        DEBUG_PRINT("  "); DEBUG_PRINT(profile->temperature); DEBUG_PRINT("°C ");
        
        // Configure heater for this profile (using datasheet-compliant durations)
        heatr_conf.heatr_temp = profile->temperature;
        heatr_conf.heatr_dur = profile->duration;
        
        int8_t rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);
        if (rslt != BME68X_OK) {
            DEBUG_PRINTLN("❌cfg");
            voc_results[profile_idx] = 0;
            continue;
        }
        
        // Perform measurement with retry logic for heat_stab
        bool measurement_valid = false;
        const int MAX_HEAT_RETRIES = 2; // Retry if heater didn't stabilize
        int retry_count = 0;
        
        for (int retry = 0; retry <= MAX_HEAT_RETRIES && !measurement_valid; retry++) {
            rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
            if (rslt != BME68X_OK) continue;
            
            // Wait for measurement: TPH measurement time + heater duration
            // Datasheet: del_period = meas_dur + (heatr_dur * 1000)
            // Cast to uint32_t before multiply to prevent implicit int promotion
            uint32_t del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme) +
                                 ((uint32_t)profile->duration * 1000u);
            bme.delay_us(del_period, bme.intf_ptr);
            
            // Read data and check heat_stab status
            uint8_t n_fields;
            rslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme);
            
            if (rslt == BME68X_OK && n_fields > 0) {
                bool gasm_valid = (data.status & BME68X_GASM_VALID_MSK);
                bool heat_stab = (data.status & BME68X_HEAT_STAB_MSK);
                
                // Datasheet Section 5.3.5.6: heat_stab_r indicates target temp reached
                if (gasm_valid && heat_stab && data.gas_resistance > 0) {
                    voc_results[profile_idx] = data.gas_resistance;
                    success = true;
                    measurement_valid = true;
                    DEBUG_PRINTF(data.gas_resistance/1000, 0); 
                    if (retry_count > 0) {
                        DEBUG_PRINT("k✅(R"); DEBUG_PRINT(retry_count); DEBUG_PRINTLN(")");
                    } else {
                        DEBUG_PRINTLN("k✅");
                    }
                } else if (!heat_stab && retry < MAX_HEAT_RETRIES) {
                    // Heater didn't reach target - retry with extra wait
                    retry_count++;
                    DEBUG_PRINT("🔄heat_stab=0,R"); DEBUG_PRINT(retry_count); DEBUG_PRINT(" ");
                    energySaveDelay(HEATER_STABILIZATION_MS);
                } else if (!gasm_valid) {
                    DEBUG_PRINT("⚠️gas_invalid ");
                }
            }
        }
        
        if (!measurement_valid) {
            DEBUG_PRINTLN("❌");
            voc_results[profile_idx] = 0;
        }
        
        // Delay between profiles for sensor cooling/stabilization
        energySaveDelay(POST_READING_STABILIZATION_MS);
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
        DEBUG_PRINTLN("⚠️ SKIP_DEEP_SLEEP: RTC memory will be LOST (use deep sleep for production)");
        
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
    
    // =========================================================================
    // COLLISION AVOIDANCE: persistent-phase-offset strategy
    //
    // Each sensor owns a stable random offset (persistent_offset_ms) stored in
    // RTC memory. Unlike a fresh jitter every cycle, this offset is preserved
    // across deep sleep so every sensor keeps its unique transmission slot
    // indefinitely. The slot is only re-randomized when consecutive TX failures
    // exceed the threshold, which is an indicator of a phase collision.
    //
    // Components:
    //  1. SEND_INTERVAL_US     – nominal 5-minute base period
    //  2. idOffset             – hard deterministic spread: 15 s × (ID-1)
    //                            guarantees minimum inter-sensor separation
    //  3. persistent_offset_ms – stable random spread within [0, 120 s]
    //                            initialized once on first boot; survives sleep
    //  4. backoffTime          – exponential backoff on consecutive failures
    //                            (doubles each consecutive failure, max 120 s)
    //                            helps escape collision quickly if it occurs
    // =========================================================================

    uint64_t sleepTime = SEND_INTERVAL_US;

    // 1. ID-based hard offset: ensures minimum separation between sensors
    uint64_t idOffset = (uint64_t)(SENSOR_ID - 1) * 15000ULL * 1000ULL;

    // 2. Persistent phase offset --------------------------------------------------
    // Threshold of consecutive failures before considering ourselves phase-locked
    // with another sensor and re-randomizing our slot.
    const uint8_t RERANDOM_THRESHOLD = 3;

    if (rtc_data.persistent_offset_ms == 0 ||
        rtc_data.consecutive_tx_failures >= RERANDOM_THRESHOLD) {
        // First boot OR stuck in collision → pick a new random slot [0, 120 s]
        rtc_data.persistent_offset_ms = esp_random() % 120000u;
        if (rtc_data.consecutive_tx_failures >= RERANDOM_THRESHOLD) {
            // Reset counter after re-randomizing so we don't re-randomize every cycle
            rtc_data.consecutive_tx_failures = 0;
            DEBUG_PRINTLN("⚡ TX slot re-randomized to escape collision");
        } else {
            DEBUG_PRINTLN("⚡ TX slot initialized");
        }
    }
    uint64_t persistentOffset = (uint64_t)rtc_data.persistent_offset_ms * 1000ULL;

    // 3. Exponential backoff on consecutive failures (1 s, 2 s, 4 s … max 120 s)
    // This moves the sensor quickly if a collision is detected, even before
    // the re-randomization threshold is reached.
    uint64_t backoffTime = 0;
    if (rtc_data.consecutive_tx_failures > 0) {
        uint32_t backoff_ms = 1000u << min((uint32_t)rtc_data.consecutive_tx_failures - 1u, (uint32_t)7u); // max 128 s
        backoff_ms = min(backoff_ms, (uint32_t)120000u); // hard cap at 120 s
        backoffTime = (uint64_t)backoff_ms * 1000ULL;
    }

    sleepTime += idOffset + persistentOffset + backoffTime;
    
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
        
        // Configure heater with BSEC-recommended optimal settings
        // Datasheet: 20-30ms heater stabilization + gas measurement time
        // BSEC standard: 320°C @ 150ms for reliable gas readings
        heatr_conf.heatr_temp = 320; // BSEC standard temperature
        heatr_conf.heatr_dur = OPTIMAL_HEATER_DURATION; // 150ms for stable measurement
        bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);
        
        // Take standard measurements
        for (int i = 0; i < NUM_READINGS; i++) {
            int8_t rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
            if (rslt != BME68X_OK) continue;
            
            // Cast to uint32_t before multiply to prevent implicit int promotion
            uint32_t del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme) +
                                 ((uint32_t)heatr_conf.heatr_dur * 1000u);
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
            
            if (i < NUM_READINGS - 1) energySaveDelay(POST_READING_STABILIZATION_MS);
        }
        
        if (valid_count > 0) {
            float avg_temp = temp_sum / valid_count;
            float avg_hum = hum_sum / valid_count;
            float avg_gas = gas_sum / valid_count;
            
            // Update standard baseline (with humidity compensation)
            updateGasBaseline(avg_gas, avg_hum, avg_temp);
            
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
                
                // Show baseline tracking info
                DEBUG_PRINT("📊 Baselines - Gas: "); DEBUG_PRINTF(gasBaseline/1000, 0); 
                DEBUG_PRINT("k | VOC: ");
                for (int i = 0; i < VOC_TEMP_COUNT; i++) {
                    if (i > 0) DEBUG_PRINT("/");
                    DEBUG_PRINTF(rtc_data.voc_baselines[i]/1000, 0);
                }
                DEBUG_PRINTLN("k");
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
        
        // Configure heater with BSEC-recommended optimal settings
        // Datasheet Section 3.3.5: Heater needs sufficient time for stable measurement
        // BSEC standard: 320°C @ 150ms for reliable IAQ readings
        heatr_conf.heatr_temp = 320; // BSEC standard temperature
        heatr_conf.heatr_dur = OPTIMAL_HEATER_DURATION; // 150ms for stable measurement
        bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);
        
        // Take multiple measurements for stability
        for (int i = 0; i < NUM_READINGS; i++) {
            int8_t rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
            if (rslt != BME68X_OK) continue;
            
            // Cast to uint32_t before multiply to prevent implicit int promotion
            uint32_t del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme) +
                                 ((uint32_t)heatr_conf.heatr_dur * 1000u);
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
            
            if (i < NUM_READINGS - 1) energySaveDelay(POST_READING_STABILIZATION_MS);
        }

        // Calculate averages and IAQ
        if (valid_count > 0) {
            float avg_temp = temp_sum / valid_count;
            float avg_hum = hum_sum / valid_count;
            float avg_gas = gas_sum / valid_count;

            updateGasBaseline(avg_gas, avg_hum, avg_temp);
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
      rtc_data.persistent_offset_ms = 0; // 0 = not yet initialized, will be set in enterDeepSleep()
      rtc_data.voc_detection_count = 0;
      rtc_data.gas_baseline = 500000.0f; // Realistic initial baseline (500kΩ)
      rtc_data.failures = 0;
      rtc_data.consecutive_tx_failures = 0;
      rtc_data.flags = 0; // Clear all flags
      rtc_data._pad = 0;
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
  
  // ============================================================================
  // SENSOR CONFIGURATION (Based on BME680 Datasheet and BSEC recommendations)
  // 
  // Datasheet Section 3.3.4 (IIR Filter):
  //   - Reduces short-term fluctuations from external disturbances
  //   - Increases T/P resolution to 20 bit
  //   - BSEC recommends filter coefficient 3 for indoor applications
  // 
  // Datasheet Section 3.3.1-3.3.3 (Oversampling):
  //   - BSEC standard: Temp=8x, Humidity=2x, Pressure=4x
  //   - Higher oversampling = lower noise, longer measurement time
  // 
  // Measurement duration calculation:
  //   Base: ~2.3ms per measurement
  //   + 2.3ms * (2^osrs_t - 1) for temperature
  //   + 2.3ms * (2^osrs_p - 1) for pressure
  //   + 2.3ms * (2^osrs_h - 1) for humidity
  //   + heater duration for gas
  // ============================================================================
  rslt = bme68x_get_conf(&conf, &bme);
  if (rslt == BME68X_OK) {
      // BSEC-recommended configuration for optimal accuracy
      conf.filter = BME68X_FILTER_SIZE_3;     // IIR filter coef 3 (reduces transient noise)
      conf.odr = BME68X_ODR_NONE;             // No standby (we use forced mode)
      conf.os_temp = BME68X_OS_8X;            // Temperature: 8x oversampling (BSEC standard)
      conf.os_hum = BME68X_OS_2X;             // Humidity: 2x oversampling (BSEC standard)
      conf.os_pres = BME68X_OS_4X;            // Pressure: 4x oversampling (BSEC standard)
      
      rslt = bme68x_set_conf(&conf, &bme);
      DEBUG_PRINTLN(rslt == BME68X_OK ? "🔧 BSEC config (IIR=3, T8x/H2x/P4x)" : "⚠️ Config failed");
  }
  
  // Configure heater with BSEC-recommended optimal settings
  // Datasheet Section 3.3.5: 320°C @ 150ms is standard for IAQ measurement
  heatr_conf.enable = BME68X_ENABLE;
  heatr_conf.heatr_temp = 320;                // BSEC standard heater temperature
  heatr_conf.heatr_dur = OPTIMAL_HEATER_DURATION; // 150ms for stable measurement
  
  rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);
  DEBUG_PRINTLN(rslt == BME68X_OK ? "🔥 Heater: 320°C @ 150ms (BSEC)" : "⚠️ Heater config failed");
  
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
          // Cast to uint32_t before multiply to prevent implicit int promotion
          uint32_t del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme) +
                               ((uint32_t)heatr_conf.heatr_dur * 1000u);
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
  
  // Reset measurement counter for this boot cycle
  currentCycleMeasurement = 0;

  for (int i = 0; i < NUM_MEASUREMENTS; i++) {
      currentCycleMeasurement = i + 1;  // Update measurement counter (1-based)
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
      rtc_data.consecutive_tx_failures = 0; // Reset exponential-backoff / re-randomization counter
  } else {
      rtc_data.failures = min((int)rtc_data.failures + 1, 10);
      rtc_data.consecutive_tx_failures = min((int)rtc_data.consecutive_tx_failures + 1, 10);
      DEBUG_PRINT("⚠️ Failures: "); DEBUG_PRINT(rtc_data.failures);
      DEBUG_PRINT(" (consecutive: "); DEBUG_PRINT(rtc_data.consecutive_tx_failures); DEBUG_PRINTLN(")");
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