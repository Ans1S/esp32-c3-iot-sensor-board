#include <Arduino.h>
#include <WiFi.h>
#include <ThingSpeak.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_pm.h>
#include "config.h"  // Include configuration file with WiFi and ThingSpeak credentials

// Station state
enum StationState {
  WAITING_FOR_DATA,
  CONNECTING_WIFI,
  UPLOADING_DATA,
  DISCONNECTING
};

StationState currentState = WAITING_FOR_DATA;

// Structure to match sensor payload format
typedef struct sensor_data {
  uint8_t sensorId;     // Identifier for the sensor
  float temperature;    // Temperature in Celsius (for sensors 1&2)
  float humidity;       // Humidity in % (for sensors 1&2)
  float iaq;            // Air Quality Index (for sensors 3+)
  int voltage;          // Battery voltage in mV (all sensors)
  uint8_t battery_p;    // Compatibility field (not used)
} sensor_data;

// Queue to store received sensor data
#define MAX_QUEUE_SIZE 20  // Increased queue size
sensor_data dataQueue[MAX_QUEUE_SIZE];
int queueCount = 0;
SemaphoreHandle_t queueMutex;

// ThingSpeak client
WiFiClient client;

// Task handles
TaskHandle_t wifiTaskHandle = NULL;
TaskHandle_t uploadTaskHandle = NULL;
TaskHandle_t powerMgmtTaskHandle = NULL;

// ThingSpeak rate limiting
unsigned long lastWriteTime = 0;
const unsigned long writeInterval = 15000;  // 15 seconds between writes

// Activity tracking for power management
unsigned long lastActivityTime = 0;
const unsigned long idleThreshold = 60000;  // 1 minute of no activity to trigger low power

// Function declarations
void processQueue();
void connectToWifi();
void uploadToThingSpeak(sensor_data *data);
void initESPNow();
void configureLowPowerMode();
void configureNormalPowerMode();

// Callback function executed when data is received via ESP-NOW
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  // Record activity
  lastActivityTime = millis();
  
  // Ensure we're in normal power mode
  configureNormalPowerMode();
  
  if (len == sizeof(sensor_data)) {
    // Take the mutex to protect the queue
    if (xSemaphoreTake(queueMutex, portMAX_DELAY) == pdTRUE) {
      // Copy incoming data to our structure if there's room in the queue
      if (queueCount < MAX_QUEUE_SIZE) {
        memcpy(&dataQueue[queueCount], incomingData, sizeof(sensor_data));
        queueCount++;
        
        // Print data to serial console based on sensor type
        sensor_data *data = (sensor_data *)incomingData;
        
        if (data->sensorId <= 2) {
          // Regular sensors (temperature + humidity)
          Serial.printf("Received - Sensor %d: Temp=%.1f°C, Humidity=%.1f%%, Battery=%dmV\n",
                      data->sensorId,
                      data->temperature,
                      data->humidity,
                      data->voltage);
        } else {
          // Extra sensors (air quality)
          Serial.printf("Received - Extra-Sensor %d: IAQ=%.1f, Battery=%dmV\n",
                      data->sensorId - 2,  // Show as Extra-Sensor 1, 2, etc.
                      data->iaq,
                      data->voltage);
        }
        
        // If we're waiting for data, start the WiFi connection process
        if (currentState == WAITING_FOR_DATA) {
          currentState = CONNECTING_WIFI;
          // Resume WiFi task if it exists
          if (wifiTaskHandle != NULL) {
            vTaskResume(wifiTaskHandle);
          }
        }
      } else {
        Serial.println("ERROR: Data queue is full!");
      }
      
      xSemaphoreGive(queueMutex);
    }
  }
}

// WiFi management task
void wifiTask(void *parameters) {
  for (;;) {
    // This task stays suspended until we have data to send
    if (currentState == CONNECTING_WIFI) {
      Serial.println("Connecting to WiFi...");
      
      // Switch to normal power mode for WiFi connection
      configureNormalPowerMode();
      
      // Temporarily disable ESP-NOW
      esp_now_deinit();
      
      // Configure WiFi
      WiFi.mode(WIFI_STA);
      WiFi.begin(WIFI_SSID, WIFI_PSWD);
      
      // Try to connect with timeout
      unsigned long startTime = millis();
      bool connected = false;
      while (millis() - startTime < WIFI_TIMEOUT_MS) {
        if (WiFi.status() == WL_CONNECTED) {
          connected = true;
          break;
        }
        Serial.print(".");
        vTaskDelay(500 / portTICK_PERIOD_MS);
      }
      
      if (connected) {
        Serial.println("\nWiFi connected");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        
        // Move to uploading state and resume upload task
        currentState = UPLOADING_DATA;
        if (uploadTaskHandle != NULL) {
          vTaskResume(uploadTaskHandle);
        }
      } else {
        Serial.println("\nWiFi connection failed");
        // Go back to ESP-NOW mode
        currentState = DISCONNECTING;
        WiFi.disconnect(true);
        initESPNow();
        currentState = WAITING_FOR_DATA;
      }
    }
    
    // Suspend until needed again
    vTaskSuspend(NULL);
  }
}

// ThingSpeak upload task
void uploadTask(void *parameters) {
  for (;;) {
    // This task stays suspended until WiFi is connected
    if (currentState == UPLOADING_DATA) {
      // Record activity
      lastActivityTime = millis();
      
      // Process all items in the queue
      processQueue();
      
      // Disconnect WiFi and go back to ESP-NOW mode
      currentState = DISCONNECTING;
      Serial.println("Disconnecting WiFi...");
      WiFi.disconnect(true);
      
      // Reinitialize ESP-NOW
      initESPNow();
      
      // Return to waiting state
      currentState = WAITING_FOR_DATA;
      Serial.println("Waiting for new sensor data...");
      
      // Check if we should enter low power mode
      if (millis() - lastActivityTime > idleThreshold) {
        configureLowPowerMode();
      }
    }
    
    // Suspend until needed again
    vTaskSuspend(NULL);
  }
}

// Power management task - monitors activity and adjusts power accordingly
void powerMgmtTask(void *parameters) {
  for (;;) {
    // Only manage power when in WAITING_FOR_DATA state
    if (currentState == WAITING_FOR_DATA) {
      if (millis() - lastActivityTime > idleThreshold) {
        // No activity for a while, enter low power mode
        configureLowPowerMode();
      } else {
        // Recent activity, ensure normal power mode
        configureNormalPowerMode();
      }
    }
    
    // Check every 5 seconds
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

// Process all data in the queue and upload to ThingSpeak
void processQueue() {
  while (queueCount > 0) {
    // Respect ThingSpeak rate limits
    unsigned long currentTime = millis();
    if (currentTime - lastWriteTime < writeInterval) {
      unsigned long waitTime = writeInterval - (currentTime - lastWriteTime);
      Serial.printf("Waiting %lu ms before next ThingSpeak write\n", waitTime);
      vTaskDelay(waitTime / portTICK_PERIOD_MS);
    }
    
    // Take a data point from the queue
    sensor_data currentData;
    if (xSemaphoreTake(queueMutex, portMAX_DELAY) == pdTRUE) {
      memcpy(&currentData, &dataQueue[0], sizeof(sensor_data));
      
      // Shift remaining items in the queue
      for (int i = 0; i < queueCount - 1; i++) {
        memcpy(&dataQueue[i], &dataQueue[i + 1], sizeof(sensor_data));
      }
      queueCount--;
      
      xSemaphoreGive(queueMutex);
    }
    
    // Upload to ThingSpeak
    uploadToThingSpeak(&currentData);
    lastWriteTime = millis();
  }
}

// Upload a single sensor data point to ThingSpeak
void uploadToThingSpeak(sensor_data *data) {
  // Clear all fields before setting new ones
  ThingSpeak.setStatus("");
  
  if (data->sensorId == 1) {
    // Sensor 1: Fields 1-3 (Temperature, Humidity, Battery Voltage)
    ThingSpeak.setField(1, data->temperature);
    ThingSpeak.setField(2, data->humidity);
    ThingSpeak.setField(3, data->voltage);
    
    Serial.printf("Uploading Sensor 1: Temp=%.1f°C, Humidity=%.1f%%, Battery=%dmV\n",
                  data->temperature, data->humidity, data->voltage);
                  
  } else if (data->sensorId == 2) {
    // Sensor 2: Fields 4-6 (Temperature, Humidity, Battery Voltage)
    ThingSpeak.setField(4, data->temperature);
    ThingSpeak.setField(5, data->humidity);
    ThingSpeak.setField(6, data->voltage);
    
    Serial.printf("Uploading Sensor 2: Temp=%.1f°C, Humidity=%.1f%%, Battery=%dmV\n",
                  data->temperature, data->humidity, data->voltage);
                  
  } else if (data->sensorId == 3) {
    // Extra-Sensor 1: Fields 7-8 (IAQ, Battery Voltage)
    ThingSpeak.setField(7, data->iaq);
    ThingSpeak.setField(8, data->voltage);
    
    Serial.printf("Uploading Extra-Sensor 1: IAQ=%.1f, Battery=%dmV\n",
                  data->iaq, data->voltage);
                  
  } else if (data->sensorId == 4) {
    // Extra-Sensor 2: Overwrites fields 7-8 (only one extra sensor at a time!)
    Serial.println("WARNING: Extra-Sensor 2 overwrites Extra-Sensor 1 data!");
    ThingSpeak.setField(7, data->iaq);
    ThingSpeak.setField(8, data->voltage);
    
    Serial.printf("Uploading Extra-Sensor 2: IAQ=%.1f, Battery=%dmV\n",
                  data->iaq, data->voltage);
                  
  } else {
    Serial.printf("ERROR: Unknown sensor ID %d\n", data->sensorId);
    return;
  }
  
  int result = ThingSpeak.writeFields(CHANNEL_ID, CHANNEL_API_KEY);
  
  if (result == 200) {
    Serial.printf("ThingSpeak upload successful for sensor %d\n", data->sensorId);
  } else {
    Serial.printf("ThingSpeak upload failed for sensor %d. Error: %d\n", data->sensorId, result);
  }
}

// Initialize ESP-NOW
void initESPNow() {
  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("ESP-NOW initialized. Waiting for sensor data...");
}

// Configure power-saving mode
void configureLowPowerMode() {
  static bool inLowPowerMode = false;
  
  if (!inLowPowerMode) {
    Serial.println("Entering low power mode...");
    
    // 1. Reduce CPU frequency
    setCpuFrequencyMhz(CPU_FREQ_MHZ_LOW);
    
    // 2. Enable modem sleep while maintaining ESP-NOW functionality
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    
    // 3. Configure automatic light sleep between tasks
    esp_pm_config_esp32s3_t pm_config = {
        .max_freq_mhz = CPU_FREQ_MHZ_LOW,
        .min_freq_mhz = CPU_FREQ_MHZ_LOW,
        .light_sleep_enable = true
    };
    esp_pm_configure(&pm_config);
    
    inLowPowerMode = true;
    Serial.printf("CPU frequency set to %d MHz, modem sleep enabled\n", CPU_FREQ_MHZ_LOW);
  }
}

// Configure normal power mode
void configureNormalPowerMode() {
  static bool inNormalPowerMode = false;
  
  if (!inNormalPowerMode) {
    Serial.println("Switching to normal power mode...");
    
    // 1. Set CPU to normal frequency
    setCpuFrequencyMhz(CPU_FREQ_MHZ_NORMAL);
    
    // 2. Disable power-saving for WiFi to ensure responsive ESP-NOW
    esp_wifi_set_ps(WIFI_PS_NONE);
    
    // 3. Disable automatic light sleep
    esp_pm_config_esp32s3_t pm_config = {
        .max_freq_mhz = CPU_FREQ_MHZ_NORMAL,
        .min_freq_mhz = CPU_FREQ_MHZ_NORMAL,
        .light_sleep_enable = false
    };
    esp_pm_configure(&pm_config);
    
    inNormalPowerMode = true;
    Serial.printf("CPU frequency set to %d MHz, modem sleep disabled\n", CPU_FREQ_MHZ_NORMAL);
  }
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\nESP32-S3 Station with ThingSpeak Starting");
  
  // Initialize power management
  configureNormalPowerMode();
  
  // Record initial activity
  lastActivityTime = millis();
  
  // Create mutex for queue protection
  queueMutex = xSemaphoreCreateMutex();
  if (queueMutex == NULL) {
    Serial.println("Error creating mutex");
  }
  
  // Initialize ThingSpeak
  ThingSpeak.begin(client);
  
  // Initialize ESP-NOW
  initESPNow();
  
  // Create WiFi task (starts suspended)
  xTaskCreate(
    wifiTask,
    "WiFi Task",
    4096,
    NULL,
    1,
    &wifiTaskHandle
  );
  
  // Create upload task (starts suspended)
  xTaskCreate(
    uploadTask,
    "Upload Task",
    4096,
    NULL,
    1,
    &uploadTaskHandle
  );
  
  // Create power management task (always running)
  xTaskCreate(
    powerMgmtTask,
    "Power Management",
    2048,
    NULL,
    tskIDLE_PRIORITY,
    &powerMgmtTaskHandle
  );
  
  Serial.println("Tasks created. Waiting for sensor data...");
}

void loop() {
  // Main loop is empty as all work is done in tasks
  delay(100);
}