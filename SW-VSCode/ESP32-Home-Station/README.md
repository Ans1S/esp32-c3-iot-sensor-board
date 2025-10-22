# 🏠 ESP32 Home Station

## Overview

This is the **central hub** for the lil-ESP32-C3 environmental monitoring system. The home station receives data from multiple sensor nodes via **ESP-NOW**, aggregates it, and uploads to **ThingSpeak** for cloud visualization.

## Features

- 📡 **ESP-NOW Receiver**: Collects data from multiple sensor nodes
- ☁️ **ThingSpeak Integration**: Cloud data upload and visualization
- 🔄 **Dual-Mode Operation**: 
  - ESP-NOW mode (low power listening)
  - WiFi mode (brief uploads only)
- 📊 **Data Queuing**: Buffers data to prevent loss
- ⚡ **Power Management**: Dynamic CPU frequency scaling
- 🎯 **Multi-Sensor Support**: Handles different sensor types

## Architecture

```
┌─────────────┐     ESP-NOW      ┌──────────────┐
│ Sensor #1   │ ─────────────>   │              │
│ (BME280)    │                  │              │
└─────────────┘                  │              │
                                 │              │
┌─────────────┐     ESP-NOW      │    Home      │     WiFi    ┌────────────┐
│ Sensor #2   │ ─────────────>   │   Station    │ ────────>  │ ThingSpeak │
│ (BME680)    │                  │  (ESP32-C3)  │             │   Cloud    │
└─────────────┘                  │              │             └────────────┘
                                 │              │
┌─────────────┐     ESP-NOW      │              │
│ Sensor #3   │ ─────────────>   │              │
│ (BMP280)    │                  │              │
└─────────────┘                  └──────────────┘
```

## Hardware Requirements

- ESP32-C3 board (from this project)
- USB-C power supply (always on)
- WiFi network (2.4GHz)
- ThingSpeak account (free)

## Configuration

1. **Copy the template:**
   ```bash
   cp src/config.h.example src/config.h
   ```

2. **Edit `config.h`:**
   ```cpp
   // WiFi credentials
   #define WIFI_SSID "your-wifi-ssid"
   #define WIFI_PSWD "your-wifi-password"
   #define WIFI_TIMEOUT_MS 20000
   
   // ThingSpeak settings
   #define CHANNEL_ID 1234567  // Your channel ID
   #define CHANNEL_API_KEY "YOUR_API_KEY_HERE"
   
   // Power management
   #define CPU_FREQ_MHZ_NORMAL 160  // Normal operation
   #define CPU_FREQ_MHZ_LOW 80      // Low-power idle
   ```

3. **Setup ThingSpeak:**
   - Create account at [thingspeak.com](https://thingspeak.com)
   - Create new channel
   - Add fields:
     - Field 1: Sensor 1 Temperature
     - Field 2: Sensor 1 Humidity
     - Field 3: Sensor 2 Temperature
     - Field 4: Sensor 2 Humidity
     - Field 5: Sensor 3 IAQ (Air Quality)
     - Field 6: Battery voltages
     - (Customize based on your sensors)
   - Copy Channel ID and Write API Key to config.h

## Installation

### Using PlatformIO (Recommended)

```bash
# Navigate to project directory
cd SW-VSCode/ESP32-Home-Station

# Build and upload
pio run --target upload

# Monitor serial output
pio device monitor
```

### Using Arduino IDE

1. Install required libraries:
   - ThingSpeak
   - WiFi
   - ESPAsyncWebServer (optional)

2. Open `main.cpp` in Arduino IDE
3. Select board: "ESP32C3 Dev Module"
4. Upload

## Operation

### Dual-Mode System

**ESP-NOW Mode (Default):**
- Low power consumption (~80mA)
- Listening for sensor data
- No WiFi connection
- Immediate response to sensor nodes

**WiFi Mode (Upload):**
- Higher power (~120mA)
- Connects to WiFi
- Uploads data to ThingSpeak
- Disconnects after upload
- Returns to ESP-NOW mode

### Data Flow

1. **Receive** data from sensor node via ESP-NOW
2. **Parse** data based on sensor ID
3. **Queue** data in memory buffer
4. **Switch** to WiFi mode
5. **Upload** to ThingSpeak (15-second intervals)
6. **Switch** back to ESP-NOW mode

### Power Management

The system automatically adjusts CPU frequency:
- **160 MHz**: Active receiving/uploading
- **80 MHz**: Idle state
- Activates on data reception
- Returns to low power after 1 minute idle

## Serial Monitor Output

Example output:
```
======================================
ESP32 Home Station Starting...
======================================
MAC Address: 24:58:7C:E4:13:B8
WiFi Manager initialized
ESP-NOW initialized successfully
Listening for sensor data...

Received - Sensor 1: Temp=22.5°C, Humidity=45.2%, Battery=3850mV
Received - Sensor 2: Temp=23.1°C, Humidity=48.7%, Battery=3920mV
Received - Extra-Sensor 1: IAQ=95.4, Battery=3780mV

Switching to WiFi mode...
Connected to WiFi: 192.168.1.100
Uploading to ThingSpeak...
Upload successful!
Switching back to ESP-NOW mode...
```

## Sensor ID Mapping

The system handles different sensor types based on ID:

```cpp
Sensor ID 1-2:  Regular sensors (BME280/BMP280)
                → Temperature + Humidity + Battery
                
Sensor ID 3+:   Extra sensors (BME680)
                → IAQ (Air Quality) + Battery
```

### ThingSpeak Field Mapping

Customize in `uploadToThingSpeak()` function:

```cpp
switch (data->sensorId) {
    case 1:
        ThingSpeak.setField(1, data->temperature);
        ThingSpeak.setField(2, data->humidity);
        break;
    case 2:
        ThingSpeak.setField(3, data->temperature);
        ThingSpeak.setField(4, data->humidity);
        break;
    case 3:
        ThingSpeak.setField(5, data->iaq);
        break;
}
```

## Troubleshooting

### Not Receiving Data

1. Check ESP-NOW is enabled (should see "ESP-NOW initialized")
2. Verify sensor nodes have correct receiver MAC
3. Check distance (< 100m line of sight)
4. Ensure WiFi channel matches sensor nodes

### WiFi Connection Issues

1. Verify SSID and password in config.h
2. Check WiFi network is 2.4GHz (ESP32 doesn't support 5GHz)
3. Increase `WIFI_TIMEOUT_MS` in config.h
4. Check router is not blocking ESP32

### ThingSpeak Upload Failing

1. Verify Channel ID and API Key
2. Check rate limiting (15 seconds between uploads)
3. Verify internet connection
4. Check ThingSpeak service status
5. Monitor serial output for error codes

### Queue Overflow

If you see "Queue full" messages:
1. Increase `MAX_QUEUE_SIZE` in code
2. Reduce sensor update frequency
3. Increase upload frequency

## Customization

### Add More Sensor Types

Edit sensor data structure:
```cpp
typedef struct sensor_data {
    uint8_t sensorId;
    float temperature;
    float humidity;
    float iaq;
    float co2;         // Add new fields
    float voc;         // Add new fields
    int voltage;
    uint8_t battery_p;
} sensor_data;
```

### Change Upload Interval

Edit in code:
```cpp
const unsigned long writeInterval = 15000;  // milliseconds
// ThingSpeak free tier: minimum 15 seconds
```

### Modify Power Management

Adjust thresholds:
```cpp
const unsigned long idleThreshold = 60000;  // 1 minute
#define CPU_FREQ_MHZ_LOW 40  // Even lower power
```

## Advanced Features

### FreeRTOS Tasks

The system uses multiple tasks:
- `wifiTask`: Handles WiFi connection
- `uploadTask`: Manages ThingSpeak uploads
- `powerMgmtTask`: Monitors and adjusts power

### Queue System

Data is buffered to prevent loss:
```cpp
#define MAX_QUEUE_SIZE 20
sensor_data dataQueue[MAX_QUEUE_SIZE];
```

### Mode Switching

Automatic and manual mode control:
```cpp
void switchWiFiMode(OperationMode mode);
// ESPNOW_MODE or WIFI_MODE
```

## Performance

| Metric | Value |
|--------|-------|
| Max sensor nodes | 20+ (limited by memory) |
| Data queue size | 20 messages |
| Upload interval | 15 seconds (ThingSpeak limit) |
| Power (ESP-NOW) | ~80 mA |
| Power (WiFi) | ~120 mA |
| Average power | ~85 mA |

## Related Projects

- [Sensor Node Sender](../ESP-Now-Air-Quality/README.md)
- [Alternative Receiver](../ESP-Now-Air-Quality-Rec/README.md)
- [Main Project](../../README.md)

## License

See [main project license](../../LICENSE)
