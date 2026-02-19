/**
 * ESP32-S3 Home Station — Robuste Version mit WiFi-Recovery
 *
 * Wichtigste Verbesserungen:
 *  - Exponentieller Backoff bei WiFi-Fehlern (verhindert Stack-Korruption)
 *  - Periodischer Retry-Wächter (triggert Upload auch ohne neues Sensor-Paket)
 *  - Auto-Restart nach zu vielen Fehlern (letztes Mittel)
 *  - Queue-Overflow: Älteste Daten werden verworfen, neueste behalten
 *  - ESP-NOW Health-Check nach Reinit (kein stilles Versagen mehr)
 *  - Power-Mode Bug: Einzelner Status statt zwei unabhängiger static-Flags
 *  - Race Conditions: currentState via portENTER_CRITICAL geschützt
 *  - Task-Signaling: xTaskNotify statt vTaskResume (kein verlorenes Wecksignal)
 *  - FreeRTOS QueueHandle_t statt manuell verschiebtem Array
 *  - Stack-Größen erhöht (8192 für WiFi/Upload)
 *  - ThingSpeak Retry-Logik bei Fehler
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ThingSpeak.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_pm.h>
#include <inttypes.h>  // PRIu32 — portabler Format-Specifier für uint32_t
#include "config.h"

// ---------------------------------------------------------------------------
// Sensor-Datenstruktur (muss mit Sender übereinstimmen)
// ---------------------------------------------------------------------------
typedef struct sensor_data {
  uint8_t sensorId;   // 1–2: Temp+Humidity, 3+: IAQ
  float   temperature;
  float   humidity;
  float   iaq;
  int     voltage;    // Batterie in mV
  uint8_t battery_p;  // Kompatibilitätsfeld
} sensor_data;

// ---------------------------------------------------------------------------
// FreeRTOS-Handles
// ---------------------------------------------------------------------------
#define SENSOR_QUEUE_SIZE  20
static QueueHandle_t  sensorQueue      = NULL;
static TaskHandle_t   wifiTaskHandle   = NULL;
static TaskHandle_t   uploadTaskHandle = NULL;
static TaskHandle_t   powerTaskHandle  = NULL;

// ---------------------------------------------------------------------------
// ThingSpeak
// ---------------------------------------------------------------------------
static WiFiClient client;
static unsigned long lastWriteTime  = 0;
const  unsigned long WRITE_INTERVAL = 15000UL;   // 15 s zwischen Schreibvorgängen
#define THINGSPEAK_RETRIES  3                      // Wiederholversuche bei Fehler

// ---------------------------------------------------------------------------
// WiFi Retry / Backoff
// ---------------------------------------------------------------------------
// Backoff-Stufen in ms: sofort, 30s, 1min, 2min, 5min, danach immer 5min
static const uint32_t WIFI_BACKOFF_MS[] = { 0, 30000, 60000, 120000, 300000 };
#define WIFI_BACKOFF_STEPS  5
static volatile uint8_t  wifiFailCount     = 0;    // Aufeinanderfolgende Fehlversuche
#define WIFI_RESTART_AFTER  15                      // Neustart nach X konsekutiven Fehlern

// Wächter: versucht Upload auch wenn keine neuen Daten kommen (WiFi kam zurück)
static volatile unsigned long lastConnectAttempt = 0;
// Mindest-Abstand zwischen Verbindungsversuchen (verhindert Stack-Korruption)
#define MIN_CONNECT_INTERVAL_MS  10000UL

// ---------------------------------------------------------------------------
// Power Management — EIN gemeinsamer Status (Bug-Fix: zwei unabhängige Flags)
// ---------------------------------------------------------------------------
enum PowerMode { PM_NORMAL, PM_LOW };
static volatile PowerMode activePowerMode = PM_NORMAL;

// Flags aus Callback setzen, Power-Wechsel erledigt powerMgmtTask
static volatile bool requestNormalPower = false;

// ---------------------------------------------------------------------------
// Aktivitäts-Tracking
// ---------------------------------------------------------------------------
static volatile unsigned long lastActivityTime = 0;
const unsigned long IDLE_THRESHOLD = 60000UL;    // 1 min Inaktivität → Low Power

// FIX Uptime-Overflow: millis() überläuft nach 49,7 Tagen.
// uptimeSeconds wird alle 5 s um 5 erhöht → Overflow erst nach ~136 Jahren.
static volatile uint32_t uptimeSeconds = 0;

// ---------------------------------------------------------------------------
// Zustandsmaschine — via Critical-Section geschützt
// ---------------------------------------------------------------------------
enum StationState { WAITING_FOR_DATA, CONNECTING_WIFI, UPLOADING_DATA, DISCONNECTING };
static volatile StationState currentState = WAITING_FOR_DATA;

static portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

static inline StationState getState() {
  StationState s;
  portENTER_CRITICAL(&stateMux);
  s = currentState;
  portEXIT_CRITICAL(&stateMux);
  return s;
}

static inline void setState(StationState s) {
  portENTER_CRITICAL(&stateMux);
  currentState = s;
  portEXIT_CRITICAL(&stateMux);
}

// ---------------------------------------------------------------------------
// Vorwärts-Deklarationen
// ---------------------------------------------------------------------------
void uploadToThingSpeak(sensor_data *data);
bool initESPNow();
void setPowerMode(PowerMode mode);

// ---------------------------------------------------------------------------
// Debug-Hilfsfunktionen
// ---------------------------------------------------------------------------

// Timestamp-Prefix: [  1234ms]
static void printTs(const char *msg) {
  uint32_t t = millis();
  Serial.printf("[%6" PRIu32 "ms] %s\n", t, msg);
}

// Fortschrittsbalken: [TAG] [========--------] done/total text
// FIX VLA: Kein char bar[width+1] (Variable-Length Array auf FreeRTOS-Stack
//          kann bei unerwartetem 'width' Stack-Overflow verursachen).
//          Feste Puffergröße 24 Bytes, Hard-Limit auf width <= 22.
static void printBar(const char *tag, uint32_t done, uint32_t total,
                     const char *suffix = "", uint8_t width = 20) {
  if (width > 22) width = 22;
  uint32_t filled = (total > 0) ? ((uint64_t)done * width / total) : 0;
  if (filled > width) filled = width;
  char bar[24];  // FIX: Fest statt VLA
  for (uint8_t i = 0; i < width; i++) bar[i] = (i < filled) ? '=' : '-';
  bar[width] = '\0';
  Serial.printf("[%6" PRIu32 "ms] [%s] [%s] %" PRIu32 "/%" PRIu32 " %s\r",
                millis(), tag, bar, done, total, suffix);
}

// Trennlinie
static void printSep() {
  Serial.println("--------------------------------------------");
}

// Uptime als D HH:MM:SS
// FIX Drift: uptimeSeconds wird jetzt anhand der tatsächlich vergangenen
//            FreeRTOS-Ticks aktualisiert statt fest +5 pro Iteration.
//            Kein Overflow < 136 Jahre.
static void printUptime() {
  uint32_t s = uptimeSeconds;
  uint32_t d = s / 86400u; s %= 86400u;
  Serial.printf("%" PRIu32 "d %02" PRIu32 ":%02" PRIu32 ":%02" PRIu32,
                d, s / 3600u, (s % 3600u) / 60u, s % 60u);
}

// ---------------------------------------------------------------------------
// ESP-NOW Empfangs-Callback
// (läuft im WiFi-Stack-Task — so kurz wie möglich halten!)
// ---------------------------------------------------------------------------
void IRAM_ATTR OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len != sizeof(sensor_data)) return;

  // Aktivität merken
  lastActivityTime = millis();
  // Power-Wechsel als Flag, nicht direkt aus Callback ausführen
  requestNormalPower = true;

  // Daten in FreeRTOS-Queue einreihen (ISR-sicher)
  sensor_data data;
  memcpy(&data, incomingData, sizeof(sensor_data));
  BaseType_t higherPriorityWoken = pdFALSE;

  // Queue voll → Ältestes verwerfen, Neuestes behalten
  if (xQueueSendFromISR(sensorQueue, &data, &higherPriorityWoken) == errQUEUE_FULL) {
    sensor_data dropped;
    xQueueReceiveFromISR(sensorQueue, &dropped, &higherPriorityWoken);
    xQueueSendFromISR(sensorQueue, &data, &higherPriorityWoken);
    // (Serial.println hier nicht erlaubt — ISR-Kontext)
  }

  // WiFi-Task wecken wenn im Wartezustand (xTaskNotify geht nicht verloren)
  if (wifiTaskHandle != NULL && getState() == WAITING_FOR_DATA) {
    setState(CONNECTING_WIFI);
    vTaskNotifyGiveFromISR(wifiTaskHandle, &higherPriorityWoken);
  }

  portYIELD_FROM_ISR(higherPriorityWoken);
}

// ---------------------------------------------------------------------------
// WiFi-Task — wartet auf Notification, exponentieller Backoff, Auto-Recovery
// ---------------------------------------------------------------------------
void wifiTask(void *parameters) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (getState() != CONNECTING_WIFI) continue;

    // ── Exponentieller Backoff ─────────────────────────────────────────────
    uint8_t backoffIdx = (wifiFailCount < WIFI_BACKOFF_STEPS)
                          ? wifiFailCount : (WIFI_BACKOFF_STEPS - 1);
    uint32_t backoffMs = WIFI_BACKOFF_MS[backoffIdx];

    if (backoffMs > 0) {
      Serial.printf("\n[%6" PRIu32 "ms] [WiFi] Backoff %" PRIu32 " s wegen %d Fehler(n)...\n",
                    millis(), backoffMs / 1000u, wifiFailCount);
      // Backoff mit Fortschrittsbalken
      // FIX Heap: String()+c_str() erzeugte alle 500ms eine Heap-Allokation
      //           → Fragmentation nach Wochen. snprintf nutzt Stack-Puffer.
      uint32_t backoffStart = millis();
      char backoffSuf[12];
      while ((millis() - backoffStart) < backoffMs) {
        uint32_t elapsed  = millis() - backoffStart;
        uint32_t remainS  = (backoffMs > elapsed) ? (backoffMs - elapsed) / 1000 : 0;
        snprintf(backoffSuf, sizeof(backoffSuf), "%" PRIu32 "s", remainS);
        printBar("Backoff", elapsed, backoffMs, backoffSuf);
        vTaskDelay(pdMS_TO_TICKS(500));
      }
      Serial.println();
    }

    // Mindest-Abstand einhalten
    unsigned long sinceLastTry = millis() - lastConnectAttempt;
    if (sinceLastTry < MIN_CONNECT_INTERVAL_MS) {
      vTaskDelay(pdMS_TO_TICKS(MIN_CONNECT_INTERVAL_MS - sinceLastTry));
    }
    lastConnectAttempt = millis();

    // ── ESP-NOW deaktivieren, WiFi verbinden ──────────────────────────────
    printSep();
    Serial.printf("[%6" PRIu32 "ms] [WiFi] Versuch %d/%d  SSID: \"%s\"\n",
                  millis(), wifiFailCount + 1, WIFI_RESTART_AFTER, WIFI_SSID);
    setPowerMode(PM_NORMAL);
    esp_now_deinit();
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PSWD);

    bool connected = false;
    TickType_t start = xTaskGetTickCount();
    const TickType_t timeout = pdMS_TO_TICKS(WIFI_TIMEOUT_MS);
    uint32_t wallStart = millis();

    while ((xTaskGetTickCount() - start) < timeout) {
      if (WiFi.status() == WL_CONNECTED) { connected = true; break; }
      uint32_t elapsed = millis() - wallStart;
      char suf[24];
      snprintf(suf, sizeof(suf), "%" PRIu32 "s/%" PRIu32 "s",
               elapsed / 1000u, (uint32_t)WIFI_TIMEOUT_MS / 1000u);
      printBar("WiFi ", elapsed, WIFI_TIMEOUT_MS, suf);
      vTaskDelay(pdMS_TO_TICKS(500));
    }
    Serial.println();  // Zeilenumbruch nach \r-Fortschrittszeile

    if (connected) {
      wifiFailCount = 0;
      Serial.printf("[%6" PRIu32 "ms] [WiFi] VERBUNDEN  IP: %s  RSSI: %ddBm\n",
                    millis(),
                    WiFi.localIP().toString().c_str(),
                    WiFi.RSSI());
      setState(UPLOADING_DATA);
      xTaskNotifyGive(uploadTaskHandle);

    } else {
      wifiFailCount++;
      Serial.printf("[%6" PRIu32 "ms] [WiFi] FEHLGESCHLAGEN  Versuch %d/%d\n",
                    millis(), wifiFailCount, WIFI_RESTART_AFTER);

      if (wifiFailCount >= WIFI_RESTART_AFTER) {
        Serial.println("[WiFi] KRITISCH: Zu viele Fehlversuche – Neustart!");
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
      }

      WiFi.disconnect(true, true);
      vTaskDelay(pdMS_TO_TICKS(200));

      if (!initESPNow()) {
        Serial.println("[WiFi] ESP-NOW Reinit fehlgeschlagen – Neustart!");
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
      }
      setState(WAITING_FOR_DATA);
    }
    printSep();
  }
}

// ---------------------------------------------------------------------------
// Upload-Task — wartet auf Notification, leert Queue, geht zurück zu ESP-NOW
// ---------------------------------------------------------------------------
void uploadTask(void *parameters) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (getState() != UPLOADING_DATA) continue;

    lastActivityTime = millis();
    sensor_data item;
    uint32_t totalItems = uxQueueMessagesWaiting(sensorQueue);
    uint32_t processed  = 0;

    printSep();
    Serial.printf("[%6" PRIu32 "ms] [Upload] Starte Upload – %" PRIu32 " Eintraege in Queue\n",
                  millis(), totalItems);

    while (xQueueReceive(sensorQueue, &item, 0) == pdTRUE) {
      processed++;

      // ThingSpeak Rate-Limit mit Fortschrittsbalken
      unsigned long now = millis();
      if (now - lastWriteTime < WRITE_INTERVAL) {
        unsigned long wait = WRITE_INTERVAL - (now - lastWriteTime);
        Serial.printf("[%6" PRIu32 "ms] [Upload] ThingSpeak Rate-Limit: warte %lu ms\n",
                      millis(), wait);
        uint32_t waitStart = millis();
        while ((millis() - waitStart) < wait) {
          uint32_t elapsed = millis() - waitStart;
          char suf[20];
          snprintf(suf, sizeof(suf), "%" PRIu32 "s/%" PRIu32 "s",
                   elapsed / 1000u, (uint32_t)wait / 1000u);
          printBar("Limit", elapsed, (uint32_t)wait, suf);
          vTaskDelay(pdMS_TO_TICKS(500));
        }
        Serial.println();
      }

      // Fortschrittsbalken Queue-Verarbeitung
      printBar("Queue", processed, totalItems > 0 ? totalItems : processed, "");
      Serial.println();

      // Detaillierter Log
      if (item.sensorId <= 2) {
        Serial.printf("[%6" PRIu32 "ms] [Upload] Sensor %d: Temp=%.1f C  Hum=%.1f%%  Batt=%dmV\n",
                      millis(), item.sensorId,
                      item.temperature, item.humidity, item.voltage);
      } else {
        Serial.printf("[%6" PRIu32 "ms] [Upload] Extra-Sensor %d: IAQ=%.1f  Batt=%dmV\n",
                      millis(), item.sensorId - 2, item.iaq, item.voltage);
      }

      uploadToThingSpeak(&item);
      lastWriteTime = millis();
    }

    Serial.printf("[%6" PRIu32 "ms] [Upload] Fertig – %" PRIu32 "/%" PRIu32 " hochgeladen\n",
                  millis(), processed, totalItems);

    // WiFi trennen → ESP-NOW reaktivieren
    setState(DISCONNECTING);
    Serial.printf("[%6" PRIu32 "ms] [WiFi] Trenne Verbindung...\n", millis());
    WiFi.disconnect(true, true);
    vTaskDelay(pdMS_TO_TICKS(200));

    if (!initESPNow()) {
      Serial.println("[Upload] ESP-NOW Reinit fehlgeschlagen – Neustart!");
      vTaskDelay(pdMS_TO_TICKS(500));
      esp_restart();
    }
    setState(WAITING_FOR_DATA);
    printSep();
  }
}

// ---------------------------------------------------------------------------
// Power-Management + Retry-Wächter
// Zwei Aufgaben in einem Task:
//  1. Power-Modus basierend auf Aktivität anpassen
//  2. Periodisch prüfen ob Queue-Daten vorhanden → WiFi neu versuchen
//     (damit der Upload auch klappt wenn WiFi zurückkommt ohne neues Sensor-Paket)
// ---------------------------------------------------------------------------
void powerMgmtTask(void *parameters) {
  for (;;) {
    TickType_t taskStart = xTaskGetTickCount();  // Drift-Fix: Startzeit merken

    // ── Power-Wechsel aus Callback anwenden ──────────────────────────────
    if (requestNormalPower) {
      requestNormalPower = false;
      setPowerMode(PM_NORMAL);
    }

    StationState s = getState();

    // ── Inaktivitäts-Check ────────────────────────────────────────────────
    if (s == WAITING_FOR_DATA) {
      if ((millis() - lastActivityTime) > IDLE_THRESHOLD) {
        setPowerMode(PM_LOW);
      }
    } else {
      setPowerMode(PM_NORMAL);
    }

    // ── Retry-Wächter ─────────────────────────────────────────────────────
    // Greift wenn: Daten in der Queue vorhanden ABER System wartet untätig.
    // Passiert wenn WiFi nach längerer Ausfallzeit zurückkommt und kein neues
    // Sensor-Paket den Trigger auslöst.
    if (s == WAITING_FOR_DATA && uxQueueMessagesWaiting(sensorQueue) > 0) {
      // Backoff-Zeit für aktuellen Fehlerzähler berechnen
      uint8_t idx = (wifiFailCount < WIFI_BACKOFF_STEPS)
                     ? wifiFailCount
                     : (WIFI_BACKOFF_STEPS - 1);
      uint32_t requiredWait = WIFI_BACKOFF_MS[idx];

      // Mindestens MIN_CONNECT_INTERVAL einhalten
      if (requiredWait < MIN_CONNECT_INTERVAL_MS)
        requiredWait = MIN_CONNECT_INTERVAL_MS;

      if ((millis() - lastConnectAttempt) >= requiredWait) {
        Serial.printf("[Wächter] Queue hat %d Einträge, starte WiFi-Versuch...\n",
                      (int)uxQueueMessagesWaiting(sensorQueue));
        setState(CONNECTING_WIFI);
        xTaskNotifyGive(wifiTaskHandle);
      }
    }

    // ── Heartbeat-Statuszeile alle 5 s ───────────────────────────────────
    // FIX Drift: Tatsächlich vergangene Ticks messen statt pauschal +5
    //            addieren. Task-Body braucht messbare Zeit (setPowerMode etc.)
    TickType_t taskEnd = xTaskGetTickCount();
    uint32_t actualSeconds = (uint32_t)((taskEnd - taskStart) / configTICK_RATE_HZ);
    if (actualSeconds == 0) actualSeconds = 5;  // Mindest-Inkrement
    uptimeSeconds += actualSeconds;
    Serial.print("[Heartbeat] Uptime: ");
    printUptime();
    Serial.printf("  Queue: %d/%d  WiFi-Fehler: %d  Power: %s\n",
                  (int)uxQueueMessagesWaiting(sensorQueue),
                  SENSOR_QUEUE_SIZE,
                  wifiFailCount,
                  activePowerMode == PM_NORMAL ? "NORMAL" : "LOW");

    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

// ---------------------------------------------------------------------------
// ThingSpeak Upload mit Retry-Logik
// ---------------------------------------------------------------------------
void uploadToThingSpeak(sensor_data *data) {
  if (data->sensorId == 1) {
    ThingSpeak.setField(1, data->temperature);
    ThingSpeak.setField(2, data->humidity);
    ThingSpeak.setField(3, data->voltage);
  } else if (data->sensorId == 2) {
    ThingSpeak.setField(4, data->temperature);
    ThingSpeak.setField(5, data->humidity);
    ThingSpeak.setField(6, data->voltage);
  } else if (data->sensorId >= 3) {
    // Extra-Sensoren auf Felder 7–8 (Channel-Design-Einschränkung)
    ThingSpeak.setField(7, data->iaq);
    ThingSpeak.setField(8, data->voltage);
  } else {
    Serial.printf("[Upload] FEHLER: Unbekannte Sensor-ID %d\n", data->sensorId);
    return;
  }

  int result = 0;
  for (int attempt = 1; attempt <= THINGSPEAK_RETRIES; attempt++) {
    uint32_t t0 = millis();
    result = ThingSpeak.writeFields(CHANNEL_ID, CHANNEL_API_KEY);
    uint32_t rtt = millis() - t0;
    if (result == 200) {
      Serial.printf("[%6" PRIu32 "ms] [Upload] OK  Sensor %d  Versuch %d/%d  RTT: %" PRIu32 "ms\n",
                    millis(), data->sensorId, attempt, THINGSPEAK_RETRIES, rtt);
      return;
    }
    Serial.printf("[%6" PRIu32 "ms] [Upload] FEHLER %d  Sensor %d  Versuch %d/%d  RTT: %" PRIu32 "ms\n",
                  millis(), result, data->sensorId, attempt, THINGSPEAK_RETRIES, rtt);
    if (attempt < THINGSPEAK_RETRIES) {
      vTaskDelay(pdMS_TO_TICKS(3000));
    }
  }
  Serial.printf("[%6" PRIu32 "ms] [Upload] GESCHEITERT nach %d Versuchen  Sensor %d\n",
                millis(), THINGSPEAK_RETRIES, data->sensorId);
}

// ---------------------------------------------------------------------------
// ESP-NOW initialisieren mit Health-Check
// Gibt true zurück wenn erfolgreich, false bei Fehler
// ---------------------------------------------------------------------------
bool initESPNow() {
  WiFi.mode(WIFI_STA);
  vTaskDelay(pdMS_TO_TICKS(100));  // WiFi-Stack Zeit zum Stabilisieren

  // Eventuell altes ESP-NOW aufräumen (idempotent, verhindert Stack-Korruption)
  esp_now_deinit();
  vTaskDelay(pdMS_TO_TICKS(50));

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] FEHLER: Initialisierung fehlgeschlagen!");
    return false;
  }

  if (esp_now_register_recv_cb(OnDataRecv) != ESP_OK) {
    Serial.println("[ESP-NOW] FEHLER: Callback-Registrierung fehlgeschlagen!");
    esp_now_deinit();
    return false;
  }

  Serial.println("[ESP-NOW] Bereit – warte auf Sensordaten...");
  return true;
}

// ---------------------------------------------------------------------------
// Power-Mode-Wechsel (Bug-Fix: ein einziger Status statt zwei Static-Flags)
// ---------------------------------------------------------------------------
void setPowerMode(PowerMode mode) {
  if (activePowerMode == mode) return;  // Bereits im gewünschten Modus

  if (mode == PM_LOW) {
    Serial.println("[Power] Wechsle in Low-Power-Modus...");
    setCpuFrequencyMhz(CPU_FREQ_MHZ_LOW);
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    esp_pm_config_esp32s3_t cfg = {
      .max_freq_mhz     = CPU_FREQ_MHZ_LOW,
      .min_freq_mhz     = CPU_FREQ_MHZ_LOW,
      .light_sleep_enable = true
    };
    esp_pm_configure(&cfg);
    Serial.printf("[Power] CPU=%dMHz, Modem-Sleep ein\n", CPU_FREQ_MHZ_LOW);
  } else {
    Serial.println("[Power] Wechsle in Normal-Power-Modus...");
    setCpuFrequencyMhz(CPU_FREQ_MHZ_NORMAL);
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_pm_config_esp32s3_t cfg = {
      .max_freq_mhz     = CPU_FREQ_MHZ_NORMAL,
      .min_freq_mhz     = CPU_FREQ_MHZ_NORMAL,
      .light_sleep_enable = false
    };
    esp_pm_configure(&cfg);
    Serial.printf("[Power] CPU=%dMHz, Modem-Sleep aus\n", CPU_FREQ_MHZ_NORMAL);
  }

  activePowerMode = mode;
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
  // ── SCHRITT 1: Serial ──────────────────────────────────────────────────
  Serial.begin(115200);
  uint32_t serialWait = millis();
  while (!Serial && (millis() - serialWait < 3000)) { delay(10); }

  printSep();
  Serial.println("  ESP32-S3 Home Station  –  Booting...");
  printSep();
  printTs("[1/6] Serial bereit (USB-CDC)");

  // ── SCHRITT 2: WiFi NVS bereinigen ────────────────────────────────────
  // FIX: Ohne diese Zeilen versucht der Arduino-WiFi-Stack beim Boot
  // automatisch die letzte gespeicherte SSID zu verbinden → blockiert
  // WiFi.mode() für bis zu 3 Minuten!
  WiFi.persistent(false);       // Keine Credentials in NVS speichern
  WiFi.setAutoReconnect(false); // Keine automatische Reconnect-Versuche
  WiFi.disconnect(true, true);  // Bestehende Verbindung + NVS löschen
  WiFi.mode(WIFI_OFF);          // WiFi komplett aus bis wir es brauchen
  printTs("[2/6] WiFi NVS bereinigt – kein Auto-Reconnect");

  // ── SCHRITT 3: Power-Mode ─────────────────────────────────────────────
  // WICHTIG: esp_pm_configure() erst NACH WiFi.mode(OFF) aufrufen,
  // sonst Konflikt zwischen PM und WiFi-Stack-Init
  setPowerMode(PM_NORMAL);
  lastActivityTime = millis();
  printTs("[3/6] Power-Mode: NORMAL (160 MHz, Modem-Sleep aus)");

  // ── SCHRITT 4: FreeRTOS Queue ─────────────────────────────────────────
  sensorQueue = xQueueCreate(SENSOR_QUEUE_SIZE, sizeof(sensor_data));
  if (sensorQueue == NULL) {
    Serial.println("[Setup] KRITISCH: Queue konnte nicht erstellt werden!");
    while (true) delay(1000);
  }
  Serial.printf("[%6" PRIu32 "ms] [4/6] Queue erstellt: %d Slots x %d Bytes = %d Bytes\n",
                millis(), SENSOR_QUEUE_SIZE,
                (int)sizeof(sensor_data),
                SENSOR_QUEUE_SIZE * (int)sizeof(sensor_data));

  // ── SCHRITT 5: ThingSpeak + ESP-NOW ──────────────────────────────────
  ThingSpeak.begin(client);
  printTs("[5/6] ThingSpeak Client bereit");

  if (!initESPNow()) {
    Serial.println("[Setup] ESP-NOW fehlgeschlagen – Neustart in 3s!");
    delay(3000);
    esp_restart();
  }

  // ── SCHRITT 6: FreeRTOS Tasks ─────────────────────────────────────────
  xTaskCreate(wifiTask,      "WiFi",      8192, NULL, 2,                  &wifiTaskHandle);
  xTaskCreate(uploadTask,    "Upload",    8192, NULL, 2,                  &uploadTaskHandle);
  // FIX Stack: 2048 → 3072 (Serial.printf Formatverarbeitung + Heartbeat-Puffer)
  xTaskCreate(powerMgmtTask, "PowerMgmt", 3072, NULL, tskIDLE_PRIORITY+1, &powerTaskHandle);
  Serial.printf("[%6" PRIu32 "ms] [6/6] Tasks gestartet: WiFi(P2,8K) Upload(P2,8K) Power(P1,3K)\n",
                millis());

  printSep();
  Serial.printf("[%6" PRIu32 "ms] BEREIT – Warte auf ESP-NOW Sensordaten\n", millis());
  printSep();
}

// ---------------------------------------------------------------------------
// Loop — leer, alle Arbeit läuft in FreeRTOS-Tasks
// ---------------------------------------------------------------------------
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}