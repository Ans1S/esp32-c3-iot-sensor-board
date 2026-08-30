# Firmware-Architektur 2

## Zielbild

Die W-Charger Station ist der dauerhaft erreichbare Kontrollpunkt. Sie betreibt
gleichzeitig ein eigenes Setup-WLAN, die Verbindung zum Heim-WLAN, ESP-NOW und
den ThingSpeak-Uploader. Die batteriebetriebenen Sensoren wachen nur kurz auf,
messen, tauschen genau ein Telemetrie-/Konfigurationspaar mit der Station aus
und gehen wieder in Deep Sleep.

```text
PCB V3/V4 Sensor
  Messen -> Telemetrie + aktuelle Config-Revision
       -> ESP-NOW -> W-Charger Station -> RAM-Queue -> HTTPS -> ThingSpeak
       <- ESP-NOW <- Intervall, Kanal, Station-MAC, Sensor-/ADC-Kalibrierung
  NVS nur bei Änderung -> Verbraucher aus -> Deep Sleep
```

`shared/lil_protocol.h` ist die einzige Schnittstelle zwischen den beiden
Firmwares. Jedes Paket enthält Magic, Protokollversion, Nachrichtentyp,
Payload-Länge, Sequenznummer und CRC32. Unbekannte oder beschädigte Pakete
werden verworfen. Änderungen am Paketformat erfordern eine neue
Protokollversion.

## Zuständigkeiten

### Station

- `config_store`: versionierte Station- und Sensordaten in NVS
- `wifi_service`: AP+STA, Reconnect, Captive DNS und mDNS
- `web_portal`: Setup-Wizard, Sensor-Provisionierung, Channel-Profile und
  responsive Statusanzeige
- `sensor_registry`: MAC-basierte Geräteverwaltung und Revisionsstand
- `espnow_gateway`: kurze Funk-Callbacks, Queue und Antworten
- `thingspeak_service`: separater FreeRTOS-Task für blockierende HTTPS-Aufrufe

Neu erkannte Sensor-MACs werden zunächst nur als nicht provisionierte Geräte
gespeichert. Sie liefern Telemetrie an das Dashboard, laden aber nichts in die
Cloud. Erst der Sensor-Assistent setzt den Provisionierungsstatus und aktiviert
optional den Upload.

Die Station speichert bis zu sechs wiederverwendbare ThingSpeak-Channel-Profile
mit Channel-ID, Read-Key und Write-Key. Die Profilzuordnung und Feldbelegung
liegen stationsseitig pro Sensor. Sechs Messgrößen (Temperatur, Feuchte, Druck,
Static IAQ, Gaswiderstand und Batterie) lassen sich unabhängig auf die
Channel-Felder 1 bis 8
abbilden. Dadurch können mehrere Sensoren denselben Channel mit gemeinsamen
Zugangsdaten und unterschiedlichen Feldern nutzen. Read- und Write-Key sind
optional; Uploads verwenden nur den Write-Key. Nicht unterstützte oder als
fehlerhaft markierte Messwerte werden nicht übertragen.

Der Cloud-Status wertet nicht nur den HTTP-Status aus: Ein Upload gilt erst mit
einer positiven, von ThingSpeak gelieferten Entry-ID als erfolgreich.

ESP-NOW wird während eines Cloud-Uploads nicht abgeschaltet. Eine volle
Funkqueue verwirft kontrolliert das älteste Cloud-Ereignis und zählt den
Verlust im Dashboard, statt den Empfangs-Callback zu blockieren.

### Sensor

- `hardware_profile`: ausschließlich die V3/V4-Pins und Polaritäten
- `power_controller`: Sensor- und ADC-Leistungspfade
- `environmental_sensor`: gemeinsame, erweiterbare I²C-Sensor-Fassade
- `bme280_driver`: BME280 im sparsamen Forced Mode
- `bme680_driver`: Bosch BSEC2 Static IAQ mit passendem 3,3-V-ULP-Profil,
  durchgehendem 5-Minuten-Takt, Deep Sleep zwischen den Messungen und
  persistentem Lernzustand; kein Wechsel des BSEC-Betriebsmodus
- `bsec_state_store`: CRC-gesicherte, verschleißarme IAQ-Zustandssicherung
- `adc_reader`: gefilterte, kalibrierte Millivolt-Messung
- `espnow_transport`: bekannter Kanal zuerst, Recovery-Scan bei Ausfällen
- `sensor_config_store`: NVS-Schreiben nur bei tatsächlicher Änderung
- `sleep_controller`: definierter Power-down und Timer-Deep-Sleep

Ein Resetauftrag bleibt auf der Station ausfallsicher vorgemerkt. Erst wenn der
Sensor die betreffende Revision bei einem späteren Kontakt bestätigt, wird der
Auftrag gelöscht.

## Persistenz und Fehlerverhalten

- WLAN- und ThingSpeak-Zugangsdaten liegen in NVS, nicht im Quellcode.
- Sensorintervall, I²C-Sensortyp, Temperatur-Offset,
  Batteriespannungs-Korrekturfaktor, Station-MAC und letzter Funkkanal liegen im
  Sensor-NVS.
- Boot-Zähler, Sequenz, Fehlerzähler und Resetbestätigung liegen im RTC-RAM.
- BSEC-Zustand und kontinuierliche Algorithmuszeit liegen im RTC-RAM; bei
  steigender Genauigkeit und danach alle sechs Stunden folgt ein NVS-Checkpoint.
- Nach zwei erfolglosen Wakeups scannt ein Sensor die Kanäle 1 bis 13 erneut.
- Die Cloud-Queue liegt bewusst nur im RAM. Ein Stationsneustart kann daher
  noch nicht hochgeladene Messungen verlieren.

## Sicherheitsgrenzen dieses Stands

- ThingSpeak verwendet TLS mit Zertifikatsprüfung.
- Das Setup-AP verwendet beim Erststart das dokumentierte WPA2-Passwort
  `W-Charger-Setup`; die Oberfläche bietet später dessen Änderung an.
- Schreibende Webanfragen benötigen zusätzlich ein zufälliges CSRF-Token.
- Die HTTP-Weboberfläche bietet einen optionalen Passwortschutz. Das
  gerätegebundene SHA-256-Derivat wird in NVS gespeichert; eine erfolgreiche
  Anmeldung setzt ein zufälliges HttpOnly-Session-Cookie, das bei jedem Neustart
  ungültig wird. Fehlversuche werden begrenzt.
- Die ThingSpeak-API-Keys werden auf ausdrücklichen Produktwunsch im lokalen
  Webinterface im Klartext zurückgegeben. Mit aktiviertem Passwort sind deshalb
  auch alle lesenden Konfigurations-, Status-, Historien- und ThingSpeak-APIs
  anmeldungsgebunden. Ohne Passwort bleibt der lokale Zugriff bewusst offen.
- CRC32 erkennt Übertragungsfehler, authentifiziert aber keinen Sensor.
- Die automatische Aufnahme neuer Sensor-MACs ist für die Inbetriebnahme offen.

Vor einer Weitergabe an Dritte fehlen deshalb noch ein zeitlich begrenzter,
physisch bestätigter Pairing-Modus, individuelle ESP-NOW-Schlüssel,
Replay-Schutz über Neustarts sowie HTTPS für die lokale Administration. Der
optionale HTTP-Login ist keine Transportverschlüsselung; gegen aktives
Mitschneiden im gleichen Netz wäre zusätzlich TLS erforderlich.

## Firmware-Updates

Dieser Stand verteilt Konfigurationsupdates beim nächsten Sensor-Wakeup. Eine
Übertragung neuer Firmware-Binärdateien über ESP-NOW ist noch nicht
implementiert. Die Partitionen von S3 und C3 besitzen bereits je zwei
OTA-App-Slots. Der nächste Schritt sollte ein signiertes Manifest, gechunkte
Übertragung mit Resume, SHA-256-Prüfung, Versions-/Hardwareprüfung und
Rollback nach fehlgeschlagenem Selbsttest sein. Ohne Signaturprüfung darf die
Funktion nicht als Endkunden-OTA angeboten werden.
