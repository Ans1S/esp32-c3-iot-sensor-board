# W-Charger Station Firmware

Die Station läuft auf dem vorhandenen Seeed XIAO ESP32-S3. Sie hält WLAN und
ESP-NOW gleichzeitig aktiv, sodass das Webinterface erreichbar bleibt und
Sensoren nicht während eines Cloud-Uploads ausgesperrt werden.

## Erster Start

1. `pio run -e station_s3 -t upload` (der Station-Upload löscht bewusst auch
   die lokale NVS-Konfiguration und startet dadurch immer als Neuinstallation)
2. Mit dem WLAN `W-Charger-XXXXXX` verbinden. Das initiale WLAN-Passwort ist
   `W-Charger-Setup`.
3. Normalerweise öffnet sich das Captive Portal automatisch. Falls nicht,
   `http://192.168.4.1/` öffnen. Beim unkonfigurierten Erststart ist kein Login
   nötig.
4. Dem Einrichtungsassistenten folgen: Messintervall, optional ThingSpeak,
   Sensoren und Heim-WLAN einstellen. Am Ende kann ein optionales, empfohlenes
   Website-Passwort vergeben werden.
5. Die Zusammenfassung bestätigen. Die Station speichert alles und startet
   danach neu.

Ein serieller Monitor ist für die Einrichtung nicht erforderlich. Das
Setup-WLAN bleibt bis zur ersten erfolgreichen Verbindung zum Heimnetz aktiv
und wird danach automatisch abgeschaltet. Es startet erst wieder bei einer
Neuinstallation oder nach Werkseinstellungen. Im Heimnetz ist die Station unter
`http://w-charger.local/` erreichbar, sofern der Client mDNS unterstützt.
Abhängig vom Router funktioniert zusätzlich `http://w-charger/`.

Nach der Einrichtung trennt das responsive Dashboard neu erkannte von bereits
eingerichteten Sensoren. Ein neues Gerät wird automatisch eingeblendet und muss
einmal im dreistufigen Assistenten bestätigt werden. Der Assistent vergibt Name
und Intervall, wählt
den angeschlossenen I²C-Sensor (automatisch, BME280, BME680 oder deaktiviert),
stellt beim BME680 optional die Temperaturkorrektur ein, wählt ein gespeichertes
ThingSpeak-Channel-Profil und ordnet die Messwerte den Feldern 1 bis 8 zu. Erst
danach erscheint der Sensor mit letztem Kontakt,
Messwerten, Batterie, Funkqualität und einem umschaltbaren lokalen Messverlauf
im normalen Dashboard. Zur Auswahl stehen Intervalle ab einer Minute sowie
eine benutzerdefinierte Zeit bis 24 Stunden. Der Stationsassistent ist nur bei
einer Neuinstallation oder nach dem Zurücksetzen auf Werkseinstellungen
erreichbar. Im laufenden Betrieb werden Änderungen direkt unter
**Einstellungen** vorgenommen.

## Module

- `config_store`: persistente Station- und Sensorkonfiguration in NVS sowie
  lokale Messhistorien in LittleFS
- `wifi_service`: einmaliger Setup-AP, Heim-WLAN-Reconnect, DNS und mDNS
- `web_portal`: Dashboard, Sensor-Assistent, Channel-Profile, WLAN und Reset
- `espnow_gateway`: validiertes, versioniertes Telemetrie-/Konfigurationsprotokoll
- `sensor_registry`: Sensoren nach MAC-Adresse statt gehardcodierter IDs
- `thingspeak_service`: HTTPS-Upload, Kontoverwaltung und automatisches
  Erstellen privater Channels

Bis zu sechs ThingSpeak-Channel-Profile können zentral gespeichert und von
mehreren Sensoren wiederverwendet werden. Jedes Profil enthält einen Namen,
die Channel-ID und optional Read- sowie Write-API-Key. Ein Sensor übernimmt
Channel-ID und Write-Key durch Auswahl des Profils; die Schlüssel müssen dabei
nicht erneut eingetippt werden. Alternativ kann im Sensor-Assistenten ein
vorhandener Channel eingetragen oder mit dem User API Key automatisch ein
neuer privater Channel angelegt werden.

Die Zuordnung zu den ThingSpeak-Feldern ist pro Sensor frei konfigurierbar.
Temperatur, Luftfeuchte, Luftdruck, Static IAQ, Gaswiderstand und
Batteriespannung können jeweils auf `Feld 1` bis `Feld 8` oder auf `Nicht
senden` gestellt werden. Mehrere Sensoren
können denselben Channel und Write-Key verwenden und dabei unterschiedliche
Felder belegen. Innerhalb eines Sensors verhindert die Oberfläche doppelte
Feldbelegungen. Ein automatisch neu angelegter Channel startet mit Temperatur,
Feuchte, Druck, IAQ, Batterie und Gaswiderstand auf den Feldern 1 bis 6. Beim
BME680 übernimmt der Assistent diese Zuordnung direkt; sie bleibt vor dem
Speichern frei änderbar.

Beim BME680 zeigt das Dashboard neben dem IAQ die BSEC-Genauigkeit von
„Hintergrundlernen“ bis „hoch“ sowie den Roh-Gaswiderstand in kΩ. BSEC läuft vom
ersten Start an ausschließlich energiesparend im ULP-Modus und misst intern alle
fünf Minuten; ESP-NOW und ThingSpeak folgen strikt dem gewählten längeren
Berichtsintervall. Die anfängliche ULP-Stabilisierung von ungefähr 20 Minuten
wird transparent angezeigt, ohne Betriebsmodus oder Funkintervall zu wechseln.

Neue, noch nicht eingerichtete Sensoren werden automatisch auf der Übersicht
eingeblendet; der zusätzliche Hinzufügen-Button ist nur noch ein direkter
Sprung dorthin. Bereits eingerichtete Sensoren können aus der lokalen Station
gelöscht werden. Solange ein gelöschter Sensor weiter sendet, erscheint er beim
nächsten Kontakt erwartungsgemäß wieder als neues Gerät.

Im Bearbeiten-Assistenten stehen außerdem eine transparente BME680-
Erstinbetriebnahme, das bewusste Zurücksetzen des IAQ-Lernzustands und eine
optionale Batteriespannungs-Kalibrierung mit Multimeterwert zur Verfügung. Das
Die Station speichert unabhängig von einem geöffneten Browser für jeden Sensor
eine rollierende 24-Stunden-Historie. Die 48 festen 30-Minuten-Fenster enthalten
jeweils die neueste empfangene Messung dieses Fensters: Temperatur, Feuchte,
Druck, IAQ samt Accuracy, Gaswiderstand und Batteriespannung. Ein Handy oder ein
anderer Browser lädt diese Daten vollständig über die lokale Stations-API; ein
Neuladen der Seite ist zum Sammeln nicht erforderlich. Die Historie übersteht
auch einen normalen Neustart der Station. Ein Dropdown wechselt den beschrifteten
Plot zwischen allen vom Sensor unterstützten Größen; Zeit, Wertebereich und
Einheit werden auf den Achsen angezeigt. Tatsächlich fehlende 30-Minuten-Fenster
werden nicht durch eine erfundene Verbindungslinie überbrückt.

Im Captive Portal und in den späteren Stationseinstellungen können Channel-ID
sowie Read- und Write-API-Key hinterlegt werden. Der Read-Key ist nur für den
Lesezugriff auf private Channels gedacht. Für das Senden verwendet die Station
den Write-Key. Der User API Key wird ausschließlich benötigt, wenn W-Charger
selbst einen neuen ThingSpeak-Channel anlegen soll. Alle API-Keys werden auf
ausdrücklichen Wunsch im lokalen Webinterface im Klartext angezeigt und nach
einem Neustart wieder in die Felder geladen. Der Cloud-Upload bleibt bis zur
ausdrücklichen Aktivierung im Sensor-Assistenten ausgeschaltet.

Mit gespeichertem User API Key lädt die Einstellungsseite die ThingSpeak-
Channels automatisch. Channels lassen sich dort erstellen, umbenennen, mit
Beschreibung, Feldern, Tags, Metadaten, Sichtbarkeit und Ortsdaten bearbeiten,
leeren oder löschen. Direkte Channel- und Feed-URLs werden angezeigt. Ein neu
erkannter Sensor kann im Assistenten einen dieser Channels auswählen;
Channel-ID und Write-Key werden dann automatisch in ein lokales Sensor-Profil
übernommen.

Das Statusfeld auf der Übersicht meldet erst dann **Verbunden**, wenn
ThingSpeak einen Schreibvorgang mit einer echten Entry-ID bestätigt hat. Bei
einem falschen Write-Key, einem HTTP-/TLS-Fehler oder fehlenden gültigen
Messwerten wird stattdessen eine konkrete Fehlermeldung angezeigt. Die lokale
24-Stunden-Historie wird ausschließlich auf der Station gehalten und ist damit
nicht von ThingSpeak oder dem Speicher eines einzelnen Browsers abhängig. Ein
vollständiger Firmware-Upload mit Flash-Löschung sowie das Zurücksetzen auf
Werkseinstellungen löschen diese Historie bewusst.

## Sicherheitsmodell dieses Stands

- Heim-WLAN- und ThingSpeak-Zugangsdaten liegen nur in NVS.
- Das Setup-WLAN nutzt WPA2 mit dem initialen Passwort `W-Charger-Setup`.
- Die lokale Weboberfläche kann bereits im Erstsetup oder später in den
  Einstellungen mit einem mindestens acht Zeichen langen Passwort geschützt
  werden. Geschützt sind Dashboard, Konfiguration, Historie, WLAN-Scan und alle
  schreibenden sowie ThingSpeak-bezogenen APIs.
- Das Website-Passwort liegt nicht im Klartext vor, sondern als gerätegebundener
  SHA-256-Hash. Erfolgreiche Anmeldungen erhalten ein zufälliges, nur bis zum
  nächsten Stationsneustart gültiges HttpOnly-Session-Cookie; wiederholte
  Fehlversuche werden kurzzeitig gedrosselt.
- ThingSpeak-API-Keys bleiben im angemeldeten Webinterface auf ausdrücklichen
  Produktwunsch sichtbar. Ohne aktiviertes Website-Passwort kann deshalb jedes
  Gerät im lokalen Netz diese Schlüssel lesen und Einstellungen ändern; das
  Dashboard weist sichtbar darauf hin.
- ThingSpeak-Aufrufe verwenden HTTPS mit geprüftem DigiCert-Root-Zertifikat.
- ESP-NOW-Pakete besitzen Magic, Protokollversion, feste Längen und CRC32.

CRC32 schützt vor Übertragungsfehlern, ist aber keine kryptografische
Authentifizierung. Individuelle ESP-NOW-Schlüssel und ein physisch bestätigter
Pairing-Ablauf sind deshalb als nächster Härtungsschritt vorgesehen.

Die lokale Weboberfläche wird weiterhin über HTTP ausgeliefert. Der
Passwortschutz verhindert unbeabsichtigten Zugriff durch andere Teilnehmer im
Heimnetz, ersetzt aber keine Transportverschlüsselung gegen aktives Mitschneiden
im selben Netz.

Die verwendete 8-MB-Partition besitzt zwei OTA-fähige App-Slots mit jeweils
rund 3,19 MiB. Das schafft Reserve, implementiert aber noch keinen
Firmware-Upload. Der aktuelle Stand verteilt ausschließlich Einstellungen an
die Sensoren; der sichere Binär-OTA-Ablauf ist in `../ARCHITECTURE.md`
abgegrenzt.
