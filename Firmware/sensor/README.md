# W-Charger Sensor Firmware

Eine gemeinsame Firmware deckt PCB V3 und V4 ab. Der Unterschied steckt nur im
Build-Environment und im `hardware_profile`.

```text
pio run -e sensor_pcb_v3 -t upload   # vorhandene Testplatine
pio run -e sensor_pcb_v4 -t upload   # neue Platine
```

Es müssen keine WLAN-Daten, Stations-MAC, Funkkanäle oder Messintervalle in den
Quellcode geschrieben werden. Ein neuer Sensor sucht die Station einmalig auf
allen 13 ESP-NOW-Kanälen. Während des ersten zehnminütigen Pairing-Fensters
wiederholt er diesen vollständigen Broadcast-Scan alle zehn Sekunden. Das gilt
bis der Sensor im Dashboard tatsächlich hinzugefügt (`provisioned=true`) wurde;
eine bloße Stationsantwort beendet den Suchmodus nicht. Nach dem Hinzufügen
speichert er MAC und Kanal in NVS. Ein eingerichteter Sensor bleibt danach auf
diesem gespeicherten Kanal.

Ein neu geflashtes Firmware-Image wird über seinen ELF-SHA-Fingerabdruck
erkannt. Beim ersten Start dieses Images werden die alte Stationszuordnung und
der IAQ-Startzustand gelöscht, auch wenn das Upload-Werkzeug die NVS-Partition
nicht mit gelöscht hat. Neustarts, Deep Sleep und vollständige Stromausfälle
mit demselben Firmware-Image behalten die gespeicherten Zustände dagegen bei.
Pro fälliger Übertragung versucht er die Station genau viermal zu erreichen:
mit dem normalen Wert von 13 dBm, danach mit 16 dBm und 19 dBm und zuletzt mit
der maximalen Sendeleistung von 21 dBm. Bleiben alle Antworten aus, beginnt
kein zusätzlicher Funkzyklus; der Sensor schläft wieder für das konfigurierte
Messintervall und versucht es beim nächsten fälligen Kontakt erneut.

Der Sensortyp wird nicht mehr in eine eigene Firmware einkompiliert. Beim
Hinzufügen oder späteren Bearbeiten wählt man auf der Station **Automatisch**,
**BME280**, **BME680** oder **Kein Umweltsensor**. Die Einstellung sowie die
BME680-Temperaturkorrektur werden mit der nächsten ESP-NOW-Antwort übertragen.
Im Automatikmodus werden die Bosch-Chip-IDs an `0x76` und `0x77` geprüft.

Bei einem normalen BME280-Zyklus passiert nur:

1. Sensorversorgung einschalten und den BME280 im Forced Mode mit 1×
   Oversampling lesen.
2. Batteriespannung lesen; auf V4 den Teiler nur für die Messung einschalten.
3. Ein validiertes Telemetriepaket per ESP-NOW senden.
4. Kurz auf die Konfigurationsantwort der Station warten.
5. Nur bei geänderter Revision die neue Konfiguration in NVS schreiben.
6. Sensor- und ADC-Pfad sicher abschalten und in Deep Sleep gehen.

Die BME280-Konfiguration bleibt bewusst bei Boschs sparsamem
Wettermonitor-Profil: Forced Mode, Temperatur/Druck/Feuchte jeweils 1× und IIR-
Filter aus. Für langsame Raumwerte verbessert höheres Oversampling die
praktische Aussagekraft kaum, verlängert aber Mess- und Aktivzeit. Die Station
weckt diesen Sensortyp deshalb ausschließlich im gewählten Berichtsintervall.

## Hardwareprofile

| Funktion | PCB V3 | PCB V4 |
|---|---:|---:|
| ADC | GPIO3 | GPIO3 |
| ADC-Freigabe | dauerhaft aktiv | GPIO6, aktiv-high |
| Sensorversorgung | GPIO10, aktiv-high | GPIO10, PMOS aktiv-low |
| I²C SDA / SCL | GPIO5 / GPIO4 | GPIO5 / GPIO4 |
| Batterieteiler | empirisch validierter Softwarefaktor 1,67 | 100 kΩ / 150 kΩ, Faktor 1,667 |

Der Sensor kann von der Station aus beim nächsten Kontakt logisch auf
Werkseinstellungen gesetzt werden. In der Weboberfläche ist das Intervall von
einer Minute bis 24 Stunden frei konfigurierbar, um kurze Tests ebenso wie
einen energiesparenden Langzeitbetrieb abzudecken.

## BME680 und Indoor Air Quality

Der BME680 läuft mit Boschs BSEC2-Algorithmus und verwendet für ein stationäres
Raumgerät den **Static IAQ**. Ein selbst aus dem Gaswiderstand berechneter Index
wäre nicht gleichwertig: Feuchtekompensation, Sensoralterung, Drift und die
adaptive Grundlinie gehören zum Algorithmus. Der Index reicht von 0 (saubere
Luft) bis 500 (stark belastet); die zugehörige Genauigkeit 0–3 erscheint direkt
im Dashboard. Solange sie 0 ist, steht dort „wird kalibriert“ und die Station
sendet den IAQ noch nicht an ThingSpeak. Temperatur, Feuchte, Luftdruck und
Gaswiderstand bleiben trotzdem verfügbar.

Erstinbetriebnahme und Langzeitbetrieb verwenden bewusst dieselbe Routine:

- vom ersten Start an und über die gesamte Lebensdauer läuft ausschließlich
  `BSEC_SAMPLE_RATE_ULP` mit intern genau einer BME680-Messung alle fünf Minuten;
- es gibt keinen Wechsel zwischen LP und ULP und damit keinen Sprung der
  gelernten BSEC-Grundlinie durch einen Betriebsmoduswechsel;
- Sensorversorgung und ESP32-C3 sind zwischen den Messungen abgeschaltet bzw.
  im Deep Sleep;
- Funkübertragung und ThingSpeak halten strikt das eingestellte
  Berichtsintervall ein, etwa 10, 30 oder 60 Minuten;
- BSEC-Zustand und logische Zeit bleiben im RTC-RAM; ein Flash-Checkpoint wird
  bei verbesserter Genauigkeit und danach höchstens alle sechs Stunden erzeugt;
- nach einem Batteriewechsel wird der letzte Flash-Zustand wiederhergestellt,
  statt die adaptive Grundlinie vollständig zu verlieren.

Die Firmware lädt ausschließlich Boschs zur 3,3-V-Sensorversorgung passenden
ULP-Konfigurationsblob. Die direkte Bosch-Sensor-API bleibt ein
Verfügbarkeits-Fallback für Temperatur, Feuchte, Druck und Roh-Gaswiderstand.
Sie erzeugt absichtlich **keinen** eigenen IAQ-Ersatzwert: Nur ein tatsächlich
von BSEC gelieferter Static IAQ wird als IAQ markiert oder hochgeladen.

Die I²C-Sensorrail wird auf **PCB V3 und V4 vollständig abgeschaltet**. Der
BME680 selbst behält dabei keinen Zustand. Deshalb wird er nach jedem Wakeup
neu initialisiert, während der zuvor serialisierte BSEC-Lernzustand und eine
kontinuierliche logische Zeit vor dem Ausschalten gesichert und beim nächsten
Start wieder eingespielt werden. Das entspricht Boschs vorgesehenem
`getState()`-/`setState()`-Ablauf für Systemabschaltungen.

Die anfängliche ULP-Stabilisierung dauert typischerweise ungefähr 20 Minuten
und ist kein Versprechen auf endgültige IAQ-Genauigkeit 3. BSEC lernt den
Gas-/IAQ-Pfad anhand wechselnder Luftzustände im Hintergrund weiter. Der
Normalbetrieb bleibt dabei durchgehend aktiv; ein gesonderter Schnellstart mit
erhöhter Heiz- und Funklast existiert nicht.

Bei explizitem BME680 muss das Berichtsintervall fünf Minuten oder ein
Vielfaches davon sein. Der vorgegebene Temperatur-Offset von `0,466 °C` stammt
aus Boschs ULP-Beispiel und kann pro Einbauort auf der Station korrigiert werden.
Er sollte erst nach einem Vergleich im thermisch eingeschwungenen Gehäuse
geändert werden.

Die Implementierung basiert auf Boschs offizieller
[BME68x Sensor API](https://github.com/boschsensortec/BME68x_SensorAPI), der
[BME68x Arduino-Bibliothek](https://github.com/boschsensortec/Bosch-BME68x-Library)
und [BSEC2](https://github.com/boschsensortec/Bosch-BSEC2-Library). Die
BME680-Grenzen, IAQ-Skala und ULP-Leistungswerte stehen im
[BME680-Datenblatt](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme680-ds001.pdf).
BSEC2 enthält eine proprietäre Binärbibliothek; für Nutzung und Weitergabe gilt
zusätzlich die [Bosch-BSEC-Lizenz](https://github.com/boschsensortec/Bosch-BSEC2-Library/blob/master/LICENSE.md).

Beim Build wird die festgehaltene BSEC2-2.1.5-Quelle automatisch mit einem
kleinen Timing-Patch versehen. BSEC2 startet im Forced Mode zunächst nur die
Messung; der Patch wartet die konfigurierte TPH-/Heizdauer plus eine
konservative Marge ab und fragt `NO_NEW_DATA` bis zu 500 ms erneut ab. Der
Treiber akzeptiert nur einen tatsächlich neuen BSEC-Ausgabedatensatz und hält
die Sensorversorgung dafür insgesamt höchstens fünf Sekunden eingeschaltet.
Liegt der nächste ULP-Termin weiter in der Zukunft, wird die Platine samt
BME680 wieder vollständig abgeschaltet und exakt zu diesem Termin geweckt;
sie wartet also nicht fünf Minuten mit eingeschaltetem Sensor. Das ist für die
hier verwendete, nach jedem Zyklus abgeschaltete Sensorversorgung erforderlich.
Die heruntergeladene Bosch-Bibliothek selbst wird nicht in das Repository
kopiert.

Falls ein Zyklus trotzdem nur eine direkte Bosch-Rohmessung liefert, bleibt der
letzte echte BSEC-IAQ-Wert auf der Stationsseite sichtbar und wird als nicht
frisch gekennzeichnet. Dieser alte Wert wird weder als neuer Verlaufspunkt noch
zu ThingSpeak hochgeladen. Der nächste fällige BSEC-Zyklus versucht erneut,
einen frischen IAQ-Wert zu erzeugen.

Die Station zeigt die anfängliche ULP-Stabilisierung und den späteren
Hintergrund-Lernstatus an. Mess-, Funk- und Cloud-Betrieb verwenden von Beginn
an dieselbe ULP-Routine. Die BSEC-Genauigkeit 0–3 bleibt sichtbar und kann sich
durch unterschiedliche Luftzustände später weiter verbessern.

## Erkennung und erneutes Hinzufügen

Ein nicht eingerichteter oder auf der Station gelöschter Sensor meldet sich
zehn Minuten lang alle zehn Sekunden. Um Sensorheizer und ADC dabei nicht
unnötig zu betreiben, wird sein Mess-Snapshot nur alle fünf Minuten erneuert.
Nach zehn Minuten geht er zwischen den Anmeldeversuchen jeweils fünf Minuten in
Deep Sleep. Beim erfolgreichen Hinzufügen beginnt der ausgewählte BME680 mit
einem frischen Lernzustand direkt im ULP-Modus. Der Sensor
speichert den Provisionierungsstatus selbst dauerhaft im NVS. Die Station hält
ihn während der Funkbestätigung bereits in der eingerichteten Liste; verspätete
Pairing-Pakete aus der Zeit vor dieser Bestätigung können ihn nicht wieder als
neues Gerät markieren.

## Treiber erweitern

`environmental_sensor` ist nur die gemeinsame Fassade. Messdaten liegen im
neutralen `EnvironmentalReading`; BME280, BME680 und BSEC-Persistenz besitzen
eigene Header-/CPP-Dateien. Ein zukünftiger I²C-Sensor benötigt dadurch einen
neuen Treiber, einen neuen Wert in `EnvironmentalSensorType` und einen Eintrag
in `beginType()` – Funk, ADC, Deep Sleep und Konfigurationsspeicher bleiben
unverändert.

## Batteriespannung kalibrieren

`analogReadMilliVolts()` nutzt bereits die ADC-Kalibrierdaten des ESP32-C3.
Zusätzlich kann pro Sensor ein Multimeter-Referenzwert eingegeben werden. Die
Station berechnet daraus einen Einpunkt-Verstärkungsfaktor, der die Toleranzen
des externen Spannungsteilers und den verbleibenden Messfehler korrigiert:

`neuer Faktor = bisheriger Faktor × Referenzspannung / letzter Messwert`

Der zulässige Bereich 0,7–1,3 verhindert versehentliche extreme Werte. Die
Kalibrierung korrigiert den Gain unter der sinnvollen Annahme eines
Nullpunktes bei 0 V; für eine echte Zwei-Punkt-Kennlinie wären zwei präzise
Referenzspannungen nötig. ADC- und Spannungsteilerpfad werden weiterhin nur für
die wenigen Millisekunden der Messung eingeschaltet.
