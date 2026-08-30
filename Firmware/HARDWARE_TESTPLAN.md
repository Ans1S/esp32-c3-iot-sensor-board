# Hardware-Testplan: PCB V3 zuerst, PCB V4 danach

Die erfolgreichen Compiler-Builds prüfen Schnittstellen und Speichergrenzen,
ersetzen aber keinen Test auf echter Hardware. Ergebnisse sollten mit Datum,
Firmware-Commit, Platinenrevision und Messgerät protokolliert werden.

## 1. Station allein

1. Station flashen und seriell prüfen, ob Setup-SSID, AP-IP und Zugangsdaten
   ausgegeben werden.
2. Ohne Heim-WLAN `192.168.4.1` öffnen und Login testen.
3. Heim-WLAN speichern; nach Neustart sowohl Setup-AP als auch `.local`/LAN-IP
   prüfen.
4. Falsches WLAN-Passwort setzen, mindestens zwei Reconnect-Zyklen abwarten und
   sicherstellen, dass das Setup-AP erreichbar bleibt.
5. Station neu starten und prüfen, dass alle Einstellungen erhalten bleiben.

## 2. Sensor PCB V3

1. V3-Environment flashen. Ein ungekoppelter Sensor muss die Station ohne
   eingetragene MAC oder Funkkanal finden.
2. Dashboard-Werte mit einem Referenzthermometer und Hygrometer plausibilisieren.
3. Batteriespannung gleichzeitig mit einem Multimeter messen. Die Firmware
   verwendet gemäß PCB-Datei 100 kΩ zu 330 kΩ, also Faktor 1,303. Abweichung
   protokollieren und erst danach einen Kalibrierfaktor festlegen.
4. Intervall im Webinterface zunächst auf 1 Minute setzen. Prüfen, dass
   `appliedRevision` spätestens beim übernächsten Kontakt der Stationsrevision
   entspricht.
5. Station während einer Übertragung ausschalten. Der Sensor muss trotzdem in
   Deep Sleep gehen und später wieder verbinden.
6. WLAN-Kanal des Routers ändern. Nach zwei erfolglosen Wakeups muss der Sensor
   die Station per Recovery-Scan wiederfinden.
7. Sensor-Reset vormerken, Station einmal während der Antwort ausschalten und
   prüfen, dass der Auftrag bis zur Sensorbestätigung bestehen bleibt.

## 3. BME680 und IAQ auf PCB V3

1. BME680 an die geschaltete I²C-Versorgung anschließen, den Sensor neu starten
   und im Hinzufügen-Assistenten prüfen, dass `BME680` erkannt wird. Danach den
   Typ explizit auf BME680 und das Intervall zunächst auf 5 Minuten setzen.
2. Im seriellen Log prüfen, dass BSEC vom ersten Start an ausschließlich mit
   `ULP/300 s` läuft. Es darf weder einen LP-/3-Sekunden-Zyklus noch später einen
   Wechsel der Sample Rate geben. Eine dauerhafte „Direktmessung“ ist ein Fehler.
3. Im Dashboard müssen Temperatur, Feuchte, Druck, Gaswiderstand und nach der
   BSEC-Stabilisierung Static IAQ erscheinen. Der IAQ beginnt mit Genauigkeit
   0/„unzuverlässig / lernt“; die anfängliche ULP-Stabilisierung dauert ungefähr
   20 Minuten. In dieser Phase darf ThingSpeak noch keinen IAQ-Wert erhalten.
4. Den Sensor mindestens 24 Stunden in einem normal genutzten Raum betreiben
   und beobachten, ob die Genauigkeit 1, 2 und schließlich 3 erreicht. Lüften
   muss den IAQ mit Verzögerung verbessern; typische Raumluftbelastung muss ihn
   verschlechtern. Keine Flüssigkeit oder konzentrierten Dämpfe direkt an den
   Sensor bringen.
5. Nach erreichter Genauigkeit 2 oder 3 die Batterie kurz trennen. Nach dem
   Neustart muss ein gespeicherter BSEC-Zustand geladen werden; ein kompletter
   Neubeginn der Langzeitkalibrierung weist auf einen Persistenzfehler hin.
6. Nach thermischem Einschwingen Temperatur und Feuchte neben einem bekannten
   Referenzgerät vergleichen. Den Temperatur-Offset nur dann in kleinen
   Schritten ändern und mindestens drei weitere Zyklen beurteilen.
7. Berichtsintervall auf 30 Minuten setzen. Der BME680 muss weiterhin alle fünf
   Minuten messen, ESP-NOW/ThingSpeak aber nur alle 30 Minuten verwenden.
8. Zwischen BME680 und BME280 wechseln und die Auswahl in der Station ändern.
   Falsche Typauswahl muss als Sensor-Lesefehler/Typabweichung sichtbar werden;
   `Automatisch` muss beide Chip-IDs an 0x76 und 0x77 erkennen.
9. Sensorrail während Deep Sleep an PCB V3 und V4 direkt messen. Sie muss 0 V
   betragen. Nach mindestens zehn komplett stromlosen Sensorzyklen muss die
   IAQ-Genauigkeit weiterlernen und darf nicht jedes Mal bei einem frischen
   Algorithmuszustand beginnen.
10. Im Assistenten den IAQ-Lernzustand zurücksetzen. Nach dem nächsten Kontakt
   muss die Genauigkeit bewusst wieder bei 0 starten und anschließend erneut
   steigen.

## 4. ThingSpeak

1. User API Key in der Station hinterlegen und für den Sensor einen privaten
   Channel erzeugen.
2. Prüfen, dass Channel-ID und Write-Key nach Stationsneustart erhalten bleiben.
3. Alle acht Felder auf richtige Einheit und Zuordnung prüfen. Die Batterie wird
   in Volt, der Gaswiderstand in Ohm und der Druck in hPa übertragen.
4. Internet für mehrere Messungen trennen und wiederherstellen. Den Zähler
   `droppedJobs` beobachten; die aktuelle Queue ist nicht reboot-persistent.

## 5. Energieprofil PCB V3

Mit einem Power-Analyzer je mindestens 20 Zyklen messen:

- Wakeup bis Sensor-Power-on
- BME280-Messzeit
- BME680-ULP-Messung einschließlich Heizphase
- ESP-NOW-Sendezeit bei bekanntem Kanal
- BME680-Zwischenzyklus ohne ESP-NOW bei längerem Berichtsintervall
- Recovery-Scan als Worst Case
- Deep-Sleep-Strom
- Ladungsverbrauch pro normalem Messzyklus

Erst aus Ladung pro Zyklus, real gemessener Akkukapazität und Selbstentladung
eine Laufzeit ableiten. Die bisherigen Laufzeitangaben im Haupt-README sind
keine Abnahmewerte.

## 6. Batteriespannungs-Kalibrierung

1. Batterie ohne USB-Verfälschung gleichzeitig mit einem kalibrierten
   Multimeter und über das Dashboard messen.
2. Multimeterwert im Sensor-Assistenten eingeben und speichern. Nach dem
   nächsten Sensor-Wakeup muss der neue Faktor übertragen sein.
3. Bei mindestens drei Spannungen, beispielsweise 4,15 V, 3,70 V und 3,25 V,
   erneut vergleichen. Einpunktkalibrierung darf den Gain korrigieren, aber
   keine deutliche Nichtlinearität oder falsche Widerstandsbestückung verdecken.
4. Kalibrierung auf Standard zurücksetzen und prüfen, dass Faktor 1,0000 wieder
   aktiv wird.

## 7. Übernahme auf PCB V4

1. Vor dem Bestücken GPIO6/`ADC_EN` und GPIO10/`SENSOR_PWR_EN` gegen Schaltplan
   und Leiterplatte durchklingeln.
2. Im Sleep muss GPIO6 low sein. Der Spannungsteiler darf dann keinen messbaren
   Dauerstrom aus der Batterie ziehen.
3. GPIO10 ist auf V4 aktiv-low: low schaltet die Sensorrail ein, high aus.
4. Batteriespannung bei etwa 4,2 V, 3,7 V und 3,2 V vergleichen. V4 verwendet
   100 kΩ zu 150 kΩ, also Faktor 1,667.
5. Brownout-, USB-an/ab- und Lade-/Batterie-Umschalttests durchführen.
6. Erst nach bestandenem Pin-, Spannungs- und Stromtest das V4-Environment für
   längere Feldtests verwenden.
