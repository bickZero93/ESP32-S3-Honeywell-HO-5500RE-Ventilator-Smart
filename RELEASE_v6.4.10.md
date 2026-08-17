# Release Notes – v6.4.10

## ESP32-S3 Honeywell HO-5500RE Smart

`6.4.10-FINAL-UI-MANUAL-OSC`

Diese Version ist ein großes Stabilitäts-, Mess- und UI-Update gegenüber der
bisher veröffentlichten `5.0.0-FINAL`.

### Highlights

- 18 vollständige Zustandsprofile statt 9
- Rich-CDF-Auswertung mit 50 Verteilungsmerkmalen
- phasenentkoppelte ADC-Messung gegen Multiplex-Aliasing
- transition-balanced Training und unabhängiger Holdout
- separates OSC-LINEAR-Modell für Drehen
- Normal-Modus für den typischen Alltagseinsatz priorisiert
- JSON-/Rohhistogramm-Export für Offline-Analyse
- robustere persistente Kalibrierung und Checkpoints
- neues Webinterface mit getrennten Bereichen Steuerung/Einstellungen
- vollständige manuelle Web-Statuskorrektur für:
  - Aus / Speed 1–3
  - Normal / Breeze / Nacht
  - Drehen AUS/AN
- LED-Abgleich direkt unter dem Sleep-Timer
- ESP32/OTA und Kalibrierung/Backup aufgeräumt in den Einstellungen

### Warum die LED-Auswertung so komplex ist

Die originalen Status-LEDs werden gemultiplext. Ein einzelner ADC-Wert ist
deshalb kein stabiler Zustandsindikator.

Frühere feste Abtastraten konnten sich zufällig mit dem Multiplex-Takt
synchronisieren. Ein Profil war dann innerhalb einer Boot-Session sehr stabil,
nach einem Neustart aber verschoben.

Die aktuelle Firmware verteilt die Messzeit bewusst und bewertet
Histogramm-/CDF-Merkmale statt nur einen Mittelwert.

### Upgrade

Bei einer bestehenden Installation:

1. neue Firmware flashen
2. **Erase All Flash nicht aktivieren**
3. vorhandene FINAL-/OSC-LINEAR-Profile bleiben erhalten
4. kein neuer kompletter Kalibrierungslauf notwendig

Die öffentliche GitHub-Datei enthält keine privaten WLAN-/OTA-Zugangsdaten.
Vor dem Flashen die Platzhalter im Kopf der `.ino` anpassen.

### Firmware

`firmware/ventilator_esp32_s3_zero_FINAL_v6.4.10_UI_MANUAL_OSC.ino`

SHA-256 des öffentlichen GitHub-Builds steht in `SHA256SUMS.txt`.
