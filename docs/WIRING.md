# Verdrahtung

## Tastersteuerung

Die originalen Taster bleiben erhalten. Der ESP32 simuliert einen Tastendruck über 2N7000 N-Kanal-MOSFETs.

| Honeywell-Funktion | GPIO | Beschreibung |
|---|---:|---|
| Power | GPIO6 | Taste 1 |
| Speed | GPIO5 | Taste 2 |
| Drehen | GPIO4 | Taste 3 |
| Modus | GPIO3 | Taste 5 |

Je MOSFET gilt im Projektprinzip:

- Source → gemeinsamer GND
- Gate → ESP32-GPIO
- Drain → Tasterleitung

GPIO3 ist ein Strapping-Pin des ESP32-S3; ein Gate-Pulldown ist empfehlenswert.

## LED-Feedback

| Status | GPIO | Messpunkt |
|---|---:|---|
| Speed 1 | GPIO7 | LED1 Anode |
| Speed 2 | GPIO8 | LED2 Anode |
| Speed 3 | GPIO9 | LED3 Anode |
| Breeze | GPIO10 | LED8 Anode |
| Night | GPIO1 | LED9 Anode |

Die Kathoden der Speed-LEDs sind auf der Platine gemeinsam verschaltet. Deshalb werden die Anodenseiten gemessen.

## Versorgung

ESP32 und Steuerplatine benötigen gemeinsamen GND. Der ESP32-S3-Zero wird im finalen Aufbau aus der vorhandenen 5-V-Schiene versorgt.

**Achtung:** Ohne geeignete Entkopplung USB-5-V nicht gleichzeitig mit der 5-V-Versorgung des Ventilators verbinden.
