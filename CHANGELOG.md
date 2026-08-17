# Changelog

## 6.4.10-FINAL-UI-MANUAL-OSC

Großes Update gegenüber der bisher im Repository veröffentlichten
**5.0.0-FINAL**.

### Webinterface

- Hauptoberfläche in **Steuerung** und **Einstellungen** aufgeteilt.
- ESP32-Info und OTA-Update aus der täglichen Steuerungsansicht entfernt und
  unter **Einstellungen → ESP32 & OTA** zusammengefasst.
- Trennschärfe-Analyse, Speed-Kalibrierung und Backup/Wiederherstellung unter
  **Einstellungen → Kalibrierung & Daten** gebündelt.
- Abstände und mobile Darstellung überarbeitet.
- **Status über LEDs abgleichen** direkt unter den Sleep-Timer verschoben.
- erklärenden Beschreibungstext unter dem LED-Abgleich entfernt.
- neues, beim Seitenaufruf standardmäßig geschlossenes Panel
  **Web-Status manuell setzen**.

### Manuelle Statuskorrektur

Neu kann der interne ESP-/Webzustand vollständig korrigiert werden, wenn der
Ventilator direkt am Gerät oder über die originale Fernbedienung bedient wurde.

Manuell setzbar:

- Aus
- Stufe 1
- Stufe 2
- Stufe 3
- Normal
- Breeze
- Nacht
- Drehen AUS / AN

Diese Bedienung erzeugt **keine physischen Tastendrücke**. Sie korrigiert nur
den vom ESP32 angenommenen Zustand.

### 18 statt 9 Zustandsprofile

5.0.0 verwendete neun Kombinationsprofile:

`3 Geschwindigkeiten × 3 Modi`

6.4.10 verwendet:

`3 Geschwindigkeiten × 3 Modi × Drehen AUS/AN = 18 Zustände`

Damit wird der Schwenkbetrieb nicht mehr nur als zusätzlicher Bool-Wert neben
dem Profil betrachtet.

### Neue LED-Messmethode

Die frühere feste/synchronisierte Mehrkanalmessung wurde durch eine
phasenentkoppelte Verteilungsmessung ersetzt:

- zufällige ADC-Abtastabstände
- zeitlich getrennte Messfenster
- 64-Bin-ADC-Histogramme
- 10 CDF-Stützstellen pro LED-Kanal
- 5 Kanäle × 10 CDF-Werte = 50 Rich-CDF-Merkmale
- relative ADC-/Kanalformen
- variable Messtiefe abhängig von Modus und Drehen
- robuste OFF-Referenz

### Warum diese Änderung nötig war

Die LED-Signale des HO-5500RE werden von der Originalplatine gemultiplext.
Eine feste ADC-Abtastrate kann sich mit diesem Multiplex-Zyklus
**phasenverriegeln**.

Dadurch konnten alte Kalibrierungen innerhalb einer Boot-Session extrem stabil
aussehen und nach einem Neustart trotzdem deutlich verschobene Werte liefern.
Das Problem war kein einzelner falscher Schwellwert, sondern Aliasing zwischen
ESP-Abtastung und LED-Multiplexing.

Die neue Messung verteilt die Abtastzeit bewusst und bewertet nicht nur einen
Mittelwert, sondern die gesamte beobachtete Signalverteilung.

### Transition-balanced Training

Die Trainingszustände werden nicht mehr überwiegend über denselben
Vorgängerzustand angefahren.

- unterschiedliche Vorgänger pro Zustand
- direkte Zustandswechsel wie im echten Webbetrieb
- kein künstliches Normalisieren vor jedem Wechsel
- reduzierte Übergangsverzerrung
- repeat-basierter Resume bleibt möglich

### Unabhängiger Holdout

Die Validierung wurde von aufeinanderfolgenden Wiederholungsmessungen auf
echte, getrennte Zustandsdurchläufe umgestellt.

Aktuell:

- 4 × 18 Training
- 2 × 18 Basis-Holdout
- optional dritte Messung nur für einzelne Grenzfälle

Ein vollständig abgeschlossener Lauf darf gespeichert werden, auch wenn
einzelne Diagnosemetriken nicht perfekt sind.

### KI-/Analyseexport

Neu:

- kompakter JSON-Export
- optional vollständige 64-Bin-Rohhistogramme
- Boot-/Resume-Provenienz
- Trainings- und Holdout-Reihenfolge
- Vorgängerzustände
- per-Round-Vorhersagen
- Speed-/Modus-/Drehen-Kontexttreffer
- Fit und Margin
- Modellparameter und CDF-Merkmale

Damit können Messläufe offline analysiert werden, ohne den Ventilator erneut
durch eine lange Messreihe zu fahren.

### OSC-LINEAR

Für **Drehen AUS/AN** wurde ein zusätzliches mode-spezifisches lineares Modell
eingeführt.

Es verwendet die bereits gemessenen Rich-CDF-Merkmale und verbessert die
Drehen-Entscheidung, ohne die funktionierende Speed-/Modus-Erkennung zu
verschlechtern.

Im realen Referenzlauf verbesserte sich der unabhängige Drehen-Holdout von
`35/38` auf `38/38`, während Speed und Modus bei `38/38` blieben.

### Normal-Priorität / Cross-Boot-Robustheit

Da der Ventilator im Praxisbetrieb fast immer in **Normal** verwendet wird,
wird dieser Modus besonders robust behandelt.

- Speed in Normal verwendet stärker die relative Kanalform.
- absolute ADC-Pegel werden weniger aggressiv gewichtet.
- gemeinsame Pegelverschiebungen zwischen Boot-Sessions führen seltener zu
  falschen Speed-Sprüngen.
- Breeze/Nacht bleiben vollständig unterstützt, werden diagnostisch aber
  konservativer behandelt.

### LED-Abgleich

- der Button ist bei geladenen FINAL-Profilen nicht mehr allein durch die
  globale 18P-Sicherheit blockiert.
- mehrere kurze Messungen statt einer Einzelentscheidung.
- Messfenster entsprechen der Hintergrunddiagnose:
  - Normal AUS: 6
  - Normal AN: 8
  - Breeze/Nacht AUS: 8
  - Breeze/Nacht AN: 10
- ein stabiler, frischer Diagnosehinweis kann beim manuellen Sync bevorzugt
  werden.

### Persistenz und Stabilität

- persistente FINAL-Profile
- CRC-verifizierte A/B-Speicherung
- Analyse-Checkpoints
- balanced Resume
- NVS-/Backup-Generationen weiterentwickelt
- große NVS-Datenstrukturen aus dem Task-Stack entfernt
- dadurch Boot-/Stack-Overflow-Risiko der OSC-LINEAR-Zwischenversion behoben
- OTA-/Software-Neustart und echter Power-Cycle werden getrennt behandelt

### Grundsatz der Zustandserkennung

Der **Webzustand bleibt autoritativ**.

Die LED-Erkennung ist Plausibilitätskontrolle und bewusste Synchronisierung.
Sie soll einen absichtlich per Web gewählten Zustand nicht automatisch
überschreiben.

Wenn die LED-Erkennung einen manuell geänderten Zustand nicht sicher erkennt,
kann der Webstatus ab 6.4.10 vollständig von Hand korrigiert werden.

### Kompatibilität

- gleiche Hardwarebelegung wie 5.0.0
- Taste 4 / Original-Timer weiterhin unbenutzt
- bestehende FINAL-/OSC-LINEAR-Kalibrierungen können bei einem normalen Update
  erhalten bleiben
- **Erase All Flash** bei einem Update vermeiden, wenn die bestehende
  Kalibrierung erhalten bleiben soll

---

## 5.0.0-FINAL

- finale 9-Kombinationsprofil-Erkennung
- optimierte Default-Schwellwerte aus synchronisierter Live-Messung
- Kanalform als primäres Merkmal bei Formanteil 1.00
- konservative Breeze/Night-Übernahme
- synchronisierte ADC-Messung mit rotierender Kanalreihenfolge
- verworfene erste ADC-Probe nach Kanalwechsel
- robuste Kalibrierung und Trennschärfeanalyse
- persistente Profile, Backup/Import
- Startscan und manueller LED-Sync
- Power-Cycle-Defaultlogik
- optimistische Weboberfläche
- OTA und Fallback-AP
