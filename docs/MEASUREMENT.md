# Mess- und Erkennungslogik

## Warum die LED-Messung beim HO-5500RE schwierig ist

Die fünf ausgewerteten LED-Anoden liefern **keine fünf statischen digitalen
Zustände**. Die Honeywell-Steuerplatine multiplext die Anzeigen. Am ESP32-ADC
entsteht deshalb für jeden Kanal eine zeitabhängige Verteilung.

Das führt zu mehreren Effekten:

1. **Phasenlage:** Der ESP32 besitzt keinen gemeinsamen Takt mit der
   Honeywell-LED-Ansteuerung.
2. **Aliasing:** Eine feste Abtastrate kann zufällig immer denselben Teil des
   Multiplex-Zyklus treffen.
3. **Boot-zu-Boot-Verschiebung:** Nach einem Neustart ist die relative Phase
   anders, obwohl der Ventilator physisch im gleichen Zustand steht.
4. **Gemeinsame Pegeldrift:** Alle fünf Kanäle können sich gleichzeitig nach
   oben oder unten verschieben.
5. **Ähnliche Zustände:** Besonders Breeze/Nacht und Drehen AUS/AN können
   elektrisch sehr nahe beieinander liegen.
6. **Übergangshistorie:** Das Messbild hängt teilweise davon ab, aus welchem
   Zustand der Ventilator in den Zielzustand gefahren wurde.
7. **Analoge Streuung:** Temperatur, Versorgung und ESP32-ADC tragen zusätzliche
   kleine Verschiebungen bei.

### Das frühere Problem mit einer festen Abtastung

In älteren Versionen konnte ein fester Messabstand innerhalb einer Session
hervorragend aussehen. Nach einem Neustart brach die Erkennungsleistung jedoch
ein.

Die Ursache war eine unbeabsichtigte Phasenverriegelung zwischen ESP32-Sampling
und dem LED-Multiplexing. Eine Kalibrierung hatte damit teilweise nicht den
Zustand selbst gelernt, sondern eine bestimmte zeitliche Beobachtungsphase.

## Aktuelle Strategie

Die Firmwaregeneration 6.4.x misst Verteilungen statt Einzelwerte.

### Sampling

- 12-Bit ADC
- ausschließlich ADC1-Pins
- 480 Sample-Zyklen pro Messfenster
- zufälliger Microsecond-Jitter
- zeitlich verteilte Messfenster
- variable Fensterzahl:
  - Normal / Drehen AUS: 6
  - Normal / Drehen AN: 8
  - Breeze/Nacht / Drehen AUS: 8
  - Breeze/Nacht / Drehen AN: 10

### Histogramm und CDF

Pro LED-Kanal wird ein 64-Bin-Histogramm aufgebaut.

Daraus werden zehn CDF-Punkte verwendet:

```text
1536
1792
2048
2304
2560
2816
3072
3328
3584
3840
```

Bei fünf LED-Kanälen entstehen damit **50 CDF-Merkmale**.

Zusätzlich stehen ADC-Mittelwerte und relative Kanalformen zur Verfügung.

## 18 Zustände

Die vollständige Klassifikation enthält:

```text
3 Geschwindigkeiten
× 3 Modi
× Drehen AUS/AN
= 18 Zustände
```

Speed, Modus und Drehen werden zur Laufzeit kontextabhängig behandelt.
Das ist absichtlich einfacher und robuster als eine blinde Entscheidung unter
allen 18 Profilen.

Die globale 18P-Erkennung bleibt Diagnose/Fallback.

## Transition-balanced Training

Ein wichtiger Fehler früherer Analyseversionen war die Übergangshistorie.

ON-Zustände wurden im Training fast immer direkt aus dem passenden OFF-Zustand
angefahren. Im späteren Holdout kamen dieselben Zustände dagegen aus völlig
anderen Speed-/Modus-Kombinationen.

Damit waren Training und Realität nicht gleich verteilt.

Die aktuelle Analyse verwendet mehrere unterschiedliche Vorgängerzustände und
dieselben direkten Schaltwege wie der normale Webbetrieb.

## Holdout

Der Holdout wird nicht direkt hintereinander dreimal im gleichen Zustand
gemessen.

Stattdessen werden vollständige unabhängige Zustandsrunden gefahren. So erhält
ein Zustand unterschiedliche Übergangspfade.

Aktuelle Strategie:

```text
4 × 18 Trainingsprofile
2 × 18 Basis-Holdout
+ optional 1 zusätzliche Messung für einzelne Grenzfälle
```

## Referenzlauf der aktuellen Kalibrierungsbasis

Ein realer vollständiger 6.4.3-Lauf ergab innerhalb derselben Messsession:

```text
Speed CV:       72 / 72
Modus CV:       72 / 72

Speed Holdout:  38 / 38
Modus Holdout:  38 / 38
Drehen Holdout: 35 / 38
```

Die Auswertung der Rohhistogramme erlaubte anschließend ein separates
OSC-LINEAR-Modell. Offline gegen den unabhängigen Holdout:

```text
Drehen Holdout mit OSC-LINEAR: 38 / 38
```

Diese Zahlen bedeuten nicht, dass jede spätere Boot-Session perfekt sein muss.
Gerade absolute ADC-Pegel können sich nach Neustarts verschieben. Deshalb wurde
die Laufzeiterkennung danach zusätzlich auf Cross-Boot-Robustheit optimiert.

## Normal-Priorität

Im realen Betrieb wird der Ventilator überwiegend in **Normal** genutzt.

Die aktuelle Firmware behandelt diesen Fall deshalb bewusst als wichtigsten
Praxisfall:

- Speed nutzt stärker die relative Form der fünf LED-Kanäle.
- gemeinsame absolute Pegelverschiebungen werden weniger stark bewertet.
- Modus Normal wird nicht leichtfertig durch eine unsichere LED-Diagnose
  überschrieben.
- Drehen kann über OSC-LINEAR bestätigt werden.

## Webzustand und LED-Zustand

Die LED-Erkennung ist nicht die alleinige Zustandsquelle.

**Webbefehle sind autoritativ.**

Das verhindert, dass ein zufälliger ADC-Ausreißer einen bewusst gesetzten
Ventilatorzustand im Interface überschreibt.

Wenn der Ventilator physisch außerhalb der Weboberfläche bedient wurde:

- **Status über LEDs abgleichen** versucht eine bewusste Synchronisierung.
- **Web-Status manuell setzen** erlaubt eine eindeutige Korrektur von
  Speed, Modus und Drehen ohne weitere Messung.

## Trennschärfe-Analyse

Die Analyse optimiert unter anderem:

- Kanalgewichte
- CDF-Gewichte
- ADC-Schwellwerte
- Form-/Distanzparameter
- Mean-/Level-Anteile
- Kontextklassifikation
- OFF-Referenz

Ein abgeschlossener Messlauf kann gespeichert werden, auch wenn einzelne
Qualitätsmetriken nicht perfekt sind. Die Metriken bleiben im Export erhalten.

## AI-Export

Der kompakte Export enthält unter anderem:

- Firmware- und Samplerversion
- Boot-ID und Resume-Provenienz
- Trainingsreihenfolge
- Holdout-Reihenfolge
- Vorgängerzustände
- Training pro Zustand
- Holdout pro Runde
- Vorhersagen
- Fit und Margin
- ADC-Mittelwerte
- 50 CDF-Merkmale

Der Raw-Export ergänzt die vollständigen 64-Bin-Histogramme.

Dadurch lassen sich neue Klassifikatoren offline testen, ohne erneut einen
langen mechanischen Messlauf durchzuführen.

## Könnte zusätzliche Hardware die Messung verbessern?

Grundsätzlich ja, aber nicht automatisch.

Mögliche Ansätze für zukünftige Experimente wären:

- ein hochohmiger Analogpuffer pro Messkanal, um die Honeywell-Platine noch
  stärker vom ESP32-ADC zu entkoppeln
- ein gezielt dimensionierter RC-Tiefpass, wenn ausschließlich der mittlere
  Duty-Cycle ausgewertet werden soll
- ein externer ADC mit besserer Linearität/Reproduzierbarkeit

Für die aktuelle Firmware sind solche Änderungen **nicht vorgesehen**:

- Ein RC-Filter verändert genau die Signalverteilung, aus der die CDF-Merkmale
  erzeugt werden.
- Ein Umbau würde deshalb eine vollständige Neukalibrierung erfordern.
- Ein zu großer Kondensator kann außerdem die originale LED-Ansteuerung
  beeinflussen.
- Ein einfacher Schmitt-Trigger wäre keine gute Lösung, weil die Information
  gerade in der analogen Verteilung liegt.

Deshalb gilt für die veröffentlichte Version: Die vorhandene direkte
ADC-Messung bleibt die unterstützte Referenzhardware. Hardwarefilter sollten
nur als eigener Messversuch und mit Oszilloskopkontrolle getestet werden.
