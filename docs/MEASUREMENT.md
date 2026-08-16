# Mess- und Erkennungslogik

## Problem

Die LEDs des HO-5500RE liefern am ADC kein einfaches statisches Ein/Aus-Signal. Die Kanäle zeigen deutliche gemeinsame Drift und Multiplex-Verhalten. Ein einzelner ADC-Wert oder ein starrer Grenzwert war deshalb nicht zuverlässig genug.

## Finale Strategie

Die Firmware:

1. misst alle fünf Kanäle innerhalb desselben Zeitfensters verschachtelt,
2. rotiert den zuerst gemessenen Kanal,
3. verwirft nach einem ADC-MUX-Wechsel die erste Probe,
4. verwendet Fließkomma-Duty-Cycles statt ganzzahlig gerundeter Prozentwerte,
5. lernt Profile über mehrere Messfenster,
6. verwendet robuste Mittelwertbildung,
7. analysiert Histogramme und Trennschärfe,
8. speichert neun Kombinationsprofile aus Speed × Modus,
9. behandelt AUS separat,
10. überschreibt bei unsicheren Breeze/Night-Treffern keinen bereits bekannten Modus.

## Letzte Live-Messung

### AUS

Die fünf Kanäle lagen bei ungefähr 48–50 %. Der Abstand zum nächstliegenden aktiven Zustand lag bei über 40 Prozentpunkten. AUS ist damit sehr deutlich erkennbar.

### Speed

Die Speed-Paare lagen zuletzt bei Trennfaktoren von ungefähr 9.7 bis 12.8 und sind damit sehr stabil unterscheidbar.

### Modus

Normal gegen Breeze/Night ist sehr deutlich. Breeze gegen Night bleibt das schwierigste Paar und kann je nach Speed nahe an der Streuung liegen. Deshalb wird diese Entscheidung konservativer behandelt.

### Optimierte Schwellwerte

```text
{2368, 3072, 2432, 3200, 2752}
```

### Kanalgewichte

```text
LED1 1.10
LED2 0.75
LED3 1.26
LED8 0.76
LED9 1.12
```

### Formanteil

```text
1.00
```

Damit wird bei vollständiger Kalibrierung vor allem die Form/Verteilung der Kanäle ausgewertet und gemeinsame Pegeldrift unterdrückt.
