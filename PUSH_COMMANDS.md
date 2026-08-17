# Git-Push-Schritte für v6.4.10

Das Paket ist so aufgebaut, dass sein Inhalt direkt über den vorhandenen
Repository-Checkout kopiert werden kann.

## Empfohlener Commit

```bash
git status

git add -A

git commit -m "Release v6.4.10: robust LED tracking and manual status UI"

git tag -a v6.4.10 -m "v6.4.10 - FINAL UI / Manual Status / OSC"

git push origin HEAD
git push origin v6.4.10
```

`git push origin HEAD` pusht den aktuell ausgecheckten Branch und vermeidet die
Annahme, ob der Branch lokal `main` oder `master` heißt.

## Vor dem Commit prüfen

```bash
git diff --check
git status
```

Außerdem kurz kontrollieren, dass in der Firmware nur diese Platzhalter stehen:

```text
YOUR_WIFI_SSID
YOUR_WIFI_PASSWORD
CHANGE_ME_AP
CHANGE_ME_OTA
```

Die gerätespezifischen Kalibrierungs- und AI-Export-JSONs sind über
`.gitignore` ausgeschlossen.
