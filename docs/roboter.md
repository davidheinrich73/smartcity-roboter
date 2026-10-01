# Roboter-Übersicht

| Roboter | IP | Zustand | Bemerkung |
|---|---|---|---|
| RM01 | ? | ? | noch nicht geprüft |
| RM02 | 10.0.12.62 (Schulnetz), früher 192.168.8.88 | fährt, Linie + Ampel laufen | Referenz-Roboter, Akku 12,5 V am 01.10. |
| RM03 | 192.168.8.88 | fährt | ein Rad schwächer, keine Sensordaten; alter Code von früheren Teams im Home-Ordner |
| RM04 | ? | ? | noch nicht geprüft |

Alle: Benutzer `jetson`, Domain-ID 30, micro-ROS-Agent startet automatisch.

Welcher Roboter ist es? Alle heißen `yahboom`. `scripts/installieren.sh` fragt einmal nach dem Namen und speichert ihn in `~/roboter_name`. `scripts/check.sh` zeigt ihn an, dazu die MAC-Adressen (bei jedem Gerät anders), bitte hier eintragen.

| Roboter | MAC WLAN | MAC LAN |
|---|---|---|
| RM01 | ? | ? |
| RM02 | ? | ? |
| RM03 | ? | ? |
| RM04 | ? | ? |
