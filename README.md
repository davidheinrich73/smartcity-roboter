# SmartCity 2026 – Roboter / Autonomes Fahren

Code für die Yahboom ROSMASTER M3 Pro (Jetson Orin, ROS 2 Humble, Domain-ID 30).
Ziel: Roboter folgt der **schwarzen Linie**, hält an **roter Ampel**, erstellt mit dem **LiDAR** eine Karte.

## Einmalig auf jedem Roboter

```bash
cd ~
git clone <REPO-URL> smartcity-roboter
cd ~/smartcity-roboter
```

Neueste Version holen: `scripts/update.sh`

## Linie + Ampel

Jeweils ein eigenes SSH-Fenster:

| Fenster | Befehl | Zweck |
|---|---|---|
| 1 | `scripts/kamera.sh` | Kamera starten, offen lassen |
| 2 | `scripts/test.sh` | Testmodus: zeigt Erkennung, **fährt nicht** |
| 2 | `scripts/fahren.sh` | Fährt los. **Strg+C = Not-Aus** |
| 3 | `scripts/stopp.sh` | Not-Aus von außen |

Einstellungen anhängen, z. B. `scripts/fahren.sh -p speed:=0.1 -p red_val_min:=220`.
Während es läuft: `ros2 param set /line_follower red_val_min 220`.

Wichtige Einstellungen:

| Name | Standard | Bedeutung |
|---|---|---|
| `speed` | 0.08 | m/s |
| `steer_gain` | 0.004 | Lenkstärke. Lenkt falsch herum → Vorzeichen umdrehen |
| `threshold` | 70 | dunkler als das = Linie |
| `strip_start` | 0.75 | Linie nur im unteren Viertel suchen |
| `red_val_min` | 200 | Mindesthelligkeit Rot. Rote Gegenstände lösen aus → höher |
| `red_min_area` | 15 | kleinste rote Fläche (Pixel) |
| `red_top/bottom/left/right` | 0/0.6/0/1 | Suchbereich Ampel im Bild |

## Karte (LiDAR)

| Fenster | Befehl |
|---|---|
| 1 | `scripts/karte_erstellen.sh` |
| 2 | `scripts/karte_anzeigen.sh` |
| – | Mit Gamepad **langsam** alle Straßen abfahren |
| 3 | `scripts/karte_speichern.sh smartcity` |

Karte liegt danach in `maps/`. Hochladen, damit alle Roboter sie haben:
```bash
git add maps && git commit -m "Karte smartcity" && git push
```
Laden: `scripts/karte_laden.sh smartcity` (ungetestet)

## Check

`scripts/check.sh` – Agent, Akku, LiDAR, Kamera auf einen Blick. Fährt nicht.

## Bekannte Probleme

- Alle Roboter heißen `yahboom` und haben dieselbe IP `192.168.8.88` → nur einen gleichzeitig einschalten, bis das geklärt ist (IP-Adresskonzept-Team).
- Kamera-Treiber startet nicht automatisch → `scripts/kamera.sh`.
- Waagrechte schwarze Flächen (Kreuzungen, Kabel) verwirren den Linienfolger.
- RM03: ein Rad schwächer, Board liefert keine Sensordaten (`/battery`, `/scan0` fehlen).
- Ampel-LEDs sind klein: Erkennung erst aus der Nähe zuverlässig.

Siehe auch `docs/roboter.md`.
