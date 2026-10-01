# SmartCity 2026 – Roboter / Autonomes Fahren

Code für die Yahboom ROSMASTER M3 Pro (Jetson Orin, ROS 2 Humble, Domain-ID 30).
Der Roboter folgt der **schwarzen Linie**. Eine **KI-Zentrale** behält Kamera, LiDAR und Tiefenkamera im Blick und entscheidet: fahren, langsam oder stopp (rote Ampel, Stoppschild, Hindernis, Person …). Dazu: Greifarm-Steuerung, LiDAR-Karte, Simulation ohne Roboter.

## Einmalig auf jedem Roboter

```bash
cd ~
git clone https://github.com/davidheinrich73/smartcity-roboter.git
cd ~/smartcity-roboter
scripts/installieren.sh
scripts/browser_installieren.sh
```

- `installieren.sh` **am Roboter-Bildschirm** ausführen, nicht über SSH. Es legt **SmartCity Panel** ins Programm-Menü und (nach Rückfrage) ins **Dock** und fragt nach dem Roboter-Namen (RM01–RM04). Außerdem bietet es an, onnxruntime für die KI zu installieren. An Yahboom-Dateien und am Autostart ändert es nichts.
- `browser_installieren.sh`: Chromium startet auf dem Jetson nicht (bekanntes Problem, siehe `docs/browser.md`). Empfohlen ist Punkt 1, das eigene Panel-Fenster ohne Browser.

## Benutzen: Klick auf "SmartCity Panel" im Dock

Startet Kamera, KI-Zentrale und das Control-Panel. **Panel-Fenster schließen = Roboter hält an, alles wird beendet.**

| Seite | Inhalt |
|---|---|
| Cockpit | **Was der Roboter gerade denkt** (FREI / LANGSAM / STOPP + Grund), Live-Bild (KI-Sicht, Linie, LiDAR-Radar, Tiefe), Szenario, Tempo-Regler, **START / TEST / STOPP**, alle Sinne auf einen Blick, Ereignis-Protokoll |
| Arm | 6 Servos per Regler, Greifer auf/zu, Posen speichern (Fahrstellung, Prüfblick), KI-Freigabe |
| Sensoren | Akku (Volt + geschätzte Prozent), Agent, Motorboard, IMU, Odometrie, beide LiDARs, Kamera, Gamepad mit Hz |
| System | **IP-Adresse für den Laptop**, Laptop-Zugriff an/aus, Kamera/KI starten, Update, Panel beenden |

Der rote **STOPP**-Knopf oben ist auf jeder Seite sichtbar. Not-Aus von außen: `scripts/stopp.sh`.

**Vom Laptop:** Im Panel unter System "Laptop-Zugriff erlauben", dann die angezeigte Adresse öffnen (z. B. `http://10.0.12.62:8080`). Die IP wird jedes Mal neu ermittelt. Achtung: Wer die Adresse kennt, kann dann den Roboter steuern.

**Panel ohne Roboter ansehen:** `python3 tests/panel_demo.py`, dann http://localhost:8099 (ohne ROS, ausgedachte Daten).

## Wer entscheidet was?

```
Kamera, LiDAR, Tiefenkamera ──► KI-Zentrale (ki/) ──► fahren / langsam / stopp ──► Linienfolger ──► Motoren
                                       └──► Arm (nur wenn freigegeben, nur im Stand)
```

| # | Situation | Entscheidung |
|---|---|---|
| 1 | LiDAR: etwas näher als 12 cm | STOPP sofort, ohne KI-Prüfung (Eigenschutz) |
| 2 | kein LiDAR / keine Kamerabilder / KI antwortet nicht | STOPP |
| 3 | LiDAR: etwas zwischen 12 und 45 cm | KI prüft mit Kamera-KI und Tiefenkamera: bestätigt → STOPP, sonst LANGSAM |
| 4 | Kamera-KI sieht Person/Auto/… im Weg | STOPP |
| 5 | Ampel rot/gelb | STOPP bis grün |
| 6 | Stoppschild | 3 s halten, dann weiter |
| 7 | keine Linie | STOPP |

Szenario **RTW-Einsatz**: 5 und 6 werden übergangen, alles andere gilt weiter. Details: `docs/ki.md`.

**Noch nicht eingebaut:** Abbiegen an Kreuzungen (zufällig durchs Straßennetz), Gegenstände mit dem Arm aufsammeln, Navigation auf der Karte.

## Einstellen

| Was | Wie |
|---|---|
| LiDAR "vorne" | **Zuerst machen!** Panel → Cockpit → LiDAR-Radar. Gegenstand vor die Kamera-Seite stellen, mit den Pfeilen drehen, bis er oben erscheint, "Speichern" (`config/roboter.yaml`). |
| Arm-Fahrstellung | Panel → Arm, vorsichtig einstellen, "als Fahrstellung speichern" (`config/arm.yaml`) |
| Ampel-LEDs | `scripts/ampel_kalibrieren.sh`: Schieberegler, Lupe, `s` = Foto, `w` = Werte in `config/ampel.yaml` |
| Tempo | Regler im Panel (0,05–0,4 m/s, Standard 0,15) |
| Linie | `scripts/test.sh -p threshold:=60` usw. |

Wichtige Einstellungen des Linienfolgers (`-p name:=wert` oder `ros2 param set /line_follower name wert`):

| Name | Standard | Bedeutung |
|---|---|---|
| `speed` | 0.15 | m/s (vorher 0.08) |
| `steer_gain` | 0.004 | Lenkstärke. Lenkt falsch herum → Vorzeichen umdrehen |
| `threshold` | 70 | dunkler als das = Linie |
| `strip_start` | 0.75 | Linie nur im unteren Viertel suchen |
| `notbremse_dist` | 0.12 | eigene Notbremse (m), unabhängig von der KI |
| `obstacle_check` | true | Notbremse an/aus (RM03 hat kein LiDAR → `false`) |
| `ki_pflicht` | true | ohne KI-Zentrale nicht fahren |

## Warum fuhr der Roboter so langsam und ruckelig?

1. Tempo war 0,08 m/s. Jetzt 0,15 m/s, im Panel einstellbar.
2. Das Motorboard stoppt die Motoren 0,3 s nach dem letzten Fahrbefehl. Der alte Linienfolger schickte nur nach jedem verarbeiteten Kamerabild einen Befehl. Jetzt geht der Befehl 20× pro Sekunde raus.
3. Große Kamerabilder kamen in ROS 2 nur stockend an. In der Simulation waren es 1 statt 15 Bilder/s, weil der Shared-Memory-Bereich von Fast DDS (512 KB) kleiner ist als ein Bild (900 KB). `config/fastdds.xml` vergrößert ihn, `scripts/env.sh` setzt ihn für alle unsere Programme. Danach kamen 15 von 15 Bildern an. Auf dem Roboter noch nicht gemessen. Das Panel zeigt "Linienfolger … Bilder/s" an.

Die Kamera war auf dem Laptop stark verzögert, weil das Panel einen Videostrom geschickt hat, der sich im WLAN aufstaut. Jetzt wird jedes Bild erst geholt, wenn das vorige da ist.

## Ohne Panel (einzelne Fenster)

| Befehl | Zweck |
|---|---|
| `scripts/kamera.sh` | Kamera starten, offen lassen |
| `scripts/ki.sh` | KI-Zentrale |
| `scripts/test.sh` | Linienfolger im Testmodus: zeigt die Linie, **fährt nicht** |
| `scripts/fahren.sh` | Fährt los (KI-Zentrale muss laufen). **Strg+C = Not-Aus** |
| `scripts/stopp.sh` | Not-Aus von außen |
| `scripts/check.sh` | Schnellcheck: Name, MAC, Agent, Akku, LiDAR, Kamera, unsere Programme. Fährt nicht. |
| `scripts/arm_suchen.sh` | Wie wird der Arm angesteuert? Nur lesen. |
| `scripts/ki_suchen.sh` | Welche KI ist auf dem Roboter? Nur lesen. |
| `scripts/update.sh` | neueste Version holen |

## Simulation (ohne Roboter, braucht ROS 2 Humble)

```bash
scripts/simulation.sh           # Simulator + Panel, im Browser http://localhost:8080, START drücken
scripts/simulation.sh pruefen   # fährt eine Runde und prüft alles automatisch
```

Ein simulierter Roboter fährt einen Rundkurs: rote Ampel (wird nach 3 s Stand grün), Stoppschild, Hindernis auf der Fahrbahn (geht nach 3 s weg) und ein Haus am Rand, das nur der LiDAR sieht. Die Simulation nutzt Domain-ID 77, nie die 30 der Roboter.

## Karte (LiDAR)

| Fenster | Befehl |
|---|---|
| 1 | `scripts/karte_erstellen.sh` (sucht selbst die Yahboom-Startdatei: `slam_mapping` oder `m3_bringup`) |
| 2 | `scripts/karte_anzeigen.sh` (eigene RViz-Ansicht, oder `... yahboom`) |
| – | Mit Gamepad **langsam** alle Straßen abfahren |
| 3 | `scripts/karte_speichern.sh smartcity` |

Hochladen: `git add maps && git commit -m "Karte smartcity" && git push`
Laden und anzeigen: `scripts/karte_laden.sh smartcity`
Klappt etwas nicht: `scripts/karte_diagnose.sh > diagnose.txt` (einmal ohne, einmal während `karte_erstellen.sh` läuft) und die Datei weitergeben.

**Alle Karten-Skripte sind noch UNGETESTET auf dem Roboter.**

## Tests

```bash
python3 tests/test_ampel.py          # Ampel-LEDs (3 Farben), rote Gegenstände
python3 tests/test_schilder.py       # Stoppschild-Form
python3 tests/test_entscheider.py    # Entscheidungen der KI
python3 tests/test_line_follower.py  # wann der Linienfolger fährt
scripts/simulation.sh pruefen        # alles zusammen (braucht ROS 2)
```

## Bekannte Probleme

- Alle Roboter heißen `yahboom` → nur einen gleichzeitig einschalten (Domain-ID 30 für alle).
- Waagrechte schwarze Flächen (Kreuzungen, Kabel) verwirren den Linienfolger.
- RM03: ein Rad schwächer, Board liefert keine Sensordaten → "Kein LiDAR", fährt nicht los.
- Gamepad und Fahrprogramm senden beide auf `/cmd_vel` → nicht gleichzeitig benutzen.
- Desktop-Symbol meldet "Untrusted Desktop File" trotz Markierung → Dock benutzen.
- Der LiDAR sieht nur eine dünne waagrechte Scheibe: Ein Schuh ist im Radar nur ein kurzer Bogen.

Siehe auch `docs/ki.md`, `docs/greifarm.md`, `docs/browser.md`, `docs/roboter.md`.
