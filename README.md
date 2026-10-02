# SmartCity 2026 – Roboter / Autonomes Fahren

Code für die Yahboom ROSMASTER M3 Pro (Jetson Orin, ROS 2 Humble, Domain-ID 30).
Der Roboter folgt der **schwarzen Linie**, auch in engen Kurven. Eine **KI-Zentrale** behält Kamera, LiDAR und Tiefenkamera im Blick und entscheidet:
- fahren, langsam oder stopp, z. B. bei roter Ampel, Stoppschild, Person oder Hindernis,
- am Zebrastreifen anhalten und sich umschauen,
- vor einer Einbahnstraße wenden,
- kleine Hindernisse mit dem Arm aufheben und beiseitelegen.

Ein **Kartograf** baut bei jeder Fahrt eine 2D- und 3D-Karte. Mit jeder Fahrt wird sie genauer. Alles ist im **Dashboard** zu sehen. Dazu gibt es eine Simulation ohne Roboter.

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

**Neue Version holen:** `scripts/update.sh` (oder Panel → System → "Neueste Version holen"). Eigene Einstellungen (Arm-Posen, LiDAR-Winkel) liegen in `config/lokal/` und bleiben erhalten. Das Skript übernimmt auch Einstellungen, die ältere Panel-Versionen direkt in `config/arm.yaml`/`config/roboter.yaml` geschrieben haben. Vorher legt es eine Sicherung an.

## Benutzen: Klick auf "SmartCity Panel" im Dock

Startet Kamera, KI-Zentrale, Kartograf und das Control-Panel. **Panel-Fenster schließen = Roboter hält an, alles wird beendet.**

| Seite | Inhalt |
|---|---|
| **Dashboard** | Alles auf einen Blick (passt auf 1024×600): was die KI gerade denkt, **Karte** (2D/3D umschaltbar), **Kamera** (KI-Sicht/Linie/Tiefe), **LiDAR** mit Fahrschlauch, **START / TEST / STOPP**, Szenario, Tempo, Wahrnehmung, Ereignisse |
| Karte | Karte groß: verschieben, zoomen, in 3D drehen, Umschauen, Speichern, Neue Karte, Export als PNG und PLY (3D) |
| Kamera / LiDAR | dieselben Ansichten groß, LiDAR-Ausrichtung einstellen |
| Arm | 6 Servos per Regler, Greifer auf/zu, **Posen einlernen** (Fahrstellung, Prüfblick, Blick links/rechts, Greifen, Greifen hoch, Ablegen), KI-Freigabe |
| Sensoren | Akku, Agent, Motorboard, IMU, Odometrie, LiDARs, Kamera, Gamepad, unsere Programme |
| System | **IP-Adresse für den Laptop**, Laptop-Zugriff an/aus, Programme starten/stoppen, Update, Panel beenden |

Der rote **STOPP**-Knopf oben ist auf jeder Seite sichtbar. Not-Aus von außen: `scripts/stopp.sh`. Nach STOPP (und im TEST) bewegt die KI den Arm nie.

**Vom Laptop:** Im Panel unter System "Laptop-Zugriff erlauben", dann die angezeigte Adresse öffnen (z. B. `http://10.0.12.62:8080`). Die IP wird jedes Mal neu ermittelt. Achtung: Wer die Adresse kennt, kann dann den Roboter steuern.

**Panel ohne Roboter ansehen:** `python3 tests/panel_demo.py`, dann http://localhost:8099 (ohne ROS, ausgedachte Daten).

## Wer entscheidet was?

```
Kamera, LiDAR, Tiefenkamera ──► KI-Zentrale (ki/) ──► fahren / langsam / stopp / Manöver ──► Linienfolger ──► Motoren
                                       └──► Arm (nur wenn freigegeben, nur im Stand, nur während der Fahrt)
LiDAR, Odometrie, IMU, Kamera, Tiefe, KI ──► Kartograf (kartograf/) ──► Karte (karten/<name>/) ──► Panel
```

| # | Situation | Entscheidung |
|---|---|---|
| 1 | LiDAR: etwas näher als 12 cm | STOPP sofort, ohne KI-Prüfung (Eigenschutz) |
| 2 | kein LiDAR / keine Kamerabilder / KI antwortet nicht | STOPP |
| 3 | LiDAR: etwas **im Fahrschlauch** (Streifen entlang der Linie), näher als 45 cm | KI prüft mit Kamera-KI und Tiefenkamera: bestätigt → STOPP (oder **aufheben**), sonst LANGSAM, ab 20 cm STOPP |
| 4 | Tiefenkamera: etwas Flaches auf der Fahrbahn (unter der LiDAR-Ebene, z. B. Holzwürfel) | STOPP oder **aufheben und beiseitelegen** |
| 5 | Kamera-KI sieht Person/Auto/… im Weg | STOPP |
| 6 | Schild "Einfahrt verboten" (Einbahnstraße) | **wenden** |
| 7 | Zebrastreifen | anhalten, **nach links und rechts schauen**, erst weiter, wenn niemand kommt |
| 8 | Ampel rot/gelb | STOPP bis grün |
| 9 | Stoppschild | 3 s halten, dann weiter |
| 10 | keine Linie | kurz suchen (drehen), dann STOPP |

Szenario **RTW-Einsatz**: 8 und 9 werden übergangen, alles andere gilt weiter. Aufgehoben wird nur, was klein ist, mitten auf der Straße liegt, kein Lebewesen/Fahrzeug ist und nicht am Zebrastreifen steht. Details: `docs/ki.md`, `docs/greifarm.md`.

**Noch nicht eingebaut:** Abbiegen an Kreuzungen (zufällig durchs Straßennetz), Navigation auf der Karte ("fahr zu Ort X").

## Einstellen (Reihenfolge)

| Was | Wie |
|---|---|
| 1. LiDAR "vorne" | Panel → LiDAR. Gegenstand vor die Kamera-Seite stellen, mit den Pfeilen drehen, bis er oben erscheint, "Speichern". |
| 2. Arm-Posen | Panel → Arm. Zuerst **Fahrstellung**, dann Prüfblick, Blick links/rechts, Greifen, Greifen hoch, Ablegen (siehe `docs/greifarm.md`). Danach KI neu starten. |
| 3. Kamera nachmessen | Höhe, Neigung und Abstand der Kamera in Fahrstellung messen und in `config/lokal/roboter.yaml` eintragen (`docs/linie.md`). Die Neigung ist am wichtigsten. |
| 4. Ampel-LEDs | `scripts/ampel_kalibrieren.sh`: Schieberegler, Lupe, `s` = Foto, `w` = Werte in `config/ampel.yaml` |
| Tempo | Regler im Panel (0,05–0,4 m/s, Standard 0,15) |

Eigene Werte gehören nach `config/lokal/` (nicht im Repository), die Standardwerte stehen in `config/roboter.yaml` und `config/arm.yaml`.

## Ohne Panel (einzelne Fenster)

| Befehl | Zweck |
|---|---|
| `scripts/kamera.sh` | Kamera starten, offen lassen |
| `scripts/ki.sh` | KI-Zentrale |
| `scripts/kartograf.sh` | Kartograf (baut die Karte, fährt nicht) |
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
scripts/simulation.sh pruefen   # fährt gut 2 Minuten und prüft alles automatisch
```

Die Simulation enthält einen Rundkurs mit engen Kurven, eine Ampel (wird nach 3 s Stand grün) und einen Zebrastreifen mit Fußgänger. Außerdem gibt es einen Holzwürfel auf der Fahrbahn, ein Stoppschild, eine Einbahnstraße und Häuser. Simuliert werden Kamera, Tiefenkamera, beide LiDARs, Odometrie (mit Fehlern), IMU und der Arm (Greifen).
`pruefen` prüft unter anderem:
- Linie auf ±4 cm,
- nicht über Rot, am Zebrastreifen gewartet,
- Würfel gegriffen und neben die Straße gelegt,
- am Stoppschild gehalten, gewendet statt in die Einbahnstraße,
- Kartenposition genauer als 10 cm.

Letzter Lauf: alles bestanden (Normal und RTW-Einsatz). Linie max. 2,5 cm, Kartenposition im Mittel 1,5 cm, max. 5,3 cm. Die Simulation nutzt Domain-ID 77, nie die 30 der Roboter, und eine eigene Karte "sim".

## Karte

Der Kartograf läuft automatisch mit der KI. Mehr dazu in `docs/karte.md`.
- Die Karte liegt in `karten/smartcity/`, nicht im Repository.
- Beim nächsten Start sucht der Roboter seine Position selbst in der vorhandenen Karte.
- Export im Panel: PNG (2D) und PLY (3D, öffnen z. B. mit MeshLab oder CloudCompare).

Die älteren Skripte `karte_erstellen.sh` usw. (Yahboom/slam_toolbox, mit Gamepad abfahren) gibt es weiterhin. Sie sind noch ungetestet.

## Tests

```bash
python3 tests/test_ampel.py          # Ampel-LEDs (3 Farben), rote Gegenstände
python3 tests/test_schilder.py       # Stoppschild-Form
python3 tests/test_entscheider.py    # Entscheidungen der KI (Zebrastreifen, Wenden, Aufheben, Sicherheitsregeln ...)
python3 tests/test_line_follower.py  # wann der Linienfolger fährt
python3 tests/test_karte.py          # Karte + Wiederfinden + Tiefenkamera (mit der Simulationswelt, ca. 20 s)
scripts/simulation.sh pruefen        # alles zusammen (braucht ROS 2)
```

## Bekannte Probleme / ungeprüft

- **Alles Neue ist nur in der Simulation getestet, nicht auf dem echten Roboter.** Dazu gehören Zebrastreifen, Einbahnstraße, Aufheben, Karte und Dashboard.
- Kameramaße (Höhe 22 cm, Neigung 32°) sind geschätzt. Bitte nachmessen, sonst wird die Linie ungenauer (`docs/linie.md`).
- Richtung von Servo 1 (Blick links/rechts) ist ungeprüft. Zeigt "Blick links" nach rechts, die Posen im Panel neu einlernen.
- Ob `/odom_raw` Geschwindigkeiten liefert, ist ungeprüft. Ohne Odometrie rechnet der Kartograf nur mit IMU und LiDAR.
- Alle Roboter heißen `yahboom` → nur einen gleichzeitig einschalten (Domain-ID 30 für alle).
- RM03: ein Rad schwächer, Board liefert keine Sensordaten → "Kein LiDAR", fährt nicht los.
- Gamepad und Fahrprogramm senden beide auf `/cmd_vel` → nicht gleichzeitig benutzen.
- Desktop-Symbol meldet "Untrusted Desktop File" trotz Markierung → Dock benutzen.

Siehe auch `docs/ki.md`, `docs/linie.md`, `docs/karte.md`, `docs/greifarm.md`, `docs/browser.md`, `docs/roboter.md`.
