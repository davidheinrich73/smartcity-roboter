# SmartCity 2026 – Roboter / Autonomes Fahren

Code für die Yahboom ROSMASTER M3 Pro (Jetson Orin, ROS 2 Humble, Domain-ID 30).
Der Roboter folgt der **schwarzen Linie**, hält an **roter Ampel**, am **Stoppschild** und vor **Hindernissen** (LiDAR + KI) und kann mit dem **LiDAR** eine Karte erstellen.

## Einmalig auf jedem Roboter

```bash
cd ~
git clone https://github.com/davidheinrich73/smartcity-roboter.git
cd ~/smartcity-roboter
scripts/installieren.sh
```

`installieren.sh` **am Roboter-Bildschirm** (nicht über SSH) ausführen. Es legt **SmartCity Panel** auf dem Desktop und im Programm-Menü an (Super-Taste, "SmartCity" tippen), fragt nach dem Roboter-Namen (RM01–RM04) und bietet an, onnxruntime für die KI zu installieren. An Yahboom-Dateien und am Autostart ändert es nichts.

## Benutzen: Doppelklick auf "SmartCity Panel"

Am zuverlässigsten: das Symbol im **Dock** (Leiste am Bildschirmrand), `installieren.sh` legt es nach Rückfrage dort an. Das Desktop-Symbol meldet auf dem Jetson trotz Markierung "Untrusted Desktop File" (Ursache noch unklar).

Startet Kamera, KI-Erkennung und das Control-Panel im Vollbild. **Fenster schließen = Roboter hält an, alles wird beendet.**

| Seite | Inhalt |
|---|---|
| Fahren | Szenario wählen (Normal, RTW-Einsatz, Langsam), **START**, **TEST** (fährt nicht), **STOPP** |
| Sensoren | Akku (Volt + geschätzte Prozent), Agent, Motorboard, IMU, Odometrie, beide LiDARs, Kamera, Gamepad mit Hz |
| Kamera | was der Linienfolger sieht (Linie, Ampel, Stoppschild) |
| KI-Erkennung | Personen, Autos usw. mit Kästen; rot = im Weg |
| LiDAR | Draufsicht beider LiDARs mit Notbrems-Bereich |
| Tiefe | Tiefenkamera farbig (rot = nah). Ein Radar hat der Roboter nicht. |
| Greifarm | noch nicht eingebunden, nur Diagnose (siehe `docs/greifarm.md`) |

Der rote **STOPP**-Knopf oben ist auf jeder Seite sichtbar. Not-Aus von außen: `scripts/stopp.sh`.

Panel ohne Roboter ansehen (z. B. auf dem Laptop): `python3 tests/panel_demo.py`, dann http://localhost:8099

Vom Laptop aus bedienen: `scripts/panel.sh --netz`, dann `http://<IP-des-Roboters>:8080`. **Achtung:** dann kann jeder im Netz den Roboter starten.

## Wann hält der Roboter an?

1. **LiDAR** sieht etwas näher als 30 cm vorne (oder LiDAR liefert nichts) → Stopp
2. **KI** meldet Person/Auto/… im Weg → Stopp
3. **Ampel rot** → Stopp, bis Rot weg ist (nicht im Szenario RTW-Einsatz)
4. **Stoppschild** → 3 s halten, dann weiter (nicht im Szenario RTW-Einsatz)
5. **keine Linie** oder **keine Kamerabilder** → Stopp

**Noch nicht eingebaut:** Abbiegen an Kreuzungen (zufällig durchs Straßennetz), Greifarm, Navigation auf der Karte.

## Einstellen

| Was | Wie |
|---|---|
| Ampel | `scripts/ampel_kalibrieren.sh`: Schieberegler, Lupe, `s` = Foto speichern, `w` = Werte in `config/ampel.yaml` speichern. Ohne Roboter mit gespeicherten Fotos: `python3 line_follower/ampel_kalibrieren.py --bilder ampel_bilder` |
| LiDAR-Richtung "vorne" | `config/roboter.yaml` → `scan_front_deg`. **Noch nicht geprüft!** Panel → LiDAR, Hand vor die Kamera-Seite halten, Winkel anpassen, bis die Hand oben im roten Bereich erscheint. |
| Linie, Geschwindigkeit | `scripts/test.sh -p speed:=0.1` usw., siehe Tabelle unten |

Wichtige Einstellungen des Linienfolgers:

| Name | Standard | Bedeutung |
|---|---|---|
| `speed` | 0.08 | m/s |
| `steer_gain` | 0.004 | Lenkstärke. Lenkt falsch herum → Vorzeichen umdrehen |
| `threshold` | 70 | dunkler als das = Linie |
| `strip_start` | 0.75 | Linie nur im unteren Viertel suchen |
| `red_val_min` | 200 | Mindesthelligkeit Rot |
| `red_max_area` | 1500 | größere rote Flächen = Gegenstand, keine LED |
| `red_dark_frac` | 0.4 | so viel der Umgebung muss dunkel sein (schwarzes Ampelgehäuse), 0 = aus |
| `obstacle_dist` | 0.30 | Notbremse ab diesem Abstand (m) |
| `obstacle_check` | true | Notbremse an/aus (RM03 hat kein LiDAR → `false`) |
| `szenario` | normal | `einsatz` = RTW, darf bei Rot fahren |

Während es läuft: `ros2 param set /line_follower red_val_min 220`.

## Ohne Panel (einzelne Fenster)

| Befehl | Zweck |
|---|---|
| `scripts/kamera.sh` | Kamera starten, offen lassen |
| `scripts/test.sh` | Linienfolger im Testmodus: zeigt Erkennung, **fährt nicht** |
| `scripts/fahren.sh` | Fährt los. **Strg+C = Not-Aus** |
| `scripts/stopp.sh` | Not-Aus von außen |
| `scripts/objekte.sh` | KI-Erkennung |
| `scripts/check.sh` | Schnellcheck: Name, MAC, Agent, Akku, LiDAR, Kamera. Fährt nicht. |
| `scripts/update.sh` | neueste Version holen |

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

## Tests (ohne Roboter)

```bash
python3 tests/test_ampel.py
python3 tests/test_schilder.py
python3 tests/test_line_follower.py
```

## Bekannte Probleme

- Alle Roboter heißen `yahboom` → nur einen gleichzeitig einschalten (Domain-ID 30 für alle).
- Waagrechte schwarze Flächen (Kreuzungen, Kabel) verwirren den Linienfolger.
- RM03: ein Rad schwächer, Board liefert keine Sensordaten → Notbremse meldet "KEIN LIDAR" und fährt nicht los.
- Gamepad und Fahrprogramm senden beide auf `/cmd_vel` → nicht gleichzeitig benutzen.
- Ampel-LEDs sind klein: Erkennung erst aus der Nähe zuverlässig.

Siehe auch `docs/roboter.md`, `docs/ki.md`, `docs/greifarm.md`.
