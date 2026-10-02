# KI-Zentrale

## Aufbau

```
 Kamera ──► Kamera-KI (YOLO) ──┐
            + Ampelfarbe        │
            + Ersatz: LED, Form │
            + Zebrastreifen     │
            + Einfahrt verboten │
 LiDAR (Fahrschlauch) ─────────►├──► Entscheider ──► /ki/befehl ──► Linienfolger ──► Motoren
 Tiefenkamera (Bodenmodell) ───┘    (ki/entscheider.py)                (lenkt nach Linie, führt Manöver aus,
                                     + Abläufe (ki/ablaeufe.py)          eigene Notbremse)
                                         │
                                         └──► Arm (nur wenn freigegeben, nur im Stand, nur während der Fahrt)
```

- **ki/zentrale.py**: ROS-Programm. Wertet die Sinne aus und schickt 10× pro Sekunde einen Befehl mit Grund.
- **ki/entscheider.py**: Entscheidungslogik, ohne ROS testbar (`python3 tests/test_entscheider.py`).
- **ki/ablaeufe.py**: Abläufe aus mehreren Schritten: Zebrastreifen, Wenden, Aufheben, Umschauen.
- **line_follower/line_follower.py**: lenkt nach der schwarzen Linie, fährt nur, wenn die KI es erlaubt, und führt deren Manöver aus. Manöver sind z. B. wenden oder zum Greifen ausrichten. Sie sind begrenzt: max. 0,10 m/s vor, 0,08 m/s seitwärts, 1,2 rad/s drehen.

## Wie entscheidet die KI? (wichtigstes zuerst)

| # | Situation | Entscheidung |
|---|---|---|
| 1 | LiDAR: etwas näher als **12 cm** | **STOPP sofort, ohne KI-Prüfung** (Eigenschutz) |
| 2 | kein LiDAR oder keine Kamerabilder | STOPP (wer nichts sieht, fährt nicht) |
| – | ein Ablauf läuft (Zebrastreifen, Wenden, Aufheben, Umschauen) | der Ablauf bestimmt, 1 und 2 gelten trotzdem |
| 3 | LiDAR: etwas **im Fahrschlauch**, näher als **45 cm** | KI prüft mit Kamera-KI **und** Tiefenkamera: bestätigt → STOPP bis frei oder **aufheben** (siehe unten). Nicht bestätigt → LANGSAM, ab **20 cm** trotzdem STOPP. Ist der Arm freigegeben und die Pose "Prüfblick" da: im Stand kurz genauer hinschauen |
| 4 | Tiefenkamera allein: etwas ragt aus der Fahrbahn (z. B. Holzwürfel, zu flach für den LiDAR) | STOPP oder aufheben |
| 5 | Kamera-KI sieht Person/Auto/… im Weg | STOPP |
| 6 | Schild **"Einfahrt verboten"** (2 Bilder hintereinander) | **wenden**: auf der Stelle drehen, bis die Linie wieder genau vorne liegt (gemessen mit dem Lagesensor, mind. 150°) |
| 7 | **Zebrastreifen** näher als 33 cm | anhalten, Kamera nach **links**, dann nach **rechts** drehen (Arm), zurück. Sieht er eine Person oder steht etwas am Zebrastreifen (LiDAR, ±30 cm seitlich): warten und neu schauen. Sonst weiter. Ohne Arm-Freigabe: anhalten und geradeaus schauen |
| 8 | Ampel rot oder gelb | STOPP, bis **grün** erkannt wird (oder die Ampel 4 s nicht mehr zu sehen ist) |
| 9 | Stoppschild (nah, d. h. groß im Bild) | 3 s halten, dann weiter. Dasselbe Schild löst erst wieder aus, wenn es 2 s weg war |

Im Szenario **RTW-Einsatz** werden 8 und 9 übergangen (Sonderrechte), alles andere gilt weiter.

**Fahrschlauch:** Der Linienfolger meldet, wo die Linie vor dem Roboter verläuft. Die KI prüft nur den Streifen von **30 cm Breite** (`fahrschlauch_breite`, Roboterbreite + Rand, **nachmessen**) entlang dieser Linie. Dasselbe gilt für die Tiefenkamera. Früher war es ein gerader Kegel nach vorne. Dann hielt er in jeder Kurve vor Häusern am Rand, und in einer Kurve hielt die Tiefenkamera einen Fußgänger am Straßenrand für ein Hindernis.

**Warum hat die Notbremse unter 12 cm das letzte Wort und nicht die KI?** Die KI kennt nur 80 Dinge. Einen Schuh, ein Kabel oder einen Karton kennt sie nicht. Darum: ganz nah = sofort stopp.

## Aufheben – Sicherheitsregeln

Die KI hebt nur auf, wenn **alles** stimmt:
- Arm freigegeben (Panel → Arm), Posen Fahrstellung, Greifen, Greifen hoch und Ablegen eingelernt.
- Höchstens 8 cm breit und 12 cm hoch.
- Liegt **mitten auf der Straße** (höchstens 8 cm neben der Linie). Am Rand steht vielleicht jemand.
- Kein Lebewesen und kein Fahrzeug erkannt.
- **Kein Zebrastreifen** in den letzten 3 s gesehen. Dort stehen Fußgänger. In der Simulation hat die KI sonst einmal versucht, einen wartenden Fußgänger aufzuheben.
- Das Fahrprogramm **fährt** (START). Im TEST und nach STOPP bewegt die KI den Arm nie. Ein laufender Ablauf wird abgebrochen, der Arm bleibt stehen. Beim nächsten START fährt er zuerst in die Fahrstellung.

Ablauf: seitlich ausrichten (bis der Gegenstand mittig 30 cm vor ihm liegt) → langsam heranfahren → Greifer auf → zu → hoch → zur Seite (Pose "Ablegen") → loslassen → Fahrstellung → prüfen, ob der Weg frei ist. Liegt er noch da: nicht nochmal versuchen, warten.

## Die Sinne

- **Kamera-KI**: YOLOv8n (`models/yolov8n.onnx`), 80 Dinge aus dem COCO-Datensatz, u. a. Person, Auto, Bus, Fahrrad, Hund, Ampel, Stoppschild. Findet sie eine **Ampel**, schaut sie im Kasten nach, welche Lampe leuchtet.
- **Ersatz, wenn die KI nichts findet**:
  - LED-Erkennung für die winzigen SmartCity-Ampeln (`line_follower/ampel.py`, einstellbar mit `scripts/ampel_kalibrieren.sh`),
  - Achteck-Form für Stoppschilder (`line_follower/schilder.py`).
- **Einfahrt verboten**: roter Kreis mit waagrechtem weißem Balken (`schilder.finde_einfahrt_verboten`).
- **Zebrastreifen**: In der Draufsicht des Linienfolgers sind mindestens 4 gleich breite, parallele Balken nebeneinander (`lib/zebra.py`).
- **LiDAR**: nächster Punkt im Fahrschlauch (`lib/lidar.py`). Richtung "vorne" je LiDAR im Panel einstellbar.
- **Tiefenkamera**: Für jeden Bildpunkt wird die Höhe über dem Boden ausgerechnet. Den Boden lernt sie, solange der Weg frei ist (`lib/tiefe.py`). Erkennt auch den 4,5 cm hohen Würfel.
- **Wohin schaut die Kamera?** Die KI hört `/arm6_joints` mit (KI, Panel und Gamepad senden dorthin). Steht der Arm nicht in der Fahrstellung, werden Draufsicht, Zebrastreifen und Tiefe nicht ausgewertet. Linienfolger und Kartograf bekommen das ebenfalls mitgeteilt.

## Getestet (in der VM, nicht auf dem Roboter)

- Entscheidungslogik: `tests/test_entscheider.py`.
- Komplette Simulation mit ROS 2 Humble (`scripts/simulation.sh pruefen`), alles bestanden:
  - Ampel, Zebrastreifen mit Fußgänger,
  - Würfel aufheben und neben die Straße legen,
  - Stoppschild, Einbahnstraße (gewendet bei ca. 160°),
  - Karte.

## Grenzen (ehrlich)

- Das Modell wurde mit **echten Fotos** trainiert. Spielfiguren und Spielzeugautos erkennt es nur teilweise oder gar nicht. In der Simulation hielt es eine gezeichnete Fläche einmal kurz für eine Katze. Für die SmartCity braucht man wahrscheinlich ein **eigenes Modell**: ca. 100–300 Fotos der echten Figuren, Autos, Schilder und Ampeln machen, markieren (z. B. Roboflow oder Label Studio), mit Ultralytics trainieren, als ONNX exportieren, nach `models/` legen und die Namen in `erkennung/yolo.py` anpassen.
- Zebrastreifen-, Einbahn- und Würfelerkennung sind nur mit **gezeichneten** Bildern getestet. Echte Schilder können anders aussehen (Größe, Farbe, Licht).
- Die Rechenzeit auf dem Jetson ist noch nicht gemessen. Das Panel zeigt sie an ("ms/Bild").

## Modell neu exportieren (auf einem Laptop mit Internet)

```bash
pip install ultralytics
yolo export model=yolov8n.pt format=onnx imgsz=320 opset=12
```

Auf dem Roboter läuft das Modell mit **onnxruntime** (`installieren.sh` bietet die Installation an).

## Roboter-eigene KI (Yahboom)

Yahboom liefert "KI-Großmodell"-Funktionen mit (Sprachmodell, auf RM03 liegt Dify). Ob dort etwas **lokal** läuft, ist ungeprüft: `scripts/ki_suchen.sh > ki.txt`. Ein Sprachmodell ist für schnelle Fahrentscheidungen zu langsam. **API-Schlüssel nie ins Repository!**

Lizenz: YOLOv8 von Ultralytics steht unter AGPL-3.0.
