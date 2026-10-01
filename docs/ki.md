# KI-Zentrale

## Aufbau

```
 Kamera ──► Kamera-KI (YOLO) ──┐
            + Ampelfarbe        │
            + Ersatz: LED, Form │
 LiDAR  ───────────────────────►├──► Entscheider ──► /ki/befehl ──► Linienfolger ──► Motoren
 Tiefenkamera (Bodenmodell) ───┘    (ki/entscheider.py)  fahren/langsam/stopp   (lenkt nach Linie,
                                         │                                       eigene Notbremse)
                                         └──► Arm (nur wenn erlaubt, nur im Stand)
```

- **ki/zentrale.py**: ROS-Programm. Wertet die Sinne aus und schickt 10× pro Sekunde einen Befehl mit Grund.
- **ki/entscheider.py**: reine Entscheidungslogik, ohne ROS testbar (`python3 tests/test_entscheider.py`).
- **line_follower/line_follower.py**: lenkt nach der schwarzen Linie und fährt nur, wenn die KI es erlaubt.

## Wie entscheidet die KI? (wichtigstes zuerst)

| # | Situation | Entscheidung |
|---|---|---|
| 1 | LiDAR: etwas näher als **12 cm** | **STOPP sofort, ohne KI-Prüfung** (Eigenschutz) |
| 2 | kein LiDAR oder keine Kamerabilder | STOPP (wer nichts sieht, fährt nicht) |
| 3 | LiDAR: etwas zwischen 12 und **45 cm** | KI prüft mit Kamera-KI **und** Tiefenkamera: bestätigt → STOPP bis frei. Nicht bestätigt (z. B. Haus am Rand) → LANGSAM. Ist der Arm freigegeben: im Stand kurz mit dem Arm genauer hinschauen |
| 4 | Kamera-KI sieht Person/Auto/… im Weg | STOPP |
| 5 | Ampel rot oder gelb | STOPP, bis **grün** erkannt wird (oder die Ampel 4 s nicht mehr zu sehen ist) |
| 6 | Stoppschild | 3 s halten, dann weiter. Dasselbe Schild löst erst wieder aus, wenn es 2 s weg war |

Im Szenario **RTW-Einsatz** werden 5 und 6 übergangen (Sonderrechte), 1–4 gelten weiter.

**Warum hat die Notbremse unter 12 cm das letzte Wort und nicht die KI?** Die KI kennt nur 80 Dinge (siehe unten). Einen Schuh, ein Kabel oder einen Karton kennt sie nicht. Müsste sie jede LiDAR-Meldung erst bestätigen, würde der Roboter in alles fahren, was sie nicht kennt. Darum: ganz nah = sofort stopp. Die Tiefenkamera bestätigt außerdem auch unbekannte Dinge ("ragt etwas aus dem Boden?").

## Die Sinne

- **Kamera-KI**: YOLOv8n (`models/yolov8n.onnx`), 80 Dinge aus dem COCO-Datensatz, u. a. Person, Auto, Bus, Fahrrad, Hund, Ampel, Stoppschild. Findet sie eine **Ampel**, schaut sie im Kasten nach, welche Lampe leuchtet (`ampel.farbe_in_box`).
- **Ersatz, wenn die KI nichts findet**: Die SmartCity-Ampeln sind winzig (5-mm-LEDs). Ob YOLO sie als Ampel erkennt, ist **unklar**. Deshalb sucht die LED-Erkennung zusätzlich leuchtende rote/gelbe/grüne Punkte in einem schwarzen Gehäuse (`line_follower/ampel.py`, einstellbar mit `scripts/ampel_kalibrieren.sh`). Ebenso die Achteck-Form für Stoppschilder (`line_follower/schilder.py`). Im Panel steht bei jeder Erkennung die Quelle (z. B. "KI 88 %" oder "LED-Erkennung").
- **LiDAR**: kleinster Abstand im Sektor ±30° vorne (Richtung "vorne" je LiDAR in `config/roboter.yaml`, im Panel einstellbar).
- **Tiefenkamera**: lernt, wie der Boden "normal" aussieht, solange der LiDAR nichts meldet. Ist ein Bereich danach deutlich näher, steht dort etwas. **Ungetestet mit der echten Kamera.**

## Getestet (in der VM, nicht auf dem Roboter)

- Entscheidungslogik: `tests/test_entscheider.py` (33 Fälle).
- Komplette Simulation mit ROS 2 Humble: `scripts/simulation.sh pruefen`. Ein simulierter Roboter fährt eine Runde mit roter Ampel (wird grün), Stoppschild, Hindernis und Haus am Rand. Alles bestanden.
- YOLO hat sogar das **gezeichnete** Stoppschild der Simulation erkannt (88 %). Personen und Autos auf echten Fotos ebenfalls.

## Grenzen (ehrlich)

- Das Modell wurde mit **echten Fotos** trainiert. Spielfiguren und Spielzeugautos erkennt es nur teilweise oder gar nicht. Für die SmartCity braucht man wahrscheinlich ein **eigenes Modell**: ca. 100–300 Fotos der echten Figuren, Autos, Schilder und Ampeln machen (Panel → KI-Sicht, oder `ampel_kalibrieren.sh` → Taste `s`), markieren (z. B. Roboflow oder Label Studio), mit Ultralytics trainieren, als ONNX exportieren, nach `models/` legen und die Namen in `erkennung/yolo.py` anpassen.
- Rechenzeit auf dem Jetson ist noch nicht gemessen. Das Panel zeigt sie an ("ms/Bild"). Die KI nutzt nur 2 Prozessorkerne (`kerne`), damit der Linienfolger nicht ruckelt.

## Modell neu exportieren (auf einem Laptop mit Internet)

```bash
pip install ultralytics
yolo export model=yolov8n.pt format=onnx imgsz=320 opset=12
```

Auf dem Roboter wird das Modell mit **onnxruntime** ausgeführt (`installieren.sh` bietet die Installation an). Das OpenCV von Ubuntu 22.04 (Version 4.5) kann es nicht laden, das ist getestet.

## Roboter-eigene KI (Yahboom)

Yahboom liefert "KI-Großmodell"-Funktionen mit (Sprachmodell, auf RM03 liegt Dify). Ob dort etwas **lokal** läuft oder nur über einen Cloud-Dienst mit API-Schlüssel, ist ungeprüft: `scripts/ki_suchen.sh > ki.txt`. Ein Sprachmodell ist für schnelle Fahrentscheidungen ohnehin zu langsam (Sekunden statt Millisekunden). Es könnte später für die Drohnen-Schnittstelle nützlich sein ("fahr zu Ort X"). **API-Schlüssel nie ins Repository!**

Lizenz: YOLOv8 von Ultralytics steht unter AGPL-3.0.
