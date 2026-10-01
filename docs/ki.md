# KI-Objekterkennung

## Was ist eingebaut

- `erkennung/yolo.py`: findet Objekte mit dem Modell **YOLOv8n** (`models/yolov8n.onnx`, 80 Dinge, z. B. Person, Auto, Bus, Fahrrad, Ampel, Stoppschild).
- `erkennung/objekte.py`: ROS-Node. Meldet auf `/erkennung/hindernis` (True/False), ob etwas Wichtiges **im Weg** ist. Der Linienfolger hält dann an.
- "Im Weg" heißt: Kasten mittig im Bild (20–80 % der Breite), Unterkante in der unteren Bildhälfte, mindestens 15 % der Bildhöhe hoch (= nah). Alles als Parameter einstellbar.

Start: automatisch über das Panel, oder einzeln `scripts/objekte.sh`.

## Getestet (in einer VM, nicht auf dem Roboter)

- Beispielfoto mit Bus und 3 Personen: alle erkannt, gleiche Ergebnisse wie das Original-Programm (Ultralytics).
- Mit OpenCV 4.5 (wie Ubuntu 22.04) lädt OpenCV das Modell **nicht**. Deshalb wird **onnxruntime** benutzt (`scripts/installieren.sh` fragt, ob es installiert werden soll).

## Grenzen (ehrlich)

- Das Modell wurde mit **echten Fotos** trainiert. Spielfiguren, Lego-Männchen und Spielzeugautos erkennt es nur teilweise oder gar nicht.
  Für die SmartCity braucht man wahrscheinlich ein **eigenes trainiertes Modell**: ca. 100–300 Fotos der echten Figuren/Autos/Schilder machen, markieren (z. B. mit Roboflow oder Label Studio), mit Ultralytics trainieren, als ONNX exportieren, nach `models/` legen.
- Die wichtigste Sicherheit ist die **LiDAR-Notbremse** im Linienfolger. Sie hält bei JEDEM Gegenstand vorne an, egal ob die KI ihn kennt.

## Modell neu exportieren (auf einem Laptop mit Internet)

```bash
pip install ultralytics
yolo export model=yolov8n.pt format=onnx imgsz=320 opset=12
```
Datei nach `models/yolov8n.onnx` kopieren. Für ein eigenes Modell andere Namen in `erkennung/yolo.py` (`namen=`) eintragen.

## Roboter-eigene KI (Yahboom)

Yahboom bewirbt den M3 Pro mit "KI-Großmodell"-Funktionen. Auf RM03 liegt Dify (Plattform für KI-Chatbots). Ob ein Modell **lokal** läuft oder nur über einen Cloud-Dienst mit API-Schlüssel, ist **nicht geprüft**.
Prüfen: `scripts/ki_suchen.sh > ki.txt` auf dem Roboter ausführen und die Datei weitergeben. Das Skript zeigt nur Dateinamen, keine Schlüssel. **API-Schlüssel nie ins Repository!**

Lizenz: YOLOv8 von Ultralytics steht unter AGPL-3.0 (frei nutzbar, Code muss offen bleiben).
