# Greifarm

## Schnittstelle (gefunden, auf unserem Roboter noch nicht geprüft)

Quelle: ein öffentlicher Treiber für den M3 Pro ([strands-labs/robots, Pull Request 3941](https://github.com/strands-labs/robots/pull/3941), Datei `strands_robots/drivers/yahboom_m3pro_wire.py`). Dessen Autor schreibt selbst, dass er keinen M3 Pro auf dem Tisch hatte. Die Werte stammen aus den Yahboom-Programmen.

| Topic | Typ | Inhalt |
|---|---|---|
| `/arm6_joints` | `arm_msgs/msg/ArmJoints` | alle 6 Servos: `joint1` … `joint6` in **ganzen Grad**, `time` = Dauer in ms |
| `/arm_joint` | `arm_msgs/msg/ArmJoint` | ein Servo: `id` (1–6), `joint` (Grad), `time` (ms) |

| Servo | Aufgabe | Bereich |
|---|---|---|
| 1 | Drehen unten | 0–180° |
| 2 | Schulter | 0–180° |
| 3 | Ellbogen | 0–180° |
| 4 | Handgelenk kippen | 0–180° |
| 5 | Handgelenk drehen | 0–270° |
| 6 | Greifer | 30° = zu, 180° = auf |

- Yahboom-Grundstellung zum Greifen: 90 / 120 / 0 / 0 / 90, Greifer auf.
- Das Board meldet die aktuelle Armstellung **nicht** zurück. Man weiß nur, was man zuletzt geschickt hat.
- Das Board hält eine Stellung, eine Nachricht reicht. Bei `/cmd_vel` ist das anders: Dort stoppt das Board nach 0,3 s ohne neuen Befehl.

**Zuerst prüfen** (nur lesen, bewegt nichts): `scripts/arm_suchen.sh > arm.txt`. Dort muss `/arm6_joints` mit Typ `arm_msgs/msg/ArmJoints` stehen und ein Subscriber (das Board).

## Im Panel (Seite "Arm")

- 6 Schieberegler, "Pose anfahren", Greifer auf/zu, Dauer der Bewegung.
- Gesperrt, solange das Fahrprogramm läuft (die Kamera sitzt am Arm).
- Werte außerhalb der Bereiche werden abgelehnt, nicht gekürzt.
- Posen speichern (`config/arm.yaml`):
  - **fahrstellung**: so steht der Arm beim Linienfolgen. **Als Erstes speichern!** Die Stellung, in der der Linienfolger auf RM02 funktioniert hat, ist nicht bekannt. Vorsichtig herantasten (kleine Schritte, Dauer 2–3 s).
  - **pruefblick**: Kamera schaut z. B. etwas weiter nach vorne/unten, um Hindernisse zu prüfen.
- Schalter "KI darf den Arm bewegen". Die KI schaut dann bei einer unklaren LiDAR-Meldung **im Stand** kurz in den Prüfblick und fährt danach zurück in die Fahrstellung, erst dann geht es weiter. In der Simulation getestet, mit dem echten Arm nicht.

## Idee "Gegenstand aufsammeln" (noch nicht gebaut)

LiDAR/KI hält vor dem Gegenstand → Tiefenkamera misst Abstand und Lage → Arm fährt hin, Greifer zu → Arm hoch → Fahrstellung → weiter. Dafür braucht man eine Umrechnung "Punkt im Kamerabild → Servowinkel" (inverse Kinematik). Yahboom hat dafür vermutlich Beispiele ("3D grasping", siehe `arm_suchen.sh`, Abschnitt Launch-Dateien). Sinnvoll erst, wenn Fahrstellung und Prüfblick sicher funktionieren.
