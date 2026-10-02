# Greifarm

## Auf RM02 geprüft (scripts/arm_suchen.sh)

- `/arm6_joints` gibt es, Typ `arm_msgs/msg/ArmJoints` mit `joint1`–`joint6` und `time` (alles `int16`). Subscriber = Motorboard (über den micro-ROS-Agent).
- Die Gamepad-Steuerung (`yahboomcar_ctrl/yahboom_joy_M3Pro.py`, startet automatisch) sendet ebenfalls auf `/arm6_joints` und `/arm_joint`. **Gamepad und Panel nicht gleichzeitig am Arm benutzen.**
- Startstellung der Gamepad-Steuerung: **90, 120, 10, 20, 90, 0**. Damit lief unser Linienfolger → in `config/arm.yaml` als `fahrstellung` (Greifer 30 statt 0).
- Yahbooms eigener Linienfolger (`largemodel_arm/follow_line.py`): **90, 90, 12, 20, 90, 0** → als `yahboom_linie`. Achtung: Dort wird Servo 1 als `180 - Wert` geschickt (bei 90 egal).
- Weitere Yahboom-Topics: `set_joint5`, `set_joint6`, `Curjoints` (`arm_interface/CurJoints`).

## Schnittstelle (Quelle)

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
- **Posen einlernen**: Regler einstellen, mit "Pose anfahren" prüfen, dann den passenden Knopf drücken. Gespeichert wird in `config/lokal/arm.yaml`. Die Datei ist nicht im Repository, bleibt also bei Updates erhalten. Standardwerte stehen in `config/arm.yaml`. Danach die KI neu starten (System).

| Pose | Wofür | Hinweis |
|---|---|---|
| **fahrstellung** | Linienfolgen. **Als Erstes!** | Startstellung der Gamepad-Steuerung ist voreingestellt. Kameramaße danach nachmessen (`docs/linie.md`) |
| pruefblick | KI schaut bei unklarer LiDAR-Meldung genauer hin | ohne diese Pose schaut die KI nicht genauer hin |
| blick_links / blick_rechts | Zebrastreifen und Karte: umschauen | ohne eigene Pose: Fahrstellung mit Servo 1 um ±45° gedreht. **Richtung ungeprüft**: zeigt "links" nach rechts, beide neu einlernen |
| greifen | Arm unten vor dem Roboter, Greifer offen um den Gegenstand | Gegenstand mittig, Abstand zur Robotermitte messen → `greif_abstand` (Standard 20 cm) |
| greifen_hoch | Gegenstand angehoben | |
| ablegen | seitlich neben der Straße, dort wird losgelassen | weit genug weg, damit er nicht wieder im Weg liegt |

- Schalter **"KI darf den Arm bewegen"**. Was die KI dann tut, hängt davon ab, welche Posen da sind:
  - Prüfblick: genauer hinschauen,
  - Blick links/rechts: am Zebrastreifen und für die Karte umschauen,
  - Greifen, Greifen hoch, Ablegen: aufheben.

  Immer nur **im Stand** und nur, solange das Fahrprogramm **fährt** (nicht im TEST, nicht nach STOPP).

## Aufheben (KI, ki/ablaeufe.py)

1. Tiefenkamera (oder LiDAR + Tiefenkamera) meldet etwas Kleines **mitten auf der Fahrbahn**.
2. Seitlich ausrichten, bis es mittig 30 cm vor dem Roboter liegt (Mecanum-Räder fahren seitwärts, max. 4 cm/s).
3. Langsam heranfahren (3 cm/s), bis es zwischen den Greiferbacken liegt (`greif_abstand`).
4. Greifer auf → zu → Arm hoch → zur Seite (ablegen) → loslassen → Fahrstellung.
5. Prüfen: Liegt es noch da, nicht nochmal versuchen, sondern warten.

**Nie aufgehoben wird:** Lebewesen und Fahrzeuge. Ebenso alles, was breiter als 8 cm, höher als 12 cm oder mehr als 8 cm neben der Linie liegt, und alles in den ersten 3 s nach einem Zebrastreifen (dort stehen Fußgänger). Dann wartet der Roboter.

In der Simulation klappt das mit dem Holzwürfel. Mit dem echten Arm ist es **ungetestet**. Vor dem ersten echten Versuch die Posen einzeln im Panel anfahren und prüfen, ob der Greifer den Würfel wirklich umschließt. Würfel dazu von Hand an die Greifstelle legen. Den ersten echten Versuch mit kleinem Tempo machen, eine Hand am STOPP.
