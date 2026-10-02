# Karte (Kartograf)

Dateien: `kartograf/kartograf.py` (ROS-Programm), `lib/karte.py` (Rechnung, ohne ROS testbar: `python3 tests/test_karte.py`).
Der Kartograf **fährt nicht** und bewegt den Arm nicht. Er startet automatisch mit der KI (Panel) oder einzeln mit `scripts/kartograf.sh`.

## Was in der Karte steckt (aus welchem Sensor)

| Schicht | Sensor | Inhalt |
|---|---|---|
| Belegung | beide LiDARs | 2D-Gitter, 2 cm je Zelle: frei / Wand oder Hindernis / unbekannt |
| Bodenfoto | Kamera | Foto des Bodens von oben (1 cm je Pixel): Linien, Zebrastreifen |
| 3D-Punkte | Tiefenkamera | alles, was aus dem Boden ragt (Würfel, Schilder, Figuren), als 2-cm-Würfelchen mit Farbe |
| Wände in 3D | LiDAR | belegte Zellen als 14 cm hohe Säulen |
| Marker | KI | Ampel, Stoppschild, Zebrastreifen, Einbahnstraße, Hindernis (nur aktuelle Fahrt) |

## Wo bin ich? (Lokalisierung)

1. **Mitrechnen**: Radgeschwindigkeit (`/odom_raw`) + Drehrate vom Lagesensor (`/imu/data_raw`). Wird mit der Zeit ungenau. In der Simulation sind es nach einer Runde 18 cm Fehler.
2. **Scan-Abgleich**: Jeder LiDAR-Scan wird ein paar Zentimeter und Grad hin und her geschoben, bis er am besten auf die bisherige Karte passt. Das korrigiert den Fehler aus 1 laufend. In der Simulation bleibt der Fehler unter 6 cm, im Mittel 1,3–2,9 cm.
3. **Neue Fahrt**: Ist schon eine Karte da, wird der erste Scan in allen Drehungen über die ganze Karte geschoben (globale Suche, unter 1 s). Erst wenn die Stelle **eindeutig** passt, wird weiter eingetragen. Vorher steht im Panel "suche Position in der Karte …".

**Jede Fahrt macht die Karte genauer:** Jede Messung macht eine Zelle "sicherer". Was öfter gesehen wird, wird fester. Was nur einmal im Weg stand (z. B. eine Person), verblasst wieder. Das Bodenfoto wird gemittelt, die 3D-Punkte werden gezählt (erst ab 2 Messungen angezeigt).

**Umschauen:** Steht `umschauen_auto:=true`, bittet der Kartograf die KI, kurz anzuhalten und mit der Kamera nach links und rechts zu schauen. Das passiert höchstens alle 1,5 m und nur dort, wo das Bodenfoto noch Lücken hat. Im Panel gibt es dafür auch einen Knopf. Die KI macht das nur während der Fahrt, nur wenn der Arm freigegeben ist und die Posen "Blick links/rechts" da sind. Am Zebrastreifen schaut sie sowieso.

## Dateien (`karten/<name>/`, nicht im Repository)

| Datei | Inhalt |
|---|---|
| `karte.npz` | alles zum Weiterbauen (wird alle 30 s und beim Beenden gespeichert) |
| `karte.png` | 2D-Karte, 1 Pixel = 1 cm (Panel → Karte → "Bild herunterladen") |
| `karte.ply` | 3D-Karte (Panel → "3D herunterladen"), öffnen mit MeshLab oder CloudCompare |
| `karte.json`, `wolke.bin` | für das Panel |

**Neue Karte:** Panel → Karte → "Neue Karte". Die alte wird **umbenannt** (`karten/smartcity_alt_<Datum>`), nicht gelöscht.
Andere Karte (z. B. andere Stadt): `scripts/kartograf.sh -p karte:=werkstatt` bzw. Panel mit `--karte werkstatt`.

## Einstellungen (in `config/lokal/roboter.yaml` unter `/**:` → `ros__parameters:`)

| Name | Standard | Bedeutung |
|---|---|---|
| `lidar_x` | 0.0 | LiDAR sitzt so weit vor der Robotermitte (m). **Nachmessen** |
| `arm_basis_x` | 0.05 | Drehachse von Servo 1 vor der Robotermitte (m), für Bilder beim Umschauen. Geschätzt |
| `kamera_*` | wie Linienfolger | Kameramaße, siehe `docs/linie.md` |
| `umschauen_auto` | false | KI um Umschauen bitten, wo die Karte Lücken hat |
| `odom_topic` / `imu_topic` | `/odom_raw` / `/imu/data_raw` | Quellen fürs Mitrechnen |

## Ungeprüft auf dem echten Roboter

- Ob `/odom_raw` Geschwindigkeiten (`twist`) liefert. Wenn nicht, rechnet der Kartograf nur mit IMU + Scan-Abgleich. Das reicht bei langsamer Fahrt, beim schnellen Drehen wird es ungenau.
- Ob das Tiefenbild auf das Farbbild ausgerichtet ist. Sonst sind die Farben der 3D-Punkte verschoben.
- Wo die LiDARs genau sitzen (`lidar_x`).
- Wie groß die echte SmartCity ist. Die Karte reicht ±8 m um den ersten Startpunkt.
