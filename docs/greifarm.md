# Greifarm (noch NICHT eingebunden)

Der Code bewegt den Arm bisher **nirgends**. Grund: Es ist nicht bekannt, über welche Topics/Services der Arm angesteuert wird, und ein falscher Befehl kann Arm, Kamera (sitzt am Arm!) oder Menschen treffen.

## Nächste Schritte

1. Herausfinden, wie der Arm angesteuert wird (nur lesen, bewegt nichts):
   - Panel → Greifarm → "Arm-Topics anzeigen", oder
   - `source scripts/env.sh && ros2 topic list -t | grep -iE "arm|servo|joint|grip"`
   - Yahboom-Doku M3 Pro, Kapitel zum Arm (Beispielprogramme im Workspace `~/M3Pro_ws/src` ansehen).
2. Mit dem Ergebnis ein kleines Testprogramm schreiben, das **eine** Gelenkposition langsam anfährt. Nur mit Aufsicht, Hände weg vom Arm.
3. Wichtig: Die Kamera sitzt am Greifarm. Wird der Arm bewegt, ändert sich der Blick → Linienfolger und Ampelerkennung funktionieren nur in der **Fahrstellung**. Vor dem Fahren muss der Arm immer in die Fahrstellung zurück.

## Idee "Gegenstand aufsammeln"

LiDAR-Notbremse hält vor dem Gegenstand → Tiefenkamera misst Abstand und Position → Arm greift → Arm zurück in Fahrstellung → weiterfahren. Das ist ein größeres Projekt und erst sinnvoll, wenn Schritt 1 und 2 funktionieren.
