#!/bin/bash
# Startet die Kartierung mit LiDAR (slam_toolbox, laut Yahboom-Anleitung M3 Pro).
# Danach in einem zweiten Fenster: scripts/karte_anzeigen.sh
# Mit dem Gamepad LANGSAM durch alle Strassen fahren.
# Speichern (dieses Fenster offen lassen!): scripts/karte_speichern.sh NAME
source "$(dirname "$0")/env.sh"
ros2 launch slam_mapping slam_toolbox.launch.py
