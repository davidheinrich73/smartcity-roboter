#!/bin/bash
# Startet die Kamera (Orbbec Dabai DCW2). Fenster offen lassen.
source "$(dirname "$0")/env.sh"
ros2 launch orbbec_camera dabai_dcw2.launch.py
