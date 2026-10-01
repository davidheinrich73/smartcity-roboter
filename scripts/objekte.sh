#!/bin/bash
# KI-Objekterkennung (Personen, Autos, ...) einzeln starten. Faehrt nicht.
# Kamera muss laufen. Das Panel startet das automatisch mit.
source "$(dirname "$0")/env.sh"
python3 "$REPO_DIR/erkennung/objekte.py" --ros-args "$@"
