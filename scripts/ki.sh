#!/bin/bash
# KI-Zentrale einzeln starten (das Panel macht das automatisch). Faehrt nicht selbst.
# Kamera muss laufen (scripts/kamera.sh).
source "$(dirname "$0")/env.sh"
python3 "$REPO_DIR/ki/zentrale.py" --ros-args --params-file "$REPO_DIR/config/roboter.yaml" "$@"
