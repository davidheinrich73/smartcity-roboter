#!/bin/bash
# Kartograf einzeln starten (das Panel macht das automatisch). Faehrt nicht, bewegt den Arm nicht.
# Baut die Karte in karten/<name>/ und verbessert sie bei jeder Fahrt.
#   scripts/kartograf.sh                       Karte "smartcity"
#   scripts/kartograf.sh -p karte:=werkstatt   andere Karte
source "$(dirname "$0")/env.sh"
python3 "$REPO_DIR/kartograf/kartograf.py" --ros-args "${PARAMS[@]}" "$@"
