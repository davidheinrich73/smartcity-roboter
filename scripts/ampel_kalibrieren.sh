#!/bin/bash
# Ampel-Erkennung einstellen (Schieberegler). FAEHRT NICHT.
# Kamera muss laufen (scripts/kamera.sh). Tasten: s = Foto, w = Werte speichern, q = Ende
source "$(dirname "$0")/env.sh"
export DISPLAY="${DISPLAY:-:0}"
python3 "$REPO_DIR/line_follower/ampel_kalibrieren.py" "$@"
