#!/bin/bash
# Linienfolger FAEHRT. Not-Aus: Strg+C in diesem Fenster oder scripts/stopp.sh
# Zusaetzliche Einstellungen anhaengen, z. B.:  scripts/fahren.sh -p speed:=0.1
source "$(dirname "$0")/env.sh"
export DISPLAY="${DISPLAY:-:0}"
# Einstellungen laden: roboter.yaml (LiDAR) und, falls vorhanden, ampel.yaml
PARAMS=(--params-file "$REPO_DIR/config/roboter.yaml")
[ -f "$REPO_DIR/config/ampel.yaml" ] && PARAMS+=(--params-file "$REPO_DIR/config/ampel.yaml")
python3 "$REPO_DIR/line_follower/line_follower.py" --ros-args "${PARAMS[@]}" -p drive:=true "$@"
