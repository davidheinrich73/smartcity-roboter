#!/bin/bash
# Linienfolger im TESTMODUS: zeigt nur an, was erkannt wird, faehrt NICHT.
# Zusaetzliche Einstellungen anhaengen, z. B.:  scripts/test.sh -p red_val_min:=220
source "$(dirname "$0")/env.sh"
export DISPLAY="${DISPLAY:-:0}"
# Einstellungen laden: roboter.yaml (LiDAR) und, falls vorhanden, ampel.yaml
PARAMS=(--params-file "$REPO_DIR/config/roboter.yaml")
[ -f "$REPO_DIR/config/ampel.yaml" ] && PARAMS+=(--params-file "$REPO_DIR/config/ampel.yaml")
python3 "$REPO_DIR/line_follower/line_follower.py" --ros-args "${PARAMS[@]}" -p drive:=false "$@"
