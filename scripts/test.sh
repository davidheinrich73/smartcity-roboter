#!/bin/bash
# Linienfolger im TESTMODUS: zeigt nur an, was erkannt wird, faehrt NICHT.
# Zusaetzliche Einstellungen anhaengen, z. B.:  scripts/test.sh -p red_val_min:=220
source "$(dirname "$0")/env.sh"
export DISPLAY="${DISPLAY:-:0}"
python3 "$REPO_DIR/line_follower/line_follower.py" --ros-args -p drive:=false "$@"
