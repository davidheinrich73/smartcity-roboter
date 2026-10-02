#!/bin/bash
# Linienfolger FAEHRT. Not-Aus: Strg+C in diesem Fenster oder scripts/stopp.sh
# Zusaetzliche Einstellungen anhaengen, z. B.:  scripts/fahren.sh -p speed:=0.1
source "$(dirname "$0")/env.sh"
export DISPLAY="${DISPLAY:-:0}"
# Einstellungen: PARAMS kommt aus env.sh (roboter.yaml, ampel.yaml, lokal/roboter.yaml)
if ! ros2 node list 2>/dev/null | grep -q ki_zentrale; then
    echo "HINWEIS: KI-Zentrale laeuft nicht -> der Roboter bleibt stehen. Zweites Fenster: scripts/ki.sh"
fi
python3 "$REPO_DIR/line_follower/line_follower.py" --ros-args "${PARAMS[@]}" -p drive:=true "$@"
