#!/bin/bash
# Simulation OHNE Roboter (braucht ROS 2 Humble auf dem Rechner, z. B. Laptop).
#   scripts/simulation.sh           -> Simulator + Panel: http://localhost:8080, START druecken
#   scripts/simulation.sh pruefen   -> faehrt eine Runde und prueft automatisch alles
# Nutzt eine EIGENE Domain-ID (77), damit nie ein echter Roboter (Domain 30) mitfaehrt.
export ROS_DOMAIN_ID_OVERRIDE=77
source "$(dirname "$0")/env.sh"
if [ "$ROS_DOMAIN_ID" = "30" ]; then echo "Domain-ID 30 = Roboter. Abbruch."; exit 1; fi
command -v ros2 >/dev/null || { echo "ROS 2 nicht gefunden."; exit 1; }
if [ "$1" = "pruefen" ]; then
    exec python3 "$REPO_DIR/sim/pruefen.py"
fi
python3 "$REPO_DIR/sim/simulator.py" &
SIM=$!
trap 'kill -TERM $SIM 2>/dev/null' EXIT
sleep 2
python3 "$REPO_DIR/panel/panel.py" --port 8080 --autostart
