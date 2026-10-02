#!/bin/bash
# Simulation OHNE Roboter (braucht ROS 2 Humble auf dem Rechner, z. B. Laptop).
#   scripts/simulation.sh           -> Simulator + Panel: http://localhost:8080, START druecken
#   scripts/simulation.sh pruefen   -> faehrt gut 2 Minuten und prueft automatisch alles
#                                      (Linie, Ampel, Zebrastreifen, Wuerfel aufheben, Stoppschild,
#                                       Einbahnstrasse, Karte)
# Nutzt eine EIGENE Domain-ID (77), damit nie ein echter Roboter (Domain 30) mitfaehrt,
# eigene Arm-Posen (sim/arm_sim.yaml, Kopie in /tmp) und eine eigene Karte "sim".
export ROS_DOMAIN_ID_OVERRIDE=77
source "$(dirname "$0")/env.sh"
if [ "$ROS_DOMAIN_ID" = "30" ]; then echo "Domain-ID 30 = Roboter. Abbruch."; exit 1; fi
command -v ros2 >/dev/null || { echo "ROS 2 nicht gefunden."; exit 1; }
export SMARTCITY_ARM_YAML=/tmp/smartcity_arm_sim.yaml
[ -f "$SMARTCITY_ARM_YAML" ] || cp "$REPO_DIR/sim/arm_sim.yaml" "$SMARTCITY_ARM_YAML"
if [ "$1" = "pruefen" ]; then
    exec python3 "$REPO_DIR/sim/pruefen.py"
fi
python3 "$REPO_DIR/sim/simulator.py" &
SIM=$!
trap 'kill -TERM $SIM 2>/dev/null' EXIT
sleep 2
python3 "$REPO_DIR/panel/panel.py" --port 8080 --autostart --karte sim
