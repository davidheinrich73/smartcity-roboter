#!/bin/bash
# Gemeinsame ROS-Umgebung fuer alle Skripte. Wird von den anderen Skripten geladen.
source /opt/ros/humble/setup.bash
for ws in "$HOME/yahboomcar_ws" "$HOME/M3Pro_ws" "$HOME/mircoROS_agent"; do
    if [ -f "$ws/install/setup.bash" ]; then
        source "$ws/install/setup.bash"
    fi
done
export ROS_DOMAIN_ID=30

# Pfad zum Repository (Ordner ueber scripts/)
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export REPO_DIR
