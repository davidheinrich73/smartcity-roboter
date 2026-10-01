#!/bin/bash
# Zeigt die entstehende Karte in RViz auf dem Roboter-Bildschirm.
source "$(dirname "$0")/env.sh"
export DISPLAY="${DISPLAY:-:0}"
ros2 launch slam_mapping slam_view.launch.py
