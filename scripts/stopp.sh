#!/bin/bash
# NOT-AUS: beendet den Linienfolger und schickt Stopp an die Motoren.
source "$(dirname "$0")/env.sh"
pkill -f line_follower.py
ros2 topic pub -t 5 -r 10 /cmd_vel geometry_msgs/msg/Twist "{}" > /dev/null
echo "Stopp gesendet."
