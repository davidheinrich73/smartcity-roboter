#!/bin/bash
# Schnellcheck eines Roboters. Aendert nichts, faehrt nicht.
source "$(dirname "$0")/env.sh"
echo "===== SYSTEM =====";   hostname; hostname -I; cat ~/Version.txt 2>/dev/null; echo "Domain-ID: $ROS_DOMAIN_ID"
echo "===== AGENT ======";   pgrep -fa micro_ros_agent | head -1 || echo "AGENT LAEUFT NICHT"
echo "===== NODES ======";   ros2 node list
echo "===== AKKU =======";   timeout 5 ros2 topic echo /battery --once 2>/dev/null || echo "keine Akkudaten"
echo "===== LIDAR ======";   for t in /scan0 /scan1; do echo -n "$t: "; timeout 4 ros2 topic hz $t 2>/dev/null | grep -m1 average || echo "keine Daten"; done
echo "===== KAMERA =====";   ros2 topic list | grep -i image || echo "Kamera nicht gestartet (scripts/kamera.sh)"
echo "===== CMD_VEL ====";   ros2 topic info /cmd_vel
