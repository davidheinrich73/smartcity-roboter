#!/bin/bash
# Findet heraus, wie der Greifarm auf DIESEM Roboter angesteuert wird. Nur lesen, bewegt NICHTS.
# Ausgabe in Datei:  scripts/arm_suchen.sh > arm.txt 2>&1
source "$(dirname "$0")/env.sh"
echo "===== TOPICS mit arm/servo/joint/grip ====="
# --no-daemon: direkt fragen (der Hintergrunddienst liefert beim ersten Aufruf oft noch nichts)
ros2 topic list -t --no-daemon --spin-time 3 | grep -iE "arm|servo|joint|grip|claw" || echo "keine"
echo "===== /arm6_joints (alle 6 Servos) ====="
ros2 topic info /arm6_joints -v 2>&1 | grep -E "Type|Node name|Reliability|count" | head -12
echo "--- Nachrichtenaufbau:"
ros2 interface show arm_msgs/msg/ArmJoints 2>&1 | head -12
echo "===== /arm_joint (ein Servo) ====="
ros2 topic info /arm_joint 2>&1 | head -3
ros2 interface show arm_msgs/msg/ArmJoint 2>&1 | head -8
echo "===== SERVICES mit arm/servo ====="
ros2 service list -t --no-daemon --spin-time 3 | grep -iE "arm|servo|joint|grip" || echo "keine"
echo "===== Yahboom-Programme, die den Arm benutzen (nur Dateinamen) ====="
for ws in "$HOME/yahboomcar_ws" "$HOME/M3Pro_ws"; do
    grep -rlE "arm6_joints|ArmJoints|arm_joint" "$ws/src" 2>/dev/null | head -20
done
echo "===== Launch-Dateien mit arm/moveit/grasp ====="
for ws in "$HOME/yahboomcar_ws" "$HOME/M3Pro_ws"; do
    find "$ws" -path "*/launch/*" \( -iname "*arm*" -o -iname "*moveit*" -o -iname "*grasp*" \) 2>/dev/null | head -20
done
