#!/bin/bash
# Sammelt alles, was man zum Reparieren der Karten-Skripte braucht. Aendert nichts, faehrt nicht.
# Am besten zweimal ausfuehren: einmal so, einmal waehrend karte_erstellen.sh laeuft.
# Ausgabe in Datei:  scripts/karte_diagnose.sh > diagnose.txt 2>&1
source "$(dirname "$0")/env.sh"
echo "===== PAKETE (slam, nav, map, bringup, lidar) ====="
ros2 pkg list | grep -iE "slam|nav2_map|map_server|mapping|bringup|lidar|laser|cartographer|gmapping" || echo "keine"
echo "===== LAUNCH-DATEIEN MIT slam/map/nav IM NAMEN ====="
for ws in "$HOME/yahboomcar_ws" "$HOME/M3Pro_ws"; do
    find "$ws" -path "*/share/*/launch/*" \( -iname "*slam*" -o -iname "*map*" -o -iname "*nav*" -o -iname "*bringup*" \) 2>/dev/null
done
echo "===== TOPICS ====="
ros2 topic list -t
echo "===== NODES ====="
ros2 node list
echo "===== LIDAR-NACHRICHT (frame_id, Winkel, Reichweite) ====="
for t in /scan /scan0 /scan1; do
    echo "--- $t"
    timeout 5 ros2 topic echo "$t" --once --no-arr 2>/dev/null | grep -E "frame_id|angle_min|angle_max|range_min|range_max" || echo "keine Daten"
done
echo "===== TF (Koordinatensysteme, 5 s) ====="
cd /tmp && timeout 8 ros2 run tf2_tools view_frames >/dev/null 2>&1 && grep -oE '"[^"]+" -> "[^"]+"' /tmp/frames_*.gv 2>/dev/null | sort -u || echo "view_frames ging nicht"
echo "===== /map ====="
timeout 5 ros2 topic echo /map --once --no-arr 2>/dev/null | grep -E "frame_id|resolution|width|height" || echo "keine Karte auf /map"
