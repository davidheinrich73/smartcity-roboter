#!/bin/bash
# Laedt eine gespeicherte Karte (nur anzeigen/bereitstellen, noch KEINE Navigation).
# Aufruf: scripts/karte_laden.sh smartcity
# UNGETESTET auf dem M3 Pro.
source "$(dirname "$0")/env.sh"
NAME="${1:-smartcity}"
MAP="$REPO_DIR/maps/$NAME.yaml"
if [ ! -f "$MAP" ]; then
    echo "Karte $MAP nicht gefunden. Vorhanden:"; ls "$REPO_DIR/maps"; exit 1
fi
echo "Lade $MAP auf Topic /map ..."
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:="$MAP" &
SERVER=$!
sleep 3
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
    -p node_names:="['map_server']" -p autostart:=true &
MANAGER=$!
trap "kill $SERVER $MANAGER 2>/dev/null" EXIT
echo "Karte bereit. In RViz Topic /map anzeigen. Beenden mit Strg+C."
wait
