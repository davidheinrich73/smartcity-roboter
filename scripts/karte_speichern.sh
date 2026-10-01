#!/bin/bash
# Speichert die aktuelle Karte ins Repository: maps/NAME.pgm + maps/NAME.yaml
# Aufruf: scripts/karte_speichern.sh smartcity
# karte_erstellen.sh muss dabei noch laufen.
source "$(dirname "$0")/env.sh"
NAME="${1:-smartcity}"
mkdir -p "$REPO_DIR/maps"
ros2 run nav2_map_server map_saver_cli -f "$REPO_DIR/maps/$NAME"
ls -l "$REPO_DIR/maps/$NAME".*
echo
echo "Damit die anderen Roboter die Karte bekommen:"
echo "  cd $REPO_DIR && git add maps && git commit -m \"Karte $NAME\" && git push"
