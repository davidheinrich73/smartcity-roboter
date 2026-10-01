#!/bin/bash
# Einmalig auf jedem Roboter ausfuehren (nach git clone):  scripts/installieren.sh
# Legt das Desktop-Symbol "SmartCity Panel" an. Aendert nichts an Yahboom-Dateien oder am Autostart.
REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
chmod +x "$REPO_DIR"/scripts/*.sh

# Roboter-Namen merken (alle heissen "yahboom", so kann man sie unterscheiden)
if [ ! -f "$HOME/roboter_name" ]; then
    read -r -p "Welcher Roboter ist das? (z. B. RM02, leer = ueberspringen) " NAME
    [ -n "$NAME" ] && echo "$NAME" > "$HOME/roboter_name"
fi

DESKTOP=$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")
mkdir -p "$DESKTOP"
DATEI="$DESKTOP/SmartCity-Panel.desktop"
cat > "$DATEI" <<EOD
[Desktop Entry]
Type=Application
Name=SmartCity Panel
Comment=Startet Kamera, KI und das Control-Panel
Exec=bash -c '"$REPO_DIR/scripts/panel.sh"; echo; read -r -p "Enter zum Schliessen"'
Icon=applications-engineering
Terminal=true
Categories=Utility;
EOD
chmod +x "$DATEI"
# Ubuntu: Datei als "vertrauenswuerdig" markieren, sonst fragt es beim Doppelklick
gio set "$DATEI" metadata::trusted true 2>/dev/null
echo "Desktop-Symbol angelegt: $DATEI"
echo "Falls Ubuntu beim ersten Doppelklick fragt: Rechtsklick -> 'Start erlauben'."

# Kurzer Check, ob alles da ist
python3 -c "import cv2, yaml" 2>/dev/null && echo "Python-Pakete: ok" || echo "WARNUNG: python3-opencv oder python3-yaml fehlt"
[ -f "$REPO_DIR/models/yolov8n.onnx" ] && echo "KI-Modell: ok" || echo "KI-Modell fehlt (models/yolov8n.onnx)"

# onnxruntime fuehrt das KI-Modell aus (OpenCV 4.5 von Ubuntu 22.04 kann es nicht)
if python3 -c "import onnxruntime" 2>/dev/null; then
    echo "onnxruntime: ok"
else
    read -r -p "onnxruntime fehlt (fuer die KI-Erkennung). Jetzt fuer diesen Benutzer installieren? (j/n) " ok
    if [ "$ok" = "j" ]; then
        # numpy<2: sonst funktioniert cv_bridge von ROS Humble nicht mehr
        pip3 install --user "onnxruntime<1.20" "numpy<2"
    fi
fi
