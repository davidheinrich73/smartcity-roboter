#!/bin/bash
# Startet ALLES: Kamera, KI-Erkennung und das Control-Panel im Vollbild.
# Wird vom Desktop-Symbol "SmartCity Panel" aufgerufen (siehe installieren.sh).
# Browser schliessen (oder "Panel beenden") = Roboter haelt an, alles wird beendet.
# Vom Laptop erreichbar machen (Vorsicht, dann kann jeder im Netz fahren):  scripts/panel.sh --netz
source "$(dirname "$0")/env.sh"
export DISPLAY="${DISPLAY:-:0}"
PORT=8080
URL="http://localhost:$PORT"

if curl -s -o /dev/null "$URL/api/status"; then
    echo "Panel laeuft schon, oeffne nur den Browser."
    LAEUFT_SCHON=1
else
    echo "Starte Panel (Kamera und KI starten automatisch mit) ..."
    python3 "$REPO_DIR/panel/panel.py" --port "$PORT" --autostart "$@" &
    PANEL=$!
    # Beim Beenden dieses Skripts (auch Fenster schliessen) das Panel sauber beenden
    trap 'kill -INT $PANEL 2>/dev/null; wait $PANEL 2>/dev/null' EXIT
    for _ in $(seq 30); do
        curl -s -o /dev/null "$URL/api/status" && break
        kill -0 $PANEL 2>/dev/null || { echo "Panel ist abgestuerzt, siehe Meldungen oben."; exit 1; }
        sleep 0.5
    done
fi

# Browser suchen. Eigenes Profil -> eigenes Fenster, Skript wartet, bis es geschlossen wird.
for B in chromium-browser chromium google-chrome firefox; do
    command -v "$B" >/dev/null || continue
    ECHT=$(readlink -f "$(command -v "$B")")
    PROFIL="$HOME/.cache/smartcity-panel-$B"
    # Snap-Programme duerfen nicht in versteckte Ordner schreiben
    if [[ "$ECHT" == /snap/* || "$ECHT" == /usr/bin/snap ]]; then
        PROFIL="$HOME/snap/${B%-browser}/common/smartcity-panel"
    fi
    mkdir -p "$PROFIL"
    echo "Oeffne $URL mit $B (Vollbild verlassen: F11, schliessen: Alt+F4)"
    if [ "$B" = firefox ]; then
        firefox --new-instance --profile "$PROFIL" --kiosk "$URL"
    else
        "$B" --user-data-dir="$PROFIL" --app="$URL" --start-fullscreen --no-first-run --noerrdialogs
    fi
    [ -n "$LAEUFT_SCHON" ] && exit 0
    echo "Browser geschlossen -> Panel wird beendet, Roboter haelt an."
    exit 0
done

echo "Kein Browser gefunden. Panel im Browser oeffnen: $URL"
echo "Beenden: Strg+C"
wait
