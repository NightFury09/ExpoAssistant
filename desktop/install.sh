#!/bin/bash
# Put the rover console in the app menu as "Rover Console": its own window on
# the rover's screen, talking to the console over loopback, so it works with
# Wi-Fi off. Needs no sudo. The console service itself is systemd/install.sh.
#
#   ./install.sh                  app menu entry
#   ./install.sh --autostart      ...and open it full screen at every login
#   ./install.sh --no-autostart   stop opening it at login (menu entry stays)
#
# Then: Activities -> type "rover" -> right-click -> Add to Favourites to pin it.
#
# --autostart opens the app at LOGIN. For it to appear at BOOT with nobody
# touching the rover, the desktop must also log in by itself -- see README.md.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
MODE=${1:-}
APPS="$HOME/.local/share/applications"
ENTRY="$APPS/rover-console.desktop"
AUTOSTART="$HOME/.config/autostart/rover-console.desktop"

if [ "$MODE" = "--no-autostart" ]; then
    rm -f "$AUTOSTART"
    echo "Removed $AUTOSTART -- the app no longer opens at login."
    exit 0
fi

/usr/bin/python3 -c "import gi; gi.require_version('Gtk','3.0'); gi.require_version('WebKit2','4.0')" 2>/dev/null || {
    echo "GTK 3 / WebKit2GTK 4.0 Python bindings are missing:"
    echo "    sudo apt install python3-gi gir1.2-webkit2-4.0"
    exit 1; }

chmod +x "$HERE/rover_app.py"
mkdir -p "$APPS"
# StartupWMClass matches what rover_app.py sets with GLib.set_prgname, so the
# dock shows the running window under this launcher instead of a second icon.
cat > "$ENTRY" <<EOF
[Desktop Entry]
Type=Application
Name=Rover Console
Comment=Drive, map and navigate the rover -- no network needed
Exec=/usr/bin/python3 $HERE/rover_app.py
Icon=$HERE/rover.svg
Terminal=false
Categories=Utility;
Keywords=rover;robot;ros;console;dashboard;
StartupWMClass=rover-console
Actions=fullscreen;

[Desktop Action fullscreen]
Name=Open full screen
Exec=/usr/bin/python3 $HERE/rover_app.py --fullscreen
EOF

command -v desktop-file-validate >/dev/null && desktop-file-validate "$ENTRY"
command -v update-desktop-database >/dev/null && update-desktop-database "$APPS" 2>/dev/null || true

echo "Installed: $ENTRY"
echo "Open it:   Activities -> \"Rover Console\""
echo "  or run:  $HERE/rover_app.py [--fullscreen]"

if [ "$MODE" = "--autostart" ]; then
    # GNOME runs every entry in ~/.config/autostart when the session starts.
    # No delay is needed for the console: if it is not answering yet, the app
    # shows "Connecting..." and retries, and starts the service if it is down.
    mkdir -p "$(dirname "$AUTOSTART")"
    cat > "$AUTOSTART" <<EOF
[Desktop Entry]
Type=Application
Name=Rover Console
Comment=Open the rover console full screen at login
Exec=/usr/bin/python3 $HERE/rover_app.py --fullscreen
Icon=$HERE/rover.svg
Terminal=false
X-GNOME-Autostart-enabled=true
EOF
    echo
    echo "Autostart: $AUTOSTART"
    echo "           opens full screen at every login (F11 leaves full screen)."
    # Read-only check. Turning automatic login on is left to a person: it
    # removes the password from the rover's desktop, which is a decision, not
    # an install step.
    if grep -qE "^\s*AutomaticLoginEnable\s*=\s*[Tt]rue" /etc/gdm3/custom.conf 2>/dev/null; then
        echo "           Automatic login is on, so it also opens at boot."
    else
        echo
        echo "Automatic login is OFF: at boot the rover stops at the login screen,"
        echo "and the app opens only after someone types the password."
        echo "To open it at boot unattended, turn automatic login on:"
        echo "    Settings -> Users -> Unlock -> Automatic Login: on"
    fi
fi

systemctl --user is-active --quiet rover-console 2>/dev/null ||
    echo "Note: the rover-console service is not running; the app will start it."
