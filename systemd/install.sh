#!/bin/bash
# Install the rover console as a service that starts at boot.
#
# Default is a USER service: it needs no root, which matters because the only
# privileged things it would otherwise want are already satisfied -- rptech is
# in dialout, video and plugdev, so the lidar, ESP32 and camera are reachable
# without it. `loginctl enable-linger` is what makes a user service start at
# boot with nobody logged in.
#
#   ./install.sh            user service, no sudo   <- what you want
#   ./install.sh --system   system-wide, needs sudo
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
MODE=${1:-}

port_holder() { ss -lptnH 'sport = :8080' 2>/dev/null | grep -o 'pid=[0-9]*' | cut -d= -f2 | head -1; }

if [ "$MODE" = "--system" ]; then
    SC="sudo systemctl"; UNIT_SRC="$HERE/rover-console.service"
    ACTIVE_CHECK="systemctl is-active --quiet rover-console"
else
    SC="systemctl --user"; UNIT_SRC="$HERE/rover-console-user.service"
    ACTIVE_CHECK="systemctl --user is-active --quiet rover-console"
fi

# A console started by hand owns :8080, and the service cannot bind on top of
# it. Starting anyway would fail, be restarted by systemd, and hit the rate
# limit -- which reads like a broken service when it is only being blocked.
HOLDER=$(port_holder)
if [ -n "$HOLDER" ] && ! $ACTIVE_CHECK 2>/dev/null; then
    echo "A rover console is already running by hand (pid $HOLDER) and owns port 8080."
    echo
    echo "Stop it first -- Ctrl+C in its terminal, or:   kill $HOLDER"
    echo "The stack and camera keep running; the service adopts them when it starts."
    exit 1
fi

mkdir -p "$HOME/AGX_Orin_Backup/rover_project/logs"

if [ "$MODE" = "--system" ]; then
    sudo cp "$UNIT_SRC" /etc/systemd/system/rover-console.service
else
    mkdir -p "$HOME/.config/systemd/user"
    cp "$UNIT_SRC" "$HOME/.config/systemd/user/rover-console.service"
    # Without lingering, a user service stops when you log out and never starts
    # at boot -- which defeats the whole point.
    loginctl enable-linger "$USER" || {
        echo "Could not enable lingering, so this will NOT start at boot."
        echo "Either run:  sudo loginctl enable-linger $USER"
        echo "or install system-wide:  ./install.sh --system"; }
fi

$SC daemon-reload
$SC enable rover-console.service
$SC restart rover-console.service

echo "waiting for the console to bind :8080 ..."
for i in $(seq 1 25); do
    if curl -s -m 2 -o /dev/null http://127.0.0.1:8080/; then OK=1; break; fi
    sleep 1
done

echo
$SC --no-pager --lines=0 status rover-console.service | head -6
echo
if [ -z "${OK:-}" ]; then
    echo "The console did not answer on :8080. What went wrong:"
    echo "    tail -40 ~/AGX_Orin_Backup/rover_project/logs/console.log"
    exit 1
fi
echo "Console:  http://$(hostname -I | awk '{print $1}'):8080"
if [ "$MODE" = "--system" ]; then
    echo "Status:   systemctl status rover-console"
    echo "Stop:     sudo systemctl stop rover-console"
else
    echo "Status:   systemctl --user status rover-console"
    echo "Stop:     systemctl --user stop rover-console"
fi
echo "Logs:     tail -f ~/AGX_Orin_Backup/rover_project/logs/console.log"
