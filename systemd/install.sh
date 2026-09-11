#!/bin/bash
# Install (or refresh) the rover console as a boot service. Needs sudo.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
UNIT=rover-console.service

# A console started by hand still owns :8080. Starting the service on top of it
# would fail to bind, exit 1, and be restarted by systemd until it hits the
# rate limit -- which reads like the service is broken when it is only being
# blocked. Catch it here instead, and ignore a console that IS the service
# (this script is also how you upgrade an already-installed one).
HOLDER=$(ss -lptnH 'sport = :8080' 2>/dev/null | grep -o 'pid=[0-9]*' | cut -d= -f2 | head -1)
if [ -n "$HOLDER" ] && ! systemctl is-active --quiet "$UNIT" 2>/dev/null; then
    echo "A rover console is already running by hand (pid $HOLDER) and owns port 8080."
    echo
    echo "Stop it first -- Ctrl+C in its terminal, or:   kill $HOLDER"
    echo "The stack and camera keep running; the service adopts them when it starts."
    exit 1
fi

sudo cp "$HERE/$UNIT" /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable "$UNIT"
sudo systemctl restart "$UNIT"

echo "waiting for the console to bind :8080 ..."
for i in $(seq 1 20); do
    if curl -s -m 2 -o /dev/null http://127.0.0.1:8080/; then OK=1; break; fi
    sleep 1
done

echo
systemctl --no-pager --lines=0 status "$UNIT" | head -6
echo
if [ -n "${OK:-}" ]; then
    echo "Console:  http://$(hostname -I | awk '{print $1}'):8080"
else
    echo "The console did not answer on :8080. What went wrong:"
    echo "    journalctl -u rover-console -n 40 --no-pager"
    exit 1
fi
echo "Logs:     journalctl -u rover-console -f"
echo "Stop:     sudo systemctl stop rover-console"
echo "Disable:  sudo systemctl disable rover-console"
