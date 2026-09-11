#!/bin/bash
# Install (or refresh) the rover console as a boot service. Needs sudo.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
sudo cp "$HERE/rover-console.service" /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable rover-console.service
sudo systemctl restart rover-console.service
sleep 3
sudo systemctl --no-pager status rover-console.service | head -15
echo
echo "Console: http://$(hostname -I | awk '{print $1}'):8080"
echo "Logs:    journalctl -u rover-console -f"
