#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Launch the RealSense Edge AI Perception Station on the ORIN NANO.
#
# RealSense DEPTH demos only: Safety Zone, Inspect, Box Vol, Greet, 3D Scan.
# AI Detect (NanoOWL) and the Product AI assistant stay on the AGX Orin
# (192.168.3.224) after the hardware fork -- see NANO_MIGRATION.md.
#
#   ./run_nano.sh            start / restart
#   ./run_nano.sh stop       stop
#   ./run_nano.sh logs       follow logs
# ---------------------------------------------------------------------------
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

IMAGE="${IMAGE:-realsense-nano:demo}"     # built from Dockerfile.nano (or reuse the AGX image)
NAME="${NAME:-realsense_demo}"
PORT="${RS_PORT:-8000}"
AGX_IP="${AGX_IP:-192.168.3.224}"         # where the Product AI assistant lives
# D455 resolution: 640x480 is universally safe. On the Nano's USB-3 ports the
# D455 can do 1280x720@30 -- try `RS_WIDTH=1280 RS_HEIGHT=720 ./run_nano.sh`
# once the basic feed works (see NANO_MIGRATION.md).
RS_WIDTH="${RS_WIDTH:-640}"
RS_HEIGHT="${RS_HEIGHT:-480}"
IP="$(hostname -I | awk '{print $1}')"

case "${1:-start}" in
  stop) docker rm -f "$NAME" >/dev/null 2>&1 && echo "stopped $NAME" || echo "not running"; exit 0 ;;
  logs) exec docker logs -f "$NAME" ;;
esac

docker rm -f "$NAME" >/dev/null 2>&1 || true
docker run -d --restart unless-stopped \
  --name "$NAME" \
  --device-cgroup-rule 'c 81:* rmw' \
  --device-cgroup-rule 'c 189:* rmw' \
  --network host \
  -v /dev:/dev \
  -v "$HERE":/app -w /app \
  -e RS_REALSENSE_ONLY=1 \
  -e RS_ASSISTANT_URL="https://${AGX_IP}:5001" \
  -e RS_PORT="$PORT" \
  -e RS_WIDTH="$RS_WIDTH" -e RS_HEIGHT="$RS_HEIGHT" \
  -e RS_JPEG_QUALITY="${RS_JPEG_QUALITY:-90}" \
  -e RS_RETENTION_HOURS="${RS_RETENTION_HOURS:-72}" \
  "$IMAGE" \
  python3 station.py >/dev/null

echo "RealSense station (Nano, RealSense-only) starting."
echo "  Open:  http://${IP}:${PORT}/"
echo "  AI Detect + Product AI are on the AGX (${AGX_IP})."
echo "  Logs:  docker logs -f ${NAME}     Stop: ./run_nano.sh stop"
