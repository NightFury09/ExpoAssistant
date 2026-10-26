#!/usr/bin/env bash
# =============================================================================
# run_reception.sh — launch the RP tech reception check-in station.
#
# Starts reception.py (webcam face check-in UI on :8001) which POSTs face
# embeddings + visitor details to the demo booth's /enroll_remote.
#
# Safe to run by hand or from systemd (reception.service). All configuration is
# via environment variables with sensible defaults; override any of them by
# exporting before running, or via the systemd unit / /etc/reception.env.
#
#   Manual:   ./run_reception.sh
#   Override: RS_PORT=9001 RS_WIDTH=640 RS_HEIGHT=480 ./run_reception.sh
# =============================================================================
set -euo pipefail

# --- resolve to this script's directory so ./models/*.onnx always resolve -----
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

# --- configuration (env override -> default) ---------------------------------
# The RealSense station now runs on THIS Nano (:8000), so reception enrols faces
# into the local booth via localhost (same host -> no network hop, DHCP-proof).
# (Was http://192.168.3.224:8000, the old AGX booth, before the migration.)
export RS_BOOTH_URL="${RS_BOOTH_URL:-http://127.0.0.1:8000}"
export RS_ENROLL_TOKEN="${RS_ENROLL_TOKEN:-rover-reception}"   # MUST match the booth
export RS_WEBCAM="${RS_WEBCAM:-0}"                             # /dev/videoN index
export RS_PORT="${RS_PORT:-8001}"
export RS_WIDTH="${RS_WIDTH:-1280}"
export RS_HEIGHT="${RS_HEIGHT:-720}"
export RS_MODEL_DIR="${RS_MODEL_DIR:-models}"

PYTHON="${PYTHON:-python3}"
CAM_WAIT="${CAM_WAIT:-25}"        # seconds to wait for the USB webcam at startup
BOOTH_WAIT="${BOOTH_WAIT:-1}"     # booth reachability is only a warning, not fatal

TAG="reception-launch"
log()  { printf '%s [%s] %s\n' "$(date '+%F %T')" "$TAG" "$*" >&2; }
die()  { log "FATAL: $*"; exit 1; }

# --- preflight 1: models present ---------------------------------------------
for m in yunet.onnx sface.onnx; do
  [ -f "$RS_MODEL_DIR/$m" ] || die "missing model $RS_MODEL_DIR/$m — copy it from the booth's reception/models/ (must be the SAME file as the booth)."
done
log "models OK ($RS_MODEL_DIR/yunet.onnx, $RS_MODEL_DIR/sface.onnx)"

# --- preflight 2: python + OpenCV with the YuNet/SFace APIs ------------------
if ! "$PYTHON" - <<'PY' 2>/tmp/reception_cvcheck.err
import cv2
cv2.FaceDetectorYN_create          # needs OpenCV >= 4.5.4
cv2.FaceRecognizerSF_create
print("cv2", cv2.__version__)
PY
then
  log "$(cat /tmp/reception_cvcheck.err 2>/dev/null)"
  die "OpenCV missing or too old (need >=4.5.4 with FaceDetectorYN/FaceRecognizerSF). Try: pip3 install -U opencv-python  OR  sudo apt install -y python3-opencv"
fi
log "OpenCV OK: $("$PYTHON" -c 'import cv2;print(cv2.__version__)' 2>/dev/null)"

# --- preflight 3: wait for the USB webcam (non-fatal; app self-heals) ---------
# systemd can start us before the USB webcam has enumerated after a boot, so we
# give it a chance. If it never shows, we still start: reception.py's capture
# watchdog keeps retrying and shows "Connecting to camera..." until it appears.
cam_dev="/dev/video${RS_WEBCAM}"
waited=0
while [ ! -e "$cam_dev" ] && [ "$waited" -lt "$CAM_WAIT" ]; do
  sleep 1; waited=$((waited + 1))
done
if [ -e "$cam_dev" ]; then
  log "webcam OK: $cam_dev present after ${waited}s"
else
  log "WARNING: $cam_dev not present after ${CAM_WAIT}s — starting anyway; the app will reconnect when the camera appears (check the USB cable / try RS_WEBCAM=1)."
fi

# --- preflight 4: port availability (warn only) ------------------------------
if command -v ss >/dev/null 2>&1 && ss -ltn 2>/dev/null | grep -q ":${RS_PORT} "; then
  log "WARNING: port ${RS_PORT} already in use — if another reception is running, stop it (systemctl stop reception) or set RS_PORT."
fi

# --- preflight 5: booth reachability (warn only; reception also logs this) ----
if command -v curl >/dev/null 2>&1; then
  if curl -s -m "$((BOOTH_WAIT + 4))" "${RS_BOOTH_URL}/whoami" >/dev/null 2>&1; then
    log "booth OK: ${RS_BOOTH_URL} reachable"
  else
    log "WARNING: booth ${RS_BOOTH_URL} not reachable right now — starting anyway (visitors can't check in until it's back; verify the IP/LAN)."
  fi
fi

log "starting reception.py -> UI on http://$(hostname -I | awk '{print $1}'):${RS_PORT}/  (booth ${RS_BOOTH_URL})"
# exec so signals (SIGINT/SIGTERM from systemd) reach python for a clean stop
exec "$PYTHON" reception.py
