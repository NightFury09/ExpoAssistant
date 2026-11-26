# RealSense Edge AI Perception Station

The booth demo: one Intel RealSense D455, one Jetson, a browser as the screen.
Everything runs on the Jetson, offline. Modes (tabs on screen): Safety Zone,
Inspect, Box Vol (live object dimensions from depth), Greet (face recognition
with depth liveness), 3D Scan (live point cloud). On the Orin Nano it runs in
RealSense-only mode; AI Detect (NanoOWL) stays on the AGX Orin.

| File | What it is |
| --- | --- |
| `station.py` | The whole app: depth pipeline, modes, web server on :8000 |
| `web/` | The booth screen (HTML/JS/CSS, vendored three.js and QR code) |
| `run_nano.sh` | Start / stop / logs for the Orin Nano (Docker) |
| `Dockerfile.nano` | Image for the Orin Nano (RealSense-only) |
| `Dockerfile.cuda` | Image for the AGX Orin |
| `NANO_MIGRATION.md` | How the booth was moved onto the Nano, and what works there |

## Run (Orin Nano)

```bash
cd realsense_station
./run_nano.sh          # start (or restart); needs the image realsense-nano:demo
./run_nano.sh logs     # follow the log
./run_nano.sh stop     # stop
```

Open `http://<jetson-ip>:8000/`. `curl localhost:8000/status` shows
`"cam_ok": true` when the camera is streaming.

Only one program can use the D455 at a time. Stop the booth before using the
rover's camera (free roam), and the other way round.

## Not in this repo

- **`models/yunet.onnx` and `models/sface.onnx`**: the OpenCV Zoo face detector
  (YuNet) and face recogniser (SFace). Put them in `realsense_station/models/`
  before using the Greet tab. They need OpenCV 4.8 or newer (the Dockerfile
  installs a matching wheel).
- **`data/`**: created at run time. It holds enrolled faces, captured leads
  (names, emails, phone numbers) and `key.bin`, the key that encrypts them.
  It is ignored by git on purpose. Back it up privately if you need it.
- **Build logs** from the image builds.

## Settings

Environment variables read by `station.py`: `RS_DATA_DIR`, `RS_MODEL_DIR`,
`RS_ENROLL_TOKEN` (shared secret for remote enrolment, default
`rover-reception`; set your own if the booth is on a shared network),
`RS_WIDTH` / `RS_HEIGHT` (default 640x480), `RS_PORT`. `run_nano.sh` passes
the resolution and port through.
