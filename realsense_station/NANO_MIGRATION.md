# RealSense Booth → Orin Nano migration brief

Move the **RealSense depth demos** to the **Orin Nano at `192.168.3.112`**.
Everything here is self-contained — run it in a fresh SSH session on the Nano.

## The fork (what goes where)
| Runs on | Demos |
|---|---|
| **Orin Nano** `192.168.3.112` (this migration) | Safety Zone · Inspect · Box Vol · Greet · 3D Scan — the RealSense station on **:8000** |
| **AGX Orin** `192.168.3.224` (already done) | AI Detect / "Ask the Camera", Generative AI, Pose, **Product AI assistant** (:5001), Edge AI Station (:8020) |

So on the Nano the station runs with **`RS_REALSENSE_ONLY=1`**: it hides the AI Detect tab and never loads NanoOWL/TensorRT (no torch, no CUDA needed). The "Product AI" tab still shows and links to the AGX assistant (`https://192.168.3.224:5001`).

---

## 1. Get the package onto the Nano
On the **AGX** a package was prepared at `~/realsense_demo/nano_pkg.tgz` (code + web + models + this brief + `run_nano.sh` + `Dockerfile.nano`; no data/PII, no 31 GB image).

From your laptop or the AGX, copy it over, then unpack on the Nano:
```bash
# from the AGX:
scp ~/realsense_demo/nano_pkg.tgz rptech@192.168.3.112:~/
# on the Nano:
mkdir -p ~/realsense_demo && tar -xzf ~/nano_pkg.tgz -C ~/realsense_demo
cd ~/realsense_demo
```

## 2. Prereqs on the Nano
```bash
docker --version           # need Docker; install if missing
df -h /                    # need free disk (Path A wants ~35 GB; Path B ~5 GB)
cat /etc/nv_tegra_release  # note the L4T / JetPack version
lsusb | grep -i intel      # confirm the D455 is plugged in and enumerated
```

## 3. Get a runtime image — pick ONE path

### Path B — build a lean image (recommended: ~3–5 GB, no CUDA)
The depth demos only need librealsense + OpenCV + numpy + cryptography.
```bash
docker build -f Dockerfile.nano -t realsense-nano:demo .
```
Takes ~20–40 min (compiles librealsense once). The build self-checks that
`pyrealsense2` and `cv2` import at the end. Then use `IMAGE=realsense-nano:demo` (the default in `run_nano.sh`).

### Path A — reuse the AGX's proven image (fast, but 31 GB)
No build; guaranteed-working RealSense stack. Needs ~35 GB free on the Nano.
```bash
# on the AGX (this streams ~10 GB compressed straight to the Nano):
docker save nanoowl-realsense:demo | gzip | \
  ssh rptech@192.168.3.112 'gunzip | docker load'
# on the Nano, tell run_nano.sh to use it:
export IMAGE=nanoowl-realsense:demo
```
NanoOWL never loads because of `RS_REALSENSE_ONLY=1`, so the extra weight just sits on disk.

## 4. Run it
```bash
cd ~/realsense_demo
./run_nano.sh                 # starts on http://<nano-ip>:8000/
# (uses IMAGE=realsense-nano:demo by default; export IMAGE=... for Path A)
```
Open **http://192.168.3.112:8000/** — you should see Safety / Inspect / Box Vol / Greet / 3D Scan / Product AI tabs, **no AI Detect tab**.

`./run_nano.sh logs` to follow, `./run_nano.sh stop` to stop. It restarts on boot (`--restart unless-stopped`) once the Docker daemon is enabled: `sudo systemctl enable docker`.

## 5. Verify each demo
- **Safety Zone** — border turns green→amber→red as you approach; distance shown.
- **Greet** — face box + liveness; enrolment works (models/yunet.onnx + sface.onnx shipped).
- **Inspect / Box Vol** — hold an object/box; L×W×H shown. (Body-veto now uses geometry only — NanoOWL veto is AGX-only. Tune `RS_OBJ_MAX_FILL` if a torso is misread.)
- **3D Scan** — point cloud renders and orbits.
- **Product AI tab** — "Open" button goes to `https://192.168.3.224:5001/` (the AGX). Confirm the AGX assistant is up.
- Startup log shows `RealSense-only mode: AI Detect / NanoOWL disabled`.

## 6. Resolution (USB-3 win)
Default is a safe **640×480**. The Nano's USB-3 ports can run the D455 at **1280×720@30** (the AGX couldn't — it was USB-2). Once the basic feed works:
```bash
RS_WIDTH=1280 RS_HEIGHT=720 ./run_nano.sh
```
If the feed goes black / "couldn't resolve requests", revert to 640×480 (the cable/port is USB-2).

## 7. Update the things that pointed at the old booth (192.168.3.224 → 192.168.3.112)
- **AGX**: stop the RealSense booth there so it doesn't double-run —
  `docker rm -f realsense_demo` on the AGX, and remove/disable its boot autostart & the realsense `booth_watchdog` cron entry (the AGX now only runs `~/nvidia_demos/run.sh`).
- **Reception check-in** (if used): set `RS_BOOTH_URL=http://192.168.3.112:8000` so it enrols faces to the Nano booth.
- **Kiosk / signage / demo QR codes** that opened `http://192.168.3.224:8000` → repoint to `192.168.3.112:8000`. (Product Q&A QR codes point to the AGX assistant `:5001` and stay unchanged.)
- Consider `sudo hostnamectl set-hostname booth` on the Nano so it's reachable as `booth.local`.

## Troubleshooting
- **`Device or resource busy`** on the camera → another process holds the D455 (`docker rm -f realsense_demo` and re-run; make sure the AGX booth isn't also on the same camera).
- **Camera not found in container** → the `--device-cgroup-rule 'c 81:* rmw'` + `-v /dev:/dev` in `run_nano.sh` grant V4L2 access; make sure the D455 shows in `lsusb` on the host first.
- **`import pyrealsense2` fails** (Path B) → the build's final self-check would have failed; check `PYTHONPATH` in `Dockerfile.nano` matches where `make install` put `pyrealsense2*.so`.
- **Product AI tab points at the Nano** → confirm `run_nano.sh` passed `RS_ASSISTANT_URL=https://192.168.3.224:5001` (it does by default; override AGX IP with `AGX_IP=...`).
