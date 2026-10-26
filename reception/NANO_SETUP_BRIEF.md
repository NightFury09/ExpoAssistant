# Task brief — deploy the "reception check-in" station on this Orin Nano

> Paste this whole file as your first message in the SSH session on the Nano.
> You (the assistant) are running on/into an **NVIDIA Jetson Orin Nano**. There is
> **no prior context** — everything you need is below.

## What you're building
This Orin Nano + a USB webcam is a **reception desk** for a trade-show booth. A
visitor checks in here: the webcam captures their face and details
(name / company / email / phone), and this app sends that to the **demo booth** —
a **separate** NVIDIA Jetson AGX Orin — so when the visitor later walks up to the
booth's RealSense camera, the booth **greets them by name**. Only face *embeddings*
(math vectors) + text cross the LAN — **never photos**.

The booth is already built and running. It exposes a token-secured
`POST /enroll_remote` endpoint that this reception app posts to. **Your job:** get
the reception app running on this Nano and verify an end-to-end check-in works.

## Key facts (reference)
- **Booth (already running):** AGX Orin, reachable at **`http://booth.local:8000`**
  over mDNS (fallback IP `http://192.168.3.224:8000` — DHCP, may change), SSH user `rptech`.
- **Confirm the booth + its live IP:** `curl -s http://booth.local:8000/whoami` →
  JSON `{service:"realsense-booth", hostname, mdns, ip, port, enrolled}`.
- **Enroll endpoint:** `POST http://booth.local:8000/enroll_remote` (JSON).
- **Shared token:** `rover-reception` (env `RS_ENROLL_TOKEN`) — must match on both ends.
- **Reception app:** `reception.py`, check-in UI on port `8001`. Its default booth
  URL is already `http://booth.local:8000`.
- **Required models:** `yunet.onnx` + `sface.onnx` — must be the SAME two files the
  booth uses (that's what makes embeddings comparable). Bundled in `reception/models/`.
- **Code location on the booth:** `rptech@booth.local:~/realsense_demo/reception/`
  (has `reception.py`, `models/`, `DEPLOY.md`, this brief).

> NOTE ON `booth.local`: it only works if the booth was renamed to `booth`
> (`sudo hostnamectl set-hostname booth` on the AGX). If `curl http://booth.local:8000/whoami`
> fails but `curl http://192.168.3.224:8000/whoami` works, the booth wasn't renamed —
> just use the IP everywhere below (pass `RS_BOOTH_URL=http://192.168.3.224:8000`).

## Step 1 — confirm this Nano can reach the booth
```bash
curl -s -m5 http://booth.local:8000/whoami; echo        # try the name first
curl -s -m5 http://192.168.3.224:8000/whoami; echo      # fallback: the IP
```
At least one must return JSON. Use whichever works as your `RS_BOOTH_URL` below.
If both fail, this Nano can't reach the booth — fix the network (same Wi-Fi/LAN) first.

## Step 2 — copy the reception folder onto the Nano
```bash
scp -r rptech@booth.local:~/realsense_demo/reception ~/reception    # or use the IP; or a USB stick
ls -la ~/reception ~/reception/models    # expect reception.py + yunet.onnx + sface.onnx
```

## Step 3 — dependencies + verify the face API
```bash
sudo apt update && sudo apt install -y python3-pip
python3 -c "import cv2,numpy; print('cv2', cv2.__version__)" 2>/dev/null || pip3 install numpy opencv-python
python3 -c "import cv2; cv2.FaceDetectorYN_create; cv2.FaceRecognizerSF_create; print('face API OK', cv2.__version__)"
```
The last line must print OK (needs OpenCV ≥ 4.5.4 with YuNet + SFace). If it raises
`AttributeError`: `pip3 install --upgrade opencv-python` (or `opencv-contrib-python`),
or use JetPack's `sudo apt install -y python3-opencv` — whichever provides BOTH functions.

## Step 4 — webcam
```bash
ls /dev/video*        # if none: replug; `groups | grep video`; try another port
```
Default index is `0`; if higher, pass `RS_WEBCAM=N` in Step 5.

## Step 5 — run it
```bash
cd ~/reception
RS_ENROLL_TOKEN=rover-reception python3 reception.py            # uses booth.local by default
# if booth.local didn't work in Step 1, use the IP instead:
# RS_BOOTH_URL=http://192.168.3.224:8000 RS_ENROLL_TOKEN=rover-reception python3 reception.py
```
On start it prints a line like `booth OK: <hostname> @ <ip> (N enrolled)` — that means
it reached the booth. Then get this Nano's IP: `hostname -I | awk '{print $1}'`.

## Step 6 — verify end-to-end
1. On a browser/touchscreen open `http://<nano-ip>:8001/`. You should see the webcam
   feed; standing in front, the caption counts "capturing N/6 angles."
2. Type a test name, tap **Check me in**. Reception log prints
   `checked in '<name>' -> booth stored N`; the UI shows/speaks "Welcome, <name>!"
3. Confirm it reached the booth: `curl -s http://booth.local:8000/leads.csv` — the
   test name should appear (with company/email/phone).
4. (At the booth) on the **Greet** tab, that person should now be recognized by name.

## Config / env vars
| var | default | meaning |
|---|---|---|
| `RS_BOOTH_URL` | `http://booth.local:8000` | booth address (name or IP) |
| `RS_ENROLL_TOKEN` | `rover-reception` | must match the booth |
| `RS_WEBCAM` | `0` | `/dev/videoN` index |
| `RS_PORT` | `8001` | reception UI port |
| `RS_WIDTH` / `RS_HEIGHT` | `1280` / `720` | webcam size (drop to `640`/`480` if it struggles) |
| `RS_ENROLL_VIEWS` | `6` | face angles captured per person |
| `RS_MIN_FACE_PX` | `90` | ignore faces smaller than this (px) |

## Troubleshooting
- **`booth.local` fails but IP works:** the booth wasn't renamed to `booth`. Use the IP
  (`RS_BOOTH_URL=http://192.168.3.224:8000`), or rename the booth (on the AGX): `sudo hostnamectl set-hostname booth`.
- **"Booth unreachable" / POST fails:** `curl …/whoami`; ensure same LAN.
- **403 / nothing stored:** token mismatch — `RS_ENROLL_TOKEN` must equal the booth's (`rover-reception`).
- **No `/dev/video*`:** replug; add user to `video` group; other USB port; set `RS_WEBCAM`.
- **`FaceDetectorYN_create` AttributeError:** OpenCV too old/missing — see Step 3.
- **Camera opens but no faces:** lighting/distance; face must be ≥ `RS_MIN_FACE_PX` px.
- **Recognized at reception but NOT at the booth:** cross-camera (webcam vs RealSense).
  Tune the **booth's** `RS_COSINE_MATCH` (default 0.363, try 0.34) ON THE BOOTH — not here.

## Auto-start on boot (only after it's verified)
```bash
( crontab -l 2>/dev/null; echo "@reboot sleep 20 && cd ~/reception && RS_ENROLL_TOKEN=rover-reception python3 reception.py >> ~/reception/reception.log 2>&1" ) | crontab -
```

## Must-know
- The two model files MUST be the identical `yunet.onnx` / `sface.onnx` from the
  booth (bundled in `reception/models/`). Different models ⇒ no recognition.
- Only embeddings + text go to the booth; no images. Data auto-wipes on the booth.

## Definition of done
A test check-in on the Nano's `:8001` UI appears in the booth's `leads.csv`, and the
booth's Greet tab recognizes that person by name. Report the OpenCV version used, the
webcam index, the booth URL that worked (name vs IP), and whether booth recognition
worked (for threshold tuning).
