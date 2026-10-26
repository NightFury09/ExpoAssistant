# Reception station (Orin Nano + webcam) — deploy guide

A visitor checks in at reception → the RealSense demo booth greets them by name.
Only face **embeddings** (vectors) + text details cross the network, never photos.

## What's in this folder
- `reception.py` — the whole reception app (webcam + YuNet/SFace + check-in UI).
- `models/yunet.onnx`, `models/sface.onnx` — the SAME face models the booth uses
  (this is what makes embeddings match between the two devices).

## 1. Copy this folder to the Orin Nano
From the booth (AGX Orin), or via a USB stick:
```bash
scp -r ~/realsense_demo/reception  <nano-user>@<nano-ip>:~/reception
```

## 2. Install dependencies on the Nano
```bash
sudo apt update && sudo apt install -y python3-pip
pip3 install opencv-python numpy          # or: opencv-contrib-python
```
Plug in the USB webcam and confirm it shows up (usually `/dev/video0`):
```bash
ls /dev/video*
```

## 3. Run it (point it at the booth)
The reception defaults to the booth's mDNS name **`http://booth.local:8000`** (works
only if the booth was renamed to `booth`: `sudo hostnamectl set-hostname booth` on the
AGX). `RS_ENROLL_TOKEN` **must match** the booth's (default `rover-reception`).
```bash
cd ~/reception
RS_ENROLL_TOKEN=rover-reception python3 reception.py
# if booth.local doesn't resolve, use the IP:
# RS_BOOTH_URL=http://192.168.3.224:8000 RS_ENROLL_TOKEN=rover-reception python3 reception.py
```
Confirm the booth first with `curl -s http://booth.local:8000/whoami` (or the IP) —
it returns the booth's hostname, live IP, and enrolled count.
Then open the reception touchscreen browser at:
```
http://<nano-ip>:8001/
```

## 4. Use it
1. Visitor stands in front of the webcam — it captures ~6 face angles automatically
   ("look around gently").
2. They type name (+ company / email / phone) and tap **Check me in**.
3. It posts to the booth and shows/speaks "Welcome, NAME!"
4. At the demo booth (Greet tab / walk-up), the RealSense recognises them and
   greets them by name. Their details also appear in the booth's **Leads CSV**.

## Environment variables
| var | default | meaning |
|-----|---------|---------|
| `RS_BOOTH_URL` | `http://booth.local:8000` | booth address — mDNS name (survives IP changes) or the IP |
| `RS_ENROLL_TOKEN` | `rover-reception` | shared secret; must equal the booth's |
| `RS_WEBCAM` | `0` | `/dev/videoN` index |
| `RS_WIDTH` / `RS_HEIGHT` | `1280` / `720` | webcam capture size |
| `RS_PORT` | `8001` | reception web UI port |
| `RS_ENROLL_VIEWS` | `6` | face angles banked per person |
| `RS_MIN_FACE_PX` | `90` | ignore faces smaller than this |

## Auto-start on boot (optional)
Add a cron `@reboot` (like the booth) once it's working:
```bash
( crontab -l 2>/dev/null; echo "@reboot sleep 20 && cd ~/reception && RS_BOOTH_URL=http://192.168.3.224:8000 RS_ENROLL_TOKEN=rover-reception python3 reception.py >> ~/reception/reception.log 2>&1" ) | crontab -
```

## Accuracy note (important)
Enrolling on a **webcam** and recognising on the **RealSense RGB** = two different
cameras/lighting/angles. SFace handles this, but cross-camera accuracy is lower
than same-camera — most people are recognised, a few may not be. To improve it:
- Keep both stations **well-lit and frontal**.
- The 6-angle capture already helps; ask visitors to look around slightly.
- If the booth misses people, lower the booth's match threshold a little:
  `RS_COSINE_MATCH=0.34` (default 0.363) — but not too low, or you get false matches.
- Best case is same camera model at both ends; a second RealSense at reception
  would match most reliably (but a webcam is fine for the demo).
